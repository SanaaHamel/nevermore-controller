#!/bin/bash
"true" '''\'
set -eu
set -o pipefail

FILE="$(readlink -f "$0")"
ROOT_DIR="$(dirname "$FILE")"

"$ROOT_DIR/setup-tool-env.bash"
"$ROOT_DIR/.venv/bin/python" "$FILE" "$@"

exit 0 # required to stop shell execution here
'''

# Script for logging sensors to local Graphite instance
#
# Copyright (C) 2023       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

__doc__ = """Script for logging sensors to local Graphite instance.

If you need a graphite instance you can set one up with the following commands:

sudo apt-get install docker.io --install-suggests
# expose web-server on port 81 instead of 80 b/c 80 is likely in use by Mainsail
sudo docker run -d\
 --name graphite\
 --restart=always\
 --env "GRAPHITE_WSGI_PROCESSES=1"\
 -p 81:80\
 -p 2003-2004:2003-2004\
 -p 2023-2024:2023-2024\
 -p 8125:8125/udp\
 -p 8126:8126\
 graphiteapp/graphite-statsd


You can then open the Graphite Dashboard via: http://localhost:81/dashboard
I suggest the following dashboard snippet (paste it via `Dashboard -> Edit Dashboard`):

```
[
  {
    "target": [
      "seriesByTag(\"name=gas\")"
    ],
    "vtitle": "VOC Index",
    "title": "VOC Index"
  },
  {
    "target": [
      "scale(seriesByTag(\"name=gas_raw\"),0.00001525902)"
    ],
    "title": "VOC Raw"
  },
  {
    "target": [
      "seriesByTag(\"name=temperature\", \"side=~(intake|exhaust)\")",
      "seriesByTag(\"name=temperature\", \"side!=~(intake|exhaust)\")",
    ],
    "title": "Temperature",
    "vtitle": "C"
  },
  {
    "target": [
      "seriesByTag(\"name=humidity\")"
    ],
    "vtitle": "Relative Humidity %",
    "title": "Humidity"
  },
  {
    "target": [
      "seriesByTag(\"name=pressure\")"
    ],
    "vtitle": "hPa",
    "title": "Pressure"
  }
]
```

"""

import asyncio
import dataclasses
import datetime
import enum
import json
import logging
import pickle
import re
import socket
import struct
import typing
import uuid
from collections import defaultdict
from dataclasses import dataclass
from typing import (
    Any,
    Callable,
    Coroutine,
    Dict,
    List,
    Optional,
    Tuple,
    TypeVar,
    Union,
    overload,
)
from uuid import UUID

import bleak
import typed_argparse as tap
import websockets.exceptions
import websockets.legacy.client as WSClient
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

if typing.TYPE_CHECKING:
    _LoggerAdapter = logging.LoggerAdapter[logging.Logger]
else:
    _LoggerAdapter = logging.LoggerAdapter

MOONRAKER_DEFAULT_PORT = 7125
GRAPHITE_DEFAULT_PICKLE_PORT = 2004
GRAPHITE_DEFAULT_RETENTION_RESOLUTION = 10

# object fields not in this list will not be logged
# (after some post processing, e.g. `exhaust_foobar` will check for `foobar`)
MOONRAKER_SENSOR_FIELDS_ALLOWED = {
    "gas",
    "gas_raw",
    "humidity",
    "pressure",
    "rpm",
    "speed",
    "target",  # used by heaters for target temperature
    "power",  # used by heaters for applied PWM %
    "temperature",
}

MOONRAKER_SENSOR_NAME_BLOCKED = [
    re.compile("temperature_sensor nevermore_.*_voc"),  # ignore common VOC plotters
]

NEVERMORE_CONTROLLER_NAMES = {"Nevermore", "Nevermore Controller"}
BOOTLOADER_NAME_PREFIX = "picowota "

BT_SCAN_GATHER_ALL_TIMEOUT = 10  # seconds


def short_uuid(x: int):
    assert 0 <= x <= 0xFFFF
    return UUID(f"0000{x:04x}-0000-1000-8000-00805f9b34fb")


UUID_SERVICE_ENVIRONMENTAL_SENSING = short_uuid(0x181A)
UUID_SERVICE_FAN = UUID("4553d138-1d00-4b6f-bc42-955a89cf8c36")
UUID_CHAR_DATA_AGGREGATE = UUID("75134bec-dd06-49b1-bac2-c15e05fd7199")
UUID_CHAR_FAN_AGGREGATE = UUID("79cd747f-91af-49a6-95b2-5b597c683129")
UUID_CHAR_CONFIG_REBOOT = UUID("f48a18bb-e03c-4583-8006-5b54422e2045")

_A = TypeVar("_A")


@dataclass
class Ip4Port:
    addr: str
    port: int

    @staticmethod
    def parse(default_addr: str, default_port: int):
        def go(raw: str) -> Ip4Port:
            parts = raw.split(":", 1)
            return Ip4Port(
                parts[0] if parts[0] != "" else default_addr,
                int(parts[1]) if 1 < len(parts) else default_port,
            )

        return go


# HACK: want `enum.StrEnum`, but that's not available on older versions we need to support
class Side(enum.Enum):
    # name must be lowercase
    intake = "intake"
    exhaust = "exhaust"


@dataclass
class SystemSnapshot:
    name: str
    sensors: Dict[str, Dict[Tuple[str, Optional[Side]], float]]
    timestamp: Optional[datetime.datetime] = None


GraphiteLogger = Callable[[SystemSnapshot], Coroutine[Any, Any, None]]


def graphite_connection(dst: Ip4Port, sampling_period: float) -> GraphiteLogger:
    assert 0 < sampling_period

    if sampling_period < GRAPHITE_DEFAULT_RETENTION_RESOLUTION / 2:
        print(
            f"WARN - `--sampling-period` of {sampling_period} sec is less than half the default graphite retention period ({GRAPHITE_DEFAULT_RETENTION_RESOLUTION} sec)."
        )
        print(
            "If you need a more fine-grain retention period then you'll need to modify your graphite installation's `/opt/graphite/conf/storage-schemas.conf`"
        )

    print(f"connecting to graphite: tcp://{dst.addr}:{dst.port}")
    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sk.connect((dst.addr, dst.port))

    async def log(snapshot: SystemSnapshot) -> None:
        def series_name(sensor_name: str, field: str, side: Optional[Side]) -> str:
            # FIXME: escape `;` and any `~` prefix?
            side_tag = "" if side is None else f";side={side.value}"
            return f"{field};source={snapshot.name};sensor={sensor_name}{side_tag}"

        timestamp = (snapshot.timestamp or datetime.datetime.now()).strftime("%s")
        metrics: List[Tuple[str, Tuple[str, float]]] = [
            (series_name(sensor_name, field, side), (timestamp, value))
            for sensor_name, values in snapshot.sensors.items()
            for (field, side), value in values.items()
        ]

        payload = pickle.dumps(metrics, protocol=2)
        header = struct.pack("!L", len(payload))
        message = header + payload
        sk.send(message)

        await asyncio.sleep(sampling_period)

    return log


# must be of the form `xx:xx:xx:xx:xx:xx`, where `x` is a hex digit (uppercase)
# FUTURE WORK: Won't work on MacOS. It uses UUIDs to abstract/hide the BT address.
def _bt_address_validate(addr: str):
    octets = addr.split(":")
    if len(octets) != 6:
        return False
    if not all(len(octet) == 2 for octet in octets):
        return False
    if not all(x in "0123456789ABCEDF" for octet in octets for x in octet):
        return False

    return True


async def discover_bluetooth_devices(
    filter: Callable[[BLEDevice], Coroutine[Any, Any, bool]],
    address: Optional[str] = None,
    timeout: float = BT_SCAN_GATHER_ALL_TIMEOUT,
) -> List[BLEDevice]:
    if address is not None:
        device = await BleakScanner.find_device_by_address(address, timeout=timeout)
        return [device] if device is not None else []

    return [x for x in await BleakScanner.discover(timeout=timeout) if await filter(x)]


async def discover_device_address(
    display_name: str,
    filter: Callable[[BLEDevice], Coroutine[Any, Any, bool]],
    bt_address: Optional[str],
):
    print(f"discovering {display_name}...")
    xs = await discover_bluetooth_devices(filter, bt_address)
    if not xs:
        logging.warning(f"no {display_name} found")
        return None

    if 1 < len(xs):
        logging.error(
            f"multiple {display_name} found. use `--bt-address` to disambiguate"
        )
        logging.error(f"{display_name} found:")
        xs.sort(key=lambda x: x.address)
        for x in xs:
            logging.error(x)
        return None

    return xs[0].address


def _is_lost_connection_exception(e: Exception, is_connecting: bool) -> bool:
    if isinstance(e, bleak.exc.BleakDBusError):
        # potentially caused by noisy environments or poor timing
        if e.dbus_error == "org.bluez.Error.NotConnected":
            return True

        ANY_TIME_DBUS_ERRORS = {
            ("org.bluez.Error.Failed", "br-connection-canceled"),
            ("org.bluez.Error.Failed", "Software caused connection abort"),
            ("org.bluez.Error.Failed", "Operation already in progress"),
        }

        CONNECTING_DBUS_ERRORS = {
            ("org.bluez.Error.Failed", "Operation already in progress"),
        }

        if (e.dbus_error, e.dbus_error_details) in ANY_TIME_DBUS_ERRORS:
            return True

        if (
            is_connecting
            and (e.dbus_error, e.dbus_error_details) in CONNECTING_DBUS_ERRORS
        ):
            return True

    if isinstance(e, bleak.exc.BleakError):
        msg = str(e).lower()
        if "not connected" in msg:
            return True
        if "device disconnected" in msg:
            return True

    return False


# TODO: This function is much more generalised than it needs to be here.
#       Reason: It is also used by `klipper.py`, but I haven't had the time or
#       energy to extract them to a shared library.
@overload
async def retry_if_disconnected(
    device: Union[
        Callable[[], Coroutine[Any, Any, Union[BLEDevice, str]]], BLEDevice, str
    ],
    go: Callable[[BleakClient], Coroutine[Any, Any, _A]],
    *,
    connection_timeout: Optional[float] = 10,
    exc_filter: Callable[[Exception], bool] = lambda e: False,
    log: Union[logging.Logger, _LoggerAdapter] = logging.root,
    retry: None = None,
) -> _A:
    pass


@overload
async def retry_if_disconnected(
    device: Union[
        Callable[[], Coroutine[Any, Any, Union[BLEDevice, str]]], BLEDevice, str
    ],
    go: Callable[[BleakClient], Coroutine[Any, Any, _A]],
    *,
    connection_timeout: Optional[float] = 10,
    exc_filter: Callable[[Exception], bool] = lambda e: False,
    log: Union[logging.Logger, _LoggerAdapter] = logging.root,
    retry: Callable[[], bool] = lambda: True,
) -> Optional[_A]:
    pass


async def retry_if_disconnected(
    device: Union[
        Callable[[], Coroutine[Any, Any, Union[BLEDevice, str]]], BLEDevice, str
    ],
    go: Callable[[BleakClient], Coroutine[Any, Any, _A]],
    *,
    connection_timeout: Optional[float] = 10,
    exc_filter: Callable[[Exception], bool] = lambda e: False,
    log: Union[logging.Logger, _LoggerAdapter] = logging.root,
    retry: Optional[Callable[[], bool]] = None,
) -> Optional[_A]:
    assert connection_timeout is None or 0 < connection_timeout

    while retry is None or retry():
        is_connected = False
        try:
            addr = device if isinstance(device, (BLEDevice, str)) else await device()
            timeout = 10 if connection_timeout is None else connection_timeout
            async with BleakClient(addr, timeout=timeout) as client:
                is_connected = True
                return await go(client)
        except asyncio.TimeoutError:
            # `TimeoutError` after we've connected aren't a connection timeout
            # and must not be suppressed.
            if is_connected or connection_timeout is not None:
                raise
        except bleak.exc.BleakDeviceNotFoundError:
            # same thing as `TimeoutError`
            if is_connected or connection_timeout is not None:
                raise
        except Exception as e:
            if _is_lost_connection_exception(e, not is_connected):
                # don't be (too) noisy about it, it happens
                log.debug("connection lost.", exc_info=e)
                log.info("connection lost. attempting reconnection...")
            elif not exc_filter(e):
                raise


async def device_is_likely_a_nevermore(device: BLEDevice) -> bool:
    # should have a short name, be it bootloader or main controller
    if device.name is None:
        return False

    if device.name in NEVERMORE_CONTROLLER_NAMES:  # expected/easy case
        return True

    # Sometimes system caches the old short-name for an address.
    # In that case, we'll have to connect to test it.
    if not device.name.startswith(BOOTLOADER_NAME_PREFIX):
        return False

    async def go(x: BleakClient):
        return x.services.get_characteristic(UUID_CHAR_CONFIG_REBOOT) is not None

    try:
        return await retry_if_disconnected(device, go)
    except TimeoutError:
        return False  # unable to connect to it in a timely manner, ignore the device


@dataclass
class Sensors:
    # key names must match those used by moonraker/klipper status (e.g. `gas` instead of `voc`)
    temperature_intake: Optional[float] = None  # Celsius
    temperature_exhaust: Optional[float] = None  # Celsius
    temperature_mcu: Optional[
        float
    ] = None  # Celsius, `float` but Maybe Float simplifies handling
    humidity_intake: Optional[float] = None  # %
    humidity_exhaust: Optional[float] = None  # %
    pressure_intake: Optional[float] = None  # hPa
    pressure_exhaust: Optional[float] = None  # hPa
    gas_intake: Optional[int] = None  # [1, VOC_INDEX_MAX]
    gas_exhaust: Optional[int] = None  # [1, VOC_INDEX_MAX]
    gas_raw_intake: Optional[int] = None  # [0, VOC_RAW_MAX]
    gas_raw_exhaust: Optional[int] = None  # [0, VOC_RAW_MAX]

    def as_dict(self) -> Dict[str, float]:
        return {
            f.name: getattr(self, f.name)
            for f in dataclasses.fields(self)
            if getattr(self, f.name) is not None
        }


class BleAttrReaderNotEnoughData(Exception):
    pass


class BleAttrReader:
    def __init__(self, raw: bytes):
        self.remaining = raw

    def humidity(self):
        return self._unsigned(2, 1, -2, 0, not_known=0xFFFF)

    def percentage8(self):
        return self._unsigned(1, 1, 0, -1, not_known=0xFF)

    def pressure(self):
        return self._unsigned(4, 1, -1, 0, not_known=0xFFFFFFFF)

    def temperature(self):
        return self._signed(2, 1, -2, 0, not_known=0x8000)

    def tachometer(self):
        return int(self._unsigned(2, 1, 0, 0))

    def voc_index(self) -> Optional[int]:  # [1, VOC_INDEX_MAX]
        return self._as_int(self._unsigned(2, 1, 0, 0, not_known=0))

    def voc_raw(self) -> Optional[int]:  # [0, VOC_RAW_MAX]
        return self._as_int(self._unsigned(2, 1, 0, 0, not_known=0xFFFF))

    @overload
    def _signed(self, sz: int, M: int, d: int, e: int) -> float:
        ...

    @overload
    def _signed(
        self, sz: int, M: int, d: int, e: int, *, not_known: int
    ) -> Optional[float]:
        ...

    def _signed(
        self, sz: int, M: int, d: int, e: int, *, not_known: Optional[int] = None
    ) -> Optional[float]:
        return self._consume(True, sz, M, d, e, not_known)

    @overload
    def _unsigned(self, sz: int, M: int, d: int, e: int) -> float:
        ...

    @overload
    def _unsigned(
        self, sz: int, M: int, d: int, e: int, *, not_known: int
    ) -> Optional[float]:
        ...

    def _unsigned(
        self, sz: int, M: int, d: int, e: int, *, not_known: Optional[int] = None
    ) -> Optional[float]:
        return self._consume(False, sz, M, d, e, not_known)

    def _consume(
        self,
        signed: bool,
        sz: int,
        M: int,
        d: int,
        e: int,
        not_known: Optional[int],
    ) -> Optional[float]:
        if len(self.remaining) < sz:
            raise BleAttrReaderNotEnoughData("insufficient data remaining")

        head = self.remaining[0:sz]
        self.remaining = self.remaining[sz:]

        # the constant is given as a hex literal (i.e. unsigned)
        if not_known == int.from_bytes(head, "little", signed=False):
            return None

        return self._raw_to_repr(int.from_bytes(head, "little", signed=signed), M, d, e)

    def _raw_to_repr(self, x: float, M: int, d: int, e: int) -> float:
        return x * M * (10**d) * (2**e)

    def _as_int(self, x: Optional[float]):
        return None if x is None else int(x)


def parse_agg_env(reader: BleAttrReader) -> Sensors:
    sensors = Sensors(
        temperature_intake=reader.temperature(),
        temperature_exhaust=reader.temperature(),
        temperature_mcu=reader.temperature(),
        humidity_intake=reader.humidity(),
        humidity_exhaust=reader.humidity(),
        pressure_intake=reader.pressure(),
        pressure_exhaust=reader.pressure(),
        gas_intake=reader.voc_index(),
        gas_exhaust=reader.voc_index(),
        gas_raw_intake=reader.voc_raw(),
        gas_raw_exhaust=reader.voc_raw(),
    )
    if sensors.pressure_intake is not None:
        sensors.pressure_intake /= 1000  # in hPA
    if sensors.pressure_exhaust is not None:
        sensors.pressure_exhaust /= 1000  # in hPA

    return sensors


@dataclass
class FanState:
    # key names must match those used by moonraker/klipper status (e.g. `rpm` instead of `tacho`)
    speed: Optional[float] = None
    rpm: Optional[float] = None

    def as_dict(self) -> Dict[str, float]:
        return {
            f.name: getattr(self, f.name)
            for f in dataclasses.fields(self)
            if getattr(self, f.name) is not None
        }


def parse_agg_fan(reader: BleAttrReader) -> FanState:
    x = FanState(
        speed=reader.percentage8(),
        rpm=reader.tachometer(),
    )
    if x.speed is not None:
        x.speed /= 100  # remap [0, 1] to match moonraker's data
    return x


async def _main_bluetooth(log: GraphiteLogger, address: Optional[str]):
    address = await discover_device_address(
        "Nevermore controllers",
        device_is_likely_a_nevermore,
        address,
    )
    if address is None:
        print(f"failed to find a nevermore")
        return

    async def _log_sensors(client: BleakClient):
        service_env = client.services.get_service(UUID_SERVICE_ENVIRONMENTAL_SENSING)
        service_fan = client.services.get_service(UUID_SERVICE_FAN)
        assert service_env is not None
        assert service_fan is not None
        char_env = service_env.get_characteristic(UUID_CHAR_DATA_AGGREGATE)
        char_fan = service_fan.get_characteristic(UUID_CHAR_FAN_AGGREGATE)
        assert char_env is not None
        assert char_fan is not None

        async def snapshot() -> SystemSnapshot:
            def key(name: str) -> Tuple[str, Optional[Side]]:
                for side in Side:
                    suffix = f"_{side.name}"
                    if name.endswith(suffix):
                        name = name[: -len(suffix)]
                        return (name, side)

                return (name, None)

            env = parse_agg_env(BleAttrReader(await client.read_gatt_char(char_env)))
            fan = parse_agg_fan(BleAttrReader(await client.read_gatt_char(char_fan)))
            sensors = {key(k): v for k, v in env.as_dict().items()}
            sensors.update((key(k), v) for k, v in fan.as_dict().items())

            return SystemSnapshot(address.replace(":", "-"), {"nevermore": sensors})

        print("ready. logging...")
        while True:
            await log(await snapshot())

    # HACK: Only want to consider these as cause for reconnect once we're
    #       in the main query loop. Mistakes during setup should be fatal.
    #       Fine for now b/c we're not a typical user tool.
    def exc_filter(e: Exception) -> bool:
        if isinstance(e, bleak.exc.BleakError):
            logging.exception("bleak error, reconnecting...", exc_info=e)
            return True

        return False

    try:
        print(f"connecting to {address}")
        await retry_if_disconnected(
            address, _log_sensors, connection_timeout=None, exc_filter=exc_filter
        )
    except asyncio.exceptions.CancelledError:
        pass
    except KeyboardInterrupt:
        pass


async def _main_moonraker(
    log: GraphiteLogger, moonraker: Ip4Port, retry_delay: Optional[float] = None
):
    assert retry_delay is None or 0 <= retry_delay
    uri = f"ws://{moonraker.addr}:{moonraker.port}/websocket"

    async def go(ws: WSClient.WebSocketClientProtocol):
        async def cmd_raw(args: Dict[str, Any]) -> Dict[str, Any]:
            args = args.copy()
            args["jsonrpc"] = "2.0"
            args["id"] = str(uuid.uuid4())

            await ws.send(json.dumps(args))
            while True:
                response = json.loads(await ws.recv())
                if response.get("id") == args["id"]:
                    return response["result"]

        async def cmd(method: str, args: Optional[Dict[str, Any]] = None):
            cmd: Dict[str, Any] = {"method": method}
            if args is not None:
                cmd["params"] = args
            return await cmd_raw(cmd)

        async def snapshot() -> SystemSnapshot:
            all_objects: list[str] = (await cmd("printer.objects.list"))["objects"]
            # future support for if/when we allow multiple nevermore instances
            nevermore_objects = [
                x for x in all_objects if x == "nevermore" or x.startswith("nevermore ")
            ]

            sensor_names: list[str] = (
                await cmd(
                    "printer.objects.query",
                    {"objects": {"heaters": ["available_sensors"]}},
                )
            )["status"]["heaters"]["available_sensors"]

            sensor_values = (
                await cmd(
                    "printer.objects.query",
                    {
                        "objects": {
                            name: None
                            for name in set(sensor_names).union(nevermore_objects)
                        }
                    },
                )
            )["status"]

            sensors: Dict[str, Dict[Tuple[str, Optional[Side]], float]] = defaultdict(
                lambda: {}
            )

            fields: Dict[str, Any]
            for sensor_name, fields in sensor_values.items():
                if any(
                    pattern.match(sensor_name.lower())
                    for pattern in MOONRAKER_SENSOR_NAME_BLOCKED
                ):
                    continue

                for field, value in fields.items():
                    if not isinstance(value, (int, float)):
                        continue

                    field_parts = field.split("_")
                    if field_parts[-1] in {"min", "max"}:
                        continue  # skip statistic fields in `nevermore`

                    try:
                        side = Side(field_parts[0].lower())
                        field_parts.pop(0)
                        if not field_parts:
                            continue  # ignore, malformed
                    except ValueError:
                        side = None

                    name = "_".join(field_parts)
                    if name not in MOONRAKER_SENSOR_FIELDS_ALLOWED:
                        continue

                    sensors[sensor_name][(name, side)] = value

            return SystemSnapshot(f"{moonraker.addr}:{moonraker.port}", sensors)

        print("ready. logging...")
        while True:
            await log(await snapshot())

    while True:
        try:
            print(f"connecting to moonraker: {uri}")
            async with WSClient.connect(uri) as ws:
                await go(ws)
        except Exception as e:
            if retry_delay is not None and isinstance(
                e,
                (
                    OSError,
                    websockets.exceptions.ConnectionClosedError,
                    websockets.exceptions.ConnectionClosedOK,
                ),
            ):
                logging.exception("connection error, retrying...", exc_info=e)
                print(f"retrying connection in {retry_delay} seconds")
                await asyncio.sleep(retry_delay)
            else:
                raise


class CmdLnArgs(tap.TypedArgs):
    # HACK: cast to `float` b/c `type-args` is stupid and doesn't do subtyping checks
    sampling_period: float = tap.arg(
        help="seconds between samples",
        # a bit less than the resolution to ensure we fill every slot
        default=float(GRAPHITE_DEFAULT_RETENTION_RESOLUTION * 0.75),
    )
    bt_address: Optional[str] = tap.arg(help="device's BT adddress")
    moonraker: Optional[Ip4Port] = tap.arg(
        help=f"ip4:port, mutex w/ `bt-address`, \"\" for \"localhost:{MOONRAKER_DEFAULT_PORT}\"",
        type=Ip4Port.parse("localhost", MOONRAKER_DEFAULT_PORT),
    )
    graphite: Ip4Port = tap.arg(
        help=f"ip4:port for graphite instance",
        default=Ip4Port("localhost", GRAPHITE_DEFAULT_PICKLE_PORT),
        type=Ip4Port.parse("localhost", GRAPHITE_DEFAULT_PICKLE_PORT),
    )
    retry_delay: Optional[float] = tap.arg(
        help="wait n seconds before re-connecting (moonraker only for now)"
    )


async def _main(args: CmdLnArgs):
    if args.bt_address is not None and not _bt_address_validate(args.bt_address):
        logging.error("invalid address for `--bt-address`")
        exit(1)

    if args.bt_address is not None and args.moonraker is not None:
        logging.error("can't specify both `--moonraker` and `--bt-address`")
        exit(1)

    log_entry = graphite_connection(args.graphite, args.sampling_period)

    if args.moonraker is None:
        await _main_bluetooth(log_entry, args.bt_address)
    else:
        await _main_moonraker(log_entry, args.moonraker, args.retry_delay)


def main():
    def go(args: CmdLnArgs):
        asyncio.run(_main(args))

    tap.Parser(CmdLnArgs).bind(go).run()


if __name__ == "__main__":
    main()
