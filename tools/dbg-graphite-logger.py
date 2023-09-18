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

__doc__ = """Script for logging sensors to local Graphite instance"""

import asyncio
import dataclasses
import datetime
import logging
import pickle
import socket
import struct
import typing
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
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

if typing.TYPE_CHECKING:
    _LoggerAdapter = logging.LoggerAdapter[logging.Logger]
else:
    _LoggerAdapter = logging.LoggerAdapter

SAMPLING_HZ = 1 / 30

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
    temperature_intake: Optional[float] = None  # Celsius
    temperature_exhaust: Optional[float] = None  # Celsius
    temperature_mcu: Optional[
        float
    ] = None  # Celsius, `float` but Maybe Float simplifies handling
    humidity_intake: Optional[float] = None  # %
    humidity_exhaust: Optional[float] = None  # %
    pressure_intake: Optional[float] = None  # kPa
    pressure_exhaust: Optional[float] = None  # kPa
    voc_index_intake: Optional[int] = None  # [1, VOC_INDEX_MAX]
    voc_index_exhaust: Optional[int] = None  # [1, VOC_INDEX_MAX]
    voc_raw_intake: Optional[int] = None  # [0, VOC_RAW_MAX]
    voc_raw_exhaust: Optional[int] = None  # [0, VOC_RAW_MAX]

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
        voc_index_intake=reader.voc_index(),
        voc_index_exhaust=reader.voc_index(),
        voc_raw_intake=reader.voc_raw(),
        voc_raw_exhaust=reader.voc_raw(),
    )
    if sensors.pressure_intake is not None:
        sensors.pressure_intake /= 1000  # in kPA
    if sensors.pressure_exhaust is not None:
        sensors.pressure_exhaust /= 1000  # in kPA

    return sensors


@dataclass
class FanState:
    power: Optional[float] = None
    tacho: Optional[float] = None

    def as_dict(self) -> Dict[str, float]:
        return {
            f.name: getattr(self, f.name)
            for f in dataclasses.fields(self)
            if getattr(self, f.name) is not None
        }


def parse_agg_fan(reader: BleAttrReader) -> FanState:
    return FanState(
        power=reader.percentage8(),
        tacho=reader.tachometer(),
    )


async def _log_sensors(client: BleakClient):
    service_env = client.services.get_service(UUID_SERVICE_ENVIRONMENTAL_SENSING)
    assert service_env is not None
    service_fan = client.services.get_service(UUID_SERVICE_FAN)
    assert service_fan is not None
    char_env = service_env.get_characteristic(UUID_CHAR_DATA_AGGREGATE)
    assert char_env is not None
    char_fan = service_fan.get_characteristic(UUID_CHAR_FAN_AGGREGATE)
    assert char_fan is not None

    print("connecting to local graphite...")
    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sk.connect(("localhost", 2004))
    print("connected")

    while True:
        env = parse_agg_env(BleAttrReader(await client.read_gatt_char(char_env)))
        fan = parse_agg_fan(BleAttrReader(await client.read_gatt_char(char_fan)))

        timestamp = datetime.datetime.now().strftime("%s")
        metrics_env: List[Tuple[str, Tuple[str, float]]] = [
            (f"{client.address}.{k}", (timestamp, v)) for k, v in env.as_dict().items()
        ]
        metrics_fan: List[Tuple[str, Tuple[str, float]]] = [
            (f"{client.address}.{k}", (timestamp, v)) for k, v in fan.as_dict().items()
        ]

        payload = pickle.dumps(metrics_env + metrics_fan, protocol=2)
        header = struct.pack("!L", len(payload))
        message = header + payload
        sk.send(message)

        await asyncio.sleep(1 / SAMPLING_HZ)


class CmdLnArgs(tap.TypedArgs):
    bt_address: Optional[str] = tap.arg(help="device's BT adddress")


async def _main(args: CmdLnArgs):
    if args.bt_address is not None and not _bt_address_validate(args.bt_address):
        logging.error("invalid address for `--bt-address`")
        return

    address_found = await discover_device_address(
        "Nevermore controllers",
        device_is_likely_a_nevermore,
        args.bt_address,
    )
    if address_found is None:
        print(f"failed to find a nevermore")
        return

    print(f"connecting to {address_found}")

    # HACK: Only want to consider these as cause for reconnect once we're
    #       in the main query loop. Mistakes during setup should be fatal.
    #       Fine for now b/c we're not a typical user tool.
    def exc_filter(e: Exception) -> bool:
        if isinstance(e, bleak.exc.BleakError):
            logging.exception("bleak error, reconnecting...", exc_info=e)
            return True

        return False

    try:
        await retry_if_disconnected(
            address_found, _log_sensors, connection_timeout=None, exc_filter=exc_filter
        )
    except asyncio.exceptions.CancelledError:
        pass
    except KeyboardInterrupt:
        pass


def main():
    def go(args: CmdLnArgs):
        asyncio.run(_main(args))

    tap.Parser(CmdLnArgs).bind(go).run()


if __name__ == "__main__":
    main()
