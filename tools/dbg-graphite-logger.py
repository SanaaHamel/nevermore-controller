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

sudo apt-get install docker.io apparmor cgroupfs-mount needrestart criu
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
      "seriesByTag(\"name=gas_raw\")",
      "alpha(seriesByTag(\"name=gas_raw_uncompensated\"), 0.25)",
      "alpha(dashed(seriesByTag(\"name=gas_raw_mean\")), 0.75)",
      "alpha(sumSeries(seriesByTag(\"name=gas_raw_mean\", \"side=intake\"), seriesByTag(\"name=gas_raw_var\", \"side=intake\")), 0.5)",
      "alpha(sumSeries(seriesByTag(\"name=gas_raw_mean\", \"side=intake\"), scale(seriesByTag(\"name=gas_raw_var\", \"side=intake\"), -1)), 0.5)",
      "alpha(sumSeries(seriesByTag(\"name=gas_raw_mean\", \"side=exhaust\"), seriesByTag(\"name=gas_raw_var\", \"side=exhaust\")), 0.5)",
      "alpha(sumSeries(seriesByTag(\"name=gas_raw_mean\", \"side=exhaust\"), scale(seriesByTag(\"name=gas_raw_var\", \"side=exhaust\"), -1)), 0.5)"
    ],
    "vtitle": "Resistance",
    "title": "VOC Raw"
  },
  {
    "target": [
      "seriesByTag(\"name=temperature\", \"side=~(intake|exhaust)\")",
      "seriesByTag(\"name=temperature\", \"side!=~(intake|exhaust)\", \"sensor!=~temperature_sensor\")"
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
  },
  {
    "target": [
      "seriesByTag(\"name=speed\", \"sensor=nevermore\")",
      "secondYAxis(seriesByTag(\"name=rpm\", \"sensor=nevermore\"))"
    ],
    "title": "Fan",
    "vtitle": "PWM %",
    "vtitleRight": "RPM"
  },
  {
    "target": [
      "seriesByTag(\"name=~gas_raw_(gamma|gating)_(mean|var)\")"
    ],
    "vtitle": "%",
    "title": "Gating",
    "yMin": "0"
  }
]
```

"""

import asyncio
import datetime
import enum
import json
import logging
import os
import pickle
import re
import shlex
import socket
import struct
import sys
import tempfile
import uuid
from collections import defaultdict
from dataclasses import dataclass
from typing import Any, Callable, Coroutine, Dict, List, Optional, Tuple, Union

import bleak
import typed_argparse as tap
import websockets.exceptions
import websockets.legacy.client as WSClient
from bleak import BleakClient
from nevermore_tool_utilities import NevermoreToolCmdLnArgs, device_address_discover
from nevermore_utilities import *
from typing_extensions import override

MOONRAKER_DEFAULT_PORT = 7125
GRAPHITE_DEFAULT_PICKLE_PORT = 2004
GRAPHITE_DEFAULT_RETENTION_RESOLUTION = 10

SYSTEMD_SERVICE_FILENAME = "nevermore-graphite-logger.service"
SYSTEMD_SERVICE_PATH = f"/etc/systemd/system/{SYSTEMD_SERVICE_FILENAME}"

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
MOONRAKER_SENSOR_FIELDS_ALLOWED_REGEX = [
    re.compile("gas_raw_.*"),
]

MOONRAKER_SENSOR_NAME_BLOCKED = [
    re.compile("temperature_sensor nevermore_.*_voc"),  # ignore common VOC plotters
]


def systemd_service_definition(args: List[str]) -> str:
    script_file = sys.argv[0]
    work_dir = os.path.abspath(".")
    return f"""
[Unit]
Description=Nevermore Graphite Logger
# we might be using moonraker, so wait for network to be up
Requires=network-online.target
After=network-online.target

[Install]
WantedBy=multi-user.target

[Service]
Type=simple
User={os.getlogin()}
RemainAfterExit=no
WorkingDirectory={work_dir}
ExecStart={script_file} {shlex.join(args)}
Restart=on-failure
RestartSec=1m
"""


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

        timestamp = (
            snapshot.timestamp.strftime("%s")
            if snapshot.timestamp is not None
            else "-1"
        )
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


def _extract_fields(fields: Dict[str, Any]):
    parts: Dict[Tuple[str, Optional[Side]], float] = {}
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
        if name not in MOONRAKER_SENSOR_FIELDS_ALLOWED and not any(
            r.fullmatch(name) for r in MOONRAKER_SENSOR_FIELDS_ALLOWED_REGEX
        ):
            continue

        parts[(name, side)] = value

    return parts


async def _main_bluetooth(log: GraphiteLogger, address: Optional[str]):
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
            intake, exhaust = SensorState.parse(
                BleAttrReader(await client.read_gatt_char(char_env))
            )
            fan = FanState.parse(BleAttrReader(await client.read_gatt_char(char_fan)))
            sensors = _extract_fields(
                ControllerState(intake, exhaust, fan.speed or 0, fan.rpm or 0).as_dict()
            )

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


@dataclass
class MoonrakerMalformedResponse(Exception):
    response: Union[str, bytes]

    @override
    def __str__(self) -> str:
        return f"malformed moonraker response: `{self.response:r}`"


async def _main_moonraker(log: GraphiteLogger, moonraker: Ip4Port):
    uri = f"ws://{moonraker.addr}:{moonraker.port}/websocket"

    async def go(ws: WSClient.WebSocketClientProtocol):
        async def cmd_raw(args: Dict[str, Any]) -> Dict[str, Any]:
            args = args.copy()
            args["jsonrpc"] = "2.0"
            args["id"] = str(uuid.uuid4())

            await ws.send(json.dumps(args))
            while True:
                response_raw = await ws.recv()

                try:
                    response = json.loads(response_raw)
                except json.JSONDecodeError:
                    raise MoonrakerMalformedResponse(response_raw)

                if not isinstance(response, dict):
                    raise MoonrakerMalformedResponse(response_raw)

                if response.get("id") != args["id"]:
                    continue

                if not isinstance(response.get("result"), dict):
                    raise MoonrakerMalformedResponse(response_raw)

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
                if not any(
                    pattern.match(sensor_name.lower())
                    for pattern in MOONRAKER_SENSOR_NAME_BLOCKED
                ):
                    sensors[sensor_name] = _extract_fields(fields)

            return SystemSnapshot(f"{moonraker.addr}:{moonraker.port}", sensors)

        print("ready. logging...")
        while True:
            await log(await snapshot())

    while True:
        print(f"connecting to moonraker: {uri}")
        async with WSClient.connect(uri) as ws:
            await go(ws)


class CmdLnArgs(NevermoreToolCmdLnArgs):
    # HACK: cast to `float` b/c `type-args` is stupid and doesn't do subtyping checks
    sampling_period: float = tap.arg(
        help="seconds between samples",
        # a bit less than the resolution to ensure we fill every slot
        default=float(GRAPHITE_DEFAULT_RETENTION_RESOLUTION * 0.75),
    )
    moonraker: Optional[Ip4Port] = tap.arg(
        help=f"ip4:port, mutex w/ `bt-address`, \"\" for \"localhost:{MOONRAKER_DEFAULT_PORT}\"",
        type=Ip4Port.parse("localhost", MOONRAKER_DEFAULT_PORT),
    )
    graphite: Ip4Port = tap.arg(
        help=f"ip4:port for graphite instance",
        default=Ip4Port("localhost", GRAPHITE_DEFAULT_PICKLE_PORT),
        type=Ip4Port.parse("localhost", GRAPHITE_DEFAULT_PICKLE_PORT),
    )
    install_systemd_service: bool = tap.arg(
        default=False, help="install a systemd startup service using current arguments"
    )

    @override
    def validate(self):
        ok = super().validate()

        if self.bt_address is not None and self.moonraker is not None:
            logging.error("can't specify both `--moonraker` and `--bt-address`")
            ok = False

        return ok


async def _main(args: CmdLnArgs):
    if not args.validate():
        exit(1)

    if args.install_systemd_service:
        # FIXME: we should be unparsing a copy of `args` w/ `install_systemd_service` disabled
        #        instead of abusing `sys.argv`.
        raw_args = sys.argv[1:]
        raw_args.remove("--install-systemd-service")
        service_def = systemd_service_definition(raw_args)

        def sudo_cmd(cmd: str):
            cmd = f"sudo {cmd}"
            print(cmd)
            os.system(cmd)

        with tempfile.NamedTemporaryFile() as f:
            f.write(service_def.encode("utf-8"))
            f.flush()
            sudo_cmd(f"cp {shlex.join([f.name, SYSTEMD_SERVICE_PATH])}")
            sudo_cmd(f"systemctl enable {SYSTEMD_SERVICE_FILENAME}")
            sudo_cmd(f"systemctl start {SYSTEMD_SERVICE_FILENAME}")

        exit(0)

    log_entry = graphite_connection(args.graphite, args.sampling_period)

    if args.moonraker is None:
        await _main_bluetooth(log_entry, await args.bt_address_discover())
    else:
        await _main_moonraker(log_entry, args.moonraker)


def main():
    def go(args: CmdLnArgs):
        asyncio.run(_main(args))

    tap.Parser(CmdLnArgs).bind(go).run()


if __name__ == "__main__":
    main()
