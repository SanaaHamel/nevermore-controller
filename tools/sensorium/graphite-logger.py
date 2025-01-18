#!/bin/bash
"true" '''\'
set -eu
set -o pipefail

FILE="$(readlink -f "$0")"
ROOT_DIR="$(dirname "$FILE")/.."

"$ROOT_DIR/setup-tool-env.bash"
"$ROOT_DIR/.venv/bin/python" "$FILE" "$@"

exit 0 # required to stop shell execution here
'''

# Script for logging sensorium data to a local Graphite instance
#
# Copyright (C) 2025       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

__doc__ = """Script for logging sensorium data to a local Graphite instance.

I suggest the following dashboard snippet (paste it via `Dashboard -> Edit Dashboard`):

FIXME UPDATE THE DASHBOARD DEFAULT

```
[
  {
    "target": [
      "seriesByTag(\"name=sensorium-voc-raw\")"
    ],
    "vtitle": "Resistance",
    "title": "VOC Raw"
  },
  {
    "target": [
      "seriesByTag(\"name=sensorium-temp\")"
    ],
    "vtitle": "C",
    "title": "Temperature"
  },
  {
    "target": [
      "seriesByTag(\"name=sensorium-rh\")"
    ],
    "vtitle": "Relative Humidity %",
    "title": "Humidity"
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
from typing import (
    Any,
    Callable,
    Coroutine,
    Dict,
    Iterable,
    List,
    Optional,
    Tuple,
    Union,
)

import serial
import typed_argparse as tap
import websockets.exceptions
import websockets.legacy.client as WSClient
from typing_extensions import override


GRAPHITE_DEFAULT_PICKLE_PORT = 2004
GRAPHITE_DEFAULT_RETENTION_RESOLUTION = 10


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


@dataclass
class SensorMeasurement:
    pin_clk: int
    pin_dat: int
    address: int
    name: str
    type: str
    value: float


GraphiteLogger = Callable[[Iterable[SensorMeasurement]], Coroutine[Any, Any, None]]


def graphite_connection(dst: Ip4Port, sampling_period: float) -> GraphiteLogger:
    assert 0 < sampling_period

    if sampling_period < GRAPHITE_DEFAULT_RETENTION_RESOLUTION / 2:
        print(
            f"WARN - `--sampling-period` of {sampling_period} sec is less than half the default graphite retention period ({GRAPHITE_DEFAULT_RETENTION_RESOLUTION} sec)."
        )
        print(
            "If you need a more fine-grain retention period then you'll need to modify the graphite installation's `/opt/graphite/conf/storage-schemas.conf`"
        )

    print(f"connecting to graphite: tcp://{dst.addr}:{dst.port}")
    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sk.connect((dst.addr, dst.port))

    def series_name(x: SensorMeasurement) -> str:
        # FIXME: escape `;` and any `~` prefix?
        return f"sensorium-{x.type};sensor={x.name};address={x.address};pin_clk={x.pin_clk};pin_dat={x.pin_dat}"

    async def log(snapshots: Iterable[SensorMeasurement]) -> None:
        TIMESTAMP = "-1"

        metrics: List[Tuple[str, Tuple[str, float]]] = [
            (series_name(x), (TIMESTAMP, x.value)) for x in snapshots
        ]

        payload = pickle.dumps(metrics, protocol=2)
        header = struct.pack("!L", len(payload))
        message = header + payload
        sk.send(message)

        await asyncio.sleep(sampling_period)

    return log


class CmdLnArgs(tap.TypedArgs):
    serial: str = tap.arg(help="sensorium serial port")
    baud_rate: int = tap.arg(help="baud rate (if applicable)", default=115200)
    # HACK: cast to `float` b/c `type-args` is stupid and doesn't do subtyping checks
    sampling_period: float = tap.arg(
        help="seconds between samples",
        # a bit less than the resolution to ensure we fill every slot
        default=float(GRAPHITE_DEFAULT_RETENTION_RESOLUTION * 0.75),
    )
    graphite: Ip4Port = tap.arg(
        help=f"ip4:port for graphite instance",
        default=Ip4Port("localhost", GRAPHITE_DEFAULT_PICKLE_PORT),
        type=Ip4Port.parse("localhost", GRAPHITE_DEFAULT_PICKLE_PORT),
    )
    echo: bool = tap.arg(help="echo serial to stdout")

    def validate(self) -> bool:
        ok = True
        # FUTURE WORK: additional validation
        return ok


RE_SENSORIUM_MEASURE = re.compile(
    r"^SENSORIUM MEASURE\s+clk=(\d+)\s+dat=(\d+)\s+addr=0x([0-9A-Fa-f]+)\s+name=([^\s]+)\s+type=([^\s]+)\s+value=([-+]?(infinity|NaN|(\d+(.\d+)?)))$"
)


def parse(line: str) -> Optional[SensorMeasurement]:
    if (r := RE_SENSORIUM_MEASURE.match(line)) is None:
        return None

    clk, dat, addr, name, typ, value = r.groups()[:6]
    return SensorMeasurement(
        pin_clk=int(clk),
        pin_dat=int(dat),
        address=int(addr),
        name=name,
        type=typ,
        value=float(value),
    )


async def _main(args: CmdLnArgs):
    if not args.validate():
        exit(1)

    log_entries = graphite_connection(args.graphite, args.sampling_period)

    print("connecting to device...")
    with serial.Serial(args.serial, baudrate=args.baud_rate, timeout=0.1) as s:
        while True:
            entries: List[SensorMeasurement] = []
            while (line := s.readline()) != b"":
                line = str(line, 'UTF-8')
                if args.echo:
                    sys.stdout.write(line)

                if (entry := parse(line.strip())) is not None:
                    entries.append(entry)

            await log_entries(entries)


def main():
    def go(args: CmdLnArgs):
        asyncio.run(_main(args))

    tap.Parser(CmdLnArgs).bind(go).run()


if __name__ == "__main__":
    main()
