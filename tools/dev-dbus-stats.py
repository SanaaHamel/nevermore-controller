#!/bin/bash
"true" '''\'
set -eu
set -o pipefail

FILE="$(readlink -f "$0")"
ROOT_DIR="$(dirname "$FILE")"

if [ "$EUID" -eq 0 ]; then
  echo "NB: Running as root, skipping tool setup check."
else
  "$ROOT_DIR/setup-tool-env.bash"
fi

"$ROOT_DIR/.venv/bin/python" "$FILE" "$@"

exit 0 # required to stop shell execution here
'''

# Basically ripped off some other script from the internet.
# Can't remember where, but claim no credit.

import asyncio
from dataclasses import dataclass
from typing import Dict, List, Optional, Set

from dbus_fast import BusType
import typed_argparse as tap
from dbus_fast.aio.message_bus import MessageBus
from dbus_fast.signature import Variant


def get_cmdline(pid: int):
    if pid <= 0:
        return "<bad PID>"

    try:
        procpath = '/proc/' + str(pid) + '/cmdline'
        with open(procpath, 'r') as f:
            return " ".join(f.readline().split('\0'))
    except:
        return "<err>"


class Args(tap.TypedArgs):
    show_all: bool = tap.arg(
        help="show all connections instead of hiding those with flushed buffers"
    )
    system: bool = tap.arg(
        help="connect to system bus instead of session bus", default=False
    )


@dataclass
class ProcRecord:
    wkn: List[str]
    pid: int
    cmd: str
    stats: Optional[Dict[str, Variant]]


# Parsing parameters
async def _main(args: Args):
    bus = MessageBus(bus_type=BusType.SYSTEM if args.system else BusType.SESSION)
    await bus.connect()

    introspect = await bus.introspect("org.freedesktop.DBus", "/org/freedesktop/DBus")
    remote_object = bus.get_proxy_object(
        "org.freedesktop.DBus", "/org/freedesktop/DBus", introspect
    )
    bus_iface = remote_object.get_interface("org.freedesktop.DBus")
    stats_iface = remote_object.get_interface("org.freedesktop.DBus.Debug.Stats")

    async def get_stats(conn: str) -> Optional[Dict[str, Variant]]:
        try:
            return await stats_iface.call_get_connection_stats(conn)
        except:
            # failed: did you enable the Stats interface? (compilation option: --enable-stats)
            # https://bugs.freedesktop.org/show_bug.cgi?id=80759 would be nice too
            return None

    names: Set[str] = set(await bus_iface.call_list_names())
    unique_names = {a for a in names if a.startswith(":")}
    well_known_names = names - unique_names
    owners = {
        name: await bus_iface.call_get_name_owner(name) for name in well_known_names
    }

    async def fetch_info(conn: str):
        pid: int = await bus_iface.call_get_connection_unix_process_id(conn)
        return ProcRecord(
            wkn=sorted(k for k, v in owners.items() if v == conn),
            pid=pid,
            cmd=get_cmdline(pid),
            stats=await get_stats(conn),
        )

    def is_boring(x: ProcRecord):
        if "klipper" in x.cmd:
            return False

        if x.stats is None:
            return True

        return (
            x.stats['IncomingBytes'].value == 0 and x.stats['OutgoingBytes'].value == 0
        )

    stats = [(name, await fetch_info(name)) for name in unique_names]
    stats.sort(key=lambda kv: (kv[1].pid, kv[0]))

    for k, v in stats:
        # hide boring connections
        if not args.show_all and is_boring(v):
            continue

        print(f"{k} with pid {v.pid} '{v.cmd}':")
        print(f"              Names: {' '.join(v.wkn)}")

        if v.stats is not None:
            print(f"      IncomingBytes: {v.stats['IncomingBytes'].value:>10}")
            print(f"  PeakIncomingBytes: {v.stats['PeakIncomingBytes'].value:>10}")
            print(f"      OutgoingBytes: {v.stats['OutgoingBytes'].value:>10}")
            print(f"  PeakOutgoingBytes: {v.stats['PeakOutgoingBytes'].value:>10}")
        else:
            print("  No stats available.")

        print("")


def main():
    def go(args: Args):
        asyncio.run(_main(args))

    tap.Parser(Args).bind(go).run()


if __name__ == "__main__":
    main()
