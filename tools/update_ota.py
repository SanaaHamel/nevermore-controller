#!/bin/bash
"true" '''\'
set -eu
set -o pipefail

FILE="$(readlink -f "$0")"
ROOT_DIR="$(dirname "$FILE")"

IGNORE_KLIPPER=0
for ARG; do
  shift
  if [ "$ARG" = "--ignore-klipper" ]; then
    IGNORE_KLIPPER=1
  fi
  set -- "$@" "$ARG"
done

# HACK: A GATT client can spuriously connect to a BlueTooth device that only
#       exposes a BlueTooth Classic interface and then interrupt/close an
#       active SPP stream.
#       I'm not sure of the how or why, but for now that means we have to make
#       sure Klipper isn't trying to connect to the controller while updating.
KLIPPER_SUSPENDED=0
ensure_klipper_not_running() {
  if [ "$(systemctl list-units --full -all -t service --no-legend --no-pager --state active | grep -F "klipper.service")" ]; then
    echo "Klipper service is active."
    echo "Due to software quirks you should stop klipper during the firmware update."
    echo "(Make sure you don't have a print in progress!)"

    while true; do
        read -r -p "Stop Klipper during update? [Yn]" ANSWER
        case $ANSWER in
        ""|[Yy])
        echo "Stopping klipper... "
        sudo systemctl stop klipper.service
        KLIPPER_SUSPENDED=1
        break
        ;;
        [Nn])
        echo "Klipper is active, cannot safely update controller. Exiting."
        exit 1
        ;;
        *) echo "Stop Klipper during update" ;;
        esac
    done
  fi
}

finish() {
  if [[ "$KLIPPER_SUSPENDED" = 1 ]]; then
    echo "Restarting klipper... "
    sudo systemctl start klipper.service
  fi
}
trap finish EXIT

"$ROOT_DIR/setup-tool-env.bash"

if [[ "$IGNORE_KLIPPER" = 0 ]]; then
  ensure_klipper_not_running
fi

"$ROOT_DIR/.venv/bin/python" "$FILE" "$@"
exit 0
'''

# Script for updating a Nevermore controller.
#
# Copyright (C) 2023       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

__doc__ = """Script for updating Nevermore controllers."""

import asyncio
import json
import logging
import os
import subprocess
import typing
from abc import abstractmethod
from dataclasses import dataclass
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Any, Iterable, List, Optional, Set, TypeVar

import serial_flash
import typed_argparse as tap
from aiohttp import ClientSession
from bleak import BleakClient
from nevermore_tool_utilities import NevermoreToolCmdLnArgs
from nevermore_utilities import *
from serial_flash.transport.bluetooth.spp import SppArgs
from serial_flash.transport.serial import SerialArgs
from typing_extensions import override

if typing.TYPE_CHECKING:
    _LoggerAdapter = logging.LoggerAdapter[logging.Logger]
else:
    _LoggerAdapter = logging.LoggerAdapter

__all__ = ['CmdLnArgs', 'main']

REBOOT_DELAY = 3  # seconds

URL_RELEASES_FETCH_LATEST = (
    "https://api.github.com/repos/sanaahamel/nevermore-controller/releases?per_page=1"
)

URL_RELEASES_FETCH_TAG = (
    "https://api.github.com/repos/sanaahamel/nevermore-controller/releases/tags/{0}"
)

FETCH_RELEASE_HEADERS = {
    "Accept": "application/vnd.github+json",
    "X-GitHub-Api-Version": "2022-11-28",
}

DOWNLOAD_ASSET_HEADERS = {
    "Accept": "application/octet-stream",
    "X-GitHub-Api-Version": "2022-11-28",
}

ADDRESS_2_BOARD_CACHE_FILENAME = ".update_ota.cache.address-2-board.json"
ADDRESS_2_BOARD_CACHE_PATH = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), ADDRESS_2_BOARD_CACHE_FILENAME
)

_A = TypeVar("_A")

P = CharacteristicProperty


def join_multi_line_list_tabbed(xs: Iterable[str]) -> Optional[str]:
    return ("\n\t" + "\n\t".join(xs)) if xs else None


def _bssid_from_bt_address(bt_address: str):
    # The BSSID of a Pico is by default 1 less than that of its BT address.
    assert bt_address_validate(bt_address)
    bssid = bt_address.split(':')
    bssid[-1] = f"{(int(bssid[-1], 16) - 1) & 0xFF:02x}"
    return ":".join(bssid)


# in order of preference
def update_filename_candidates(board: str) -> List[str]:
    xs = [f"picowota_ota-nevermore-controller_{board}.uf2"]
    if board == "pico_w":  # fallback to old build names
        xs.append("picowota_nevermore-controller-ota.uf2")

    return xs


@dataclass
class ReleaseInfo:
    tag: str
    info: str
    assets: List[str]


async def fetch_release(url: str) -> Optional[ReleaseInfo]:
    async with ClientSession() as session:
        async with session.get(url, headers=FETCH_RELEASE_HEADERS) as response:
            content = await response.read()
            if response.status != 200:
                logging.error(f"fetch releases failed. http code={response.status}")
                return None

            content = json.loads(content)
            if isinstance(content, list):
                content = content[0]

            return ReleaseInfo(
                tag=content["tag_name"],
                info=content.get("body", "<<NO RELEASE DESCRIPTION>>"),
                assets=[x["browser_download_url"] for x in content["assets"]],
            )


async def fetch_release_by_tag(tag: str) -> Optional[ReleaseInfo]:
    return await fetch_release(URL_RELEASES_FETCH_TAG.format(tag))


async def fetch_release_latest() -> Optional[ReleaseInfo]:
    return await fetch_release(URL_RELEASES_FETCH_LATEST)


async def fetch_asset(asset_url: str, file_suffix: str):
    async with ClientSession() as session:
        async with session.get(asset_url, headers=DOWNLOAD_ASSET_HEADERS) as response:
            content = await response.read()
            if response.status != 200:
                logging.error(f"download asset failed. http code={response.status}")
                return None

            file = NamedTemporaryFile(suffix=file_suffix)
            file.write(content)
            return file


async def download_filename(release: ReleaseInfo, filename: str):
    assert_candidates = [x for x in release.assets if x.endswith(filename)]
    if 1 < len(assert_candidates):
        logging.error(
            f"multiple candidates for `{filename}`:{join_multi_line_list_tabbed(sorted(assert_candidates))}"
        )
        return None

    if not assert_candidates:
        return None

    print(f"downloading {release.tag} [{assert_candidates[0]}]")
    return await fetch_asset(assert_candidates[0], file_suffix=filename)


# keep downloaded files alive until program terminates
g_download_board_files: Set[Any] = set()


async def download_board_update(release: ReleaseInfo, board: str):
    global g_download_board_files
    candidates = update_filename_candidates(board)
    for filename in candidates:
        file = await download_filename(release, filename)
        if file is not None:
            g_download_board_files.add(file)
            return Path(file.name)

    logging.error(f"release {release.tag} has no updates for board `{board}`.")
    logging.error(f"candidates:{join_multi_line_list_tabbed(candidates) or ' <NONE>'}")
    logging.error(
        f"release assets:{join_multi_line_list_tabbed(sorted(release.assets)) or ' <NONE>'}"
    )
    return None


async def hardware_board(transport: Transport):
    try:
        return str(await transport(UUID_CHAR_HARDWARE_REVISION).read(), "UTF-8")
    except Transport.AttrNotFound:
        logging.warning(
            "device does not have a hardware revision characteristic, assuming board is `pico_w`"
        )
        return "pico_w"


async def software_revision(transport: Transport):
    HEADER = "commit: "
    DIRTY = "-dirty"

    revision = str(await transport(UUID_CHAR_SOFTWARE_REVISION).read(), "UTF-8")
    commit = revision
    if commit.startswith(HEADER):
        commit = commit[len(HEADER) :]
    dirty = commit.endswith(DIRTY)
    if dirty:
        commit = commit[: -len(DIRTY)]

    try:
        described = subprocess.check_output(
            ["git", "describe", "--long", "--tags", commit]
        ).strip()
        return str(described, "UTF-8") + (DIRTY if dirty else "")
    except subprocess.CalledProcessError:
        logging.exception("failed to describe revision")
        return revision


async def reset_setting_defaults(transport: Transport):
    await transport(UUID_CHAR_CONFIG_RESET, {P.WRITE}).write(
        (1 << 1).to_bytes(1, 'little')
    )


class PinState(enum.Enum):
    UNKNOWN = enum.auto()
    DEFAULT = enum.auto()
    CUSTOMISED = enum.auto()


async def pin_config(transport: Transport) -> PinState:
    try:
        current = await transport(UUID_CHAR_CONFIG_PINS).read()
        default = await transport(UUID_CHAR_CONFIG_PINS_DEFAULT).read()
    except Transport.AttrNotFound:
        logging.warning(
            "device does not have pin configuration characteristics; ignoring pin config checks"
        )
        return PinState.UNKNOWN

    return PinState.DEFAULT if current == default else PinState.CUSTOMISED


async def reboot_device(transport: Transport, *, bootloader: bool):
    try:
        char_reboot = transport(UUID_CHAR_CONFIG_REBOOT, {P.WRITE})
    except Transport.AttrNotFound:
        logging.error(
            "Nevermore is too old to update with OTA. Manually flash the latest UF2 to it."
        )
        return None

    print(f"sending {'reboot-to-ota' if bootloader else 'reboot'} command...")
    # b"\1" to reboot to OTA, "0" to restart
    await char_reboot.write(b"\1" if bootloader else b"\0")


class CmdLnArgs(NevermoreToolCmdLnArgs):
    file: Optional[Path] = tap.arg(help="filepath for image")
    tag: Optional[str] = tap.arg(help="download specific release tag")
    board: Optional[str] = tap.arg(help="override/specify board (USE WITH CAUTION!)")
    # `--ignore-klipper` are used/handled by the shell wrapper script.
    # They're listed here for `--help` and unknown-argument checking.
    ignore_klipper: bool = tap.arg(help="don't check if klipper is running")
    unattended: bool = tap.arg(help="do not ask user for input", default=False)

    @override
    def validate(self):
        ok = super().validate()

        if self.file is not None and self.tag is not None:
            logging.error("`--tag` and `--file` are mutually exclusive")
            ok = False

        return ok


async def _update_via_serial(args: CmdLnArgs):
    assert args.file is not None
    assert args.serial is not None

    serial_flash.run(
        SerialArgs(
            filename=str(args.file.absolute()),
            port=args.serial,
            baud_rate=args.baud_rate,
        )
    )


async def _update_via_bt_spp(args: CmdLnArgs):
    assert args.file is not None
    assert args.bt_address is not None

    while True:
        try:
            serial_flash.run(
                SppArgs(
                    filename=str(args.file.absolute()),
                    addr=args.bt_address,
                    channel=1,
                )
            )
            break
        except TimeoutError as e:
            logging.warning(
                f"potentially transient comm error, retrying...", exc_info=e
            )


async def _post_update_actions_interactive(
    args: CmdLnArgs, prev_version: str, prev_pin_state: PinState
):
    if args.bt_address is None and args.serial is None:
        return

    print(f"connecting to controller for post-upgrade actions...")
    print("(this may take longer than usual)")
    print(
        "NOTE: Ignore logged exceptions about `A message handler raised an exception: 'org.bluez.Device1'` or `org.bluez.GattService1`."
    )
    print(
        "      This is caused by a bug in `bleak` but should be benign for this application."
    )

    async def attempt(transport: Transport) -> None:
        curr_version = await software_revision(transport)
        print(f"previous version: {prev_version}")
        print(f" current version: {curr_version}")

        # FUTURE WORK: query settings and only ask if they're different (ideally present a diff)
        if input_yes_no(
            args,
            True,
            "Default settings may have changed.\n"
            "Do you wish to use any new defaults? [Recommended]\n"
            "(Settings customised in the Klipper config will be restored when Klipper reconnects.)",
        ):
            print("applying new default settings...")
            await reset_setting_defaults(transport)

        if await pin_config(transport) == PinState.CUSTOMISED:
            if prev_pin_state == PinState.DEFAULT:
                print("Pin assignment defaults have changed.")
            else:
                print("Current pin assignments do not match the defaults.")

            print(
                "  To replace the current pin config with the new defaults, run:  ./pin-config.py --reset-default\n"
                "  To see the current pin config, run:  ./pin-config.py --echo-current\n"
                "\n"
                "You may safely ignore this warning if the controller is part of a completed filter\n"
                "and you do not intend to make hardware changes."
            )

    while True:
        if (transport := await args.connect(timeout=None)) is None:
            print("unable to connect to nevermore")
            return

        try:
            async with transport:
                await attempt(transport)
                return
        except Transport.AttrNotFound as e:
            # don't do this hack for non BT connections
            if args.bt_address is None:
                raise e

            # HACK: Treat missing attributes as a broken early connection
            #       This isn't true for every very old controller versions, but
            #       dear gods those are old.
            logging.warning(f"{e}\npossibly a broken early connection, reconnecting...")
        except Exception as e:
            if not is_lost_connection_exception(e):
                raise


def input_yes_no(args: CmdLnArgs, default: bool, msg: str) -> bool:
    if args.unattended:
        return default

    while True:
        response = input(f"{msg} ({'Y/n' if default else 'y/N'})")
        if response == "y":
            return True
        if response == "n":
            return False
        if response == "":
            return default


def address_2_board_cache_load() -> Dict[str, str]:
    try:
        with open(ADDRESS_2_BOARD_CACHE_PATH) as f:
            xs = json.load(f)

        if not isinstance(xs, dict):
            raise TypeError

        for k, v in xs.items():
            if not isinstance(k, str) or not isinstance(v, str):
                raise TypeError

        return xs
    except FileNotFoundError:
        return {}  # quieter to avoid alarming people
    except:
        logging.warning("unable to read address-board cache", exc_info=True)
        return {}


def address_2_board_cache_add(device_key: str, board: str):
    xs = address_2_board_cache_load()
    xs[device_key.lower()] = board

    try:
        with open(ADDRESS_2_BOARD_CACHE_PATH, "w") as f:
            json.dump(xs, f, sort_keys=True)
    except:
        logging.warning("failed to save address-2-board cache", exc_info=True)


def guess_board(args: CmdLnArgs) -> Optional[str]:
    device_key = args.bt_address or args.serial
    if device_key is not None:
        board = address_2_board_cache_load().get(device_key.lower())
        if board is not None:
            print(f"`{device_key}` was previously seen using `{board}`")
            return board

    if args.unattended:
        logging.error("unattended and unable to guess board type")
        return None

    print("Failed to determine automatically board type.")
    print("Please specify board type. e.g. pico_w")
    while True:
        board = input("Board type: ").strip()
        if board != "":
            return board


async def main(args: CmdLnArgs) -> int:
    if not args.validate():
        return 1

    release: Optional[ReleaseInfo] = None

    if args.file is None:
        if args.tag is None:
            release = await fetch_release_latest()
        else:
            release = await fetch_release_by_tag(args.tag)

        if release is None:
            return 1

        print(f"RELEASE TAG: {release.tag}")
        print(f"RELEASE DESCRIPTION:\n{release.info}\n")

        if not args.unattended:
            input("PRESS ENTER TO CONTINUE")

    prev_version = "<unknown>"
    prev_pin_state = PinState.UNKNOWN

    if (transport := await args.connect()) is not None:
        async with transport:
            prev_pin_state = await pin_config(transport)
            prev_version = await software_revision(transport)
            board = await hardware_board(transport)
            print(f"board  : {board}")
            print(f"version: {prev_version}")

            args.board = args.board or board
            if args.board != board:
                logging.warning(
                    f"board mismatch: override=`{args.board}`; reported={board}"
                )
                if not input_yes_no(
                    args, False, f"Do you wish to continue anyway using `{args.board}`?"
                ):
                    return 1

            device_key = args.bt_address or args.serial
            assert device_key is not None
            address_2_board_cache_add(device_key, args.board)

            assert args.file is not None or release is not None
            args.file = args.file or await download_board_update(release, args.board)
            if args.file is not None:
                await reboot_device(transport, bootloader=True)

        print(f"waiting for device to reboot ({REBOOT_DELAY} seconds)...")
        await asyncio.sleep(REBOOT_DELAY)
    else:
        print("unable to connect to nevermore controller.")
        print("attempting to connect to bootloader...")

        if args.file is None:
            assert release is not None
            args.board = args.board or guess_board(args)
            if args.board is not None:
                args.file = await download_board_update(release, args.board)

    if args.file is None:  # no file specified or didn't manage to download an update
        return 1

    if args.serial is not None:
        await _update_via_serial(args)
    else:
        await _update_via_bt_spp(args)

    print(
        """\n
----------------------
-- Update Completed --
----------------------
"""
    )

    try:
        if not args.unattended:
            print(
                """
Reconnecting for post-update actions.
It is recommended to wait so the updater can verify if there are post-update actions you should perform.
( You may abort if the following steps take too long. [ctrl-c] )
"""
            )
            print(f"waiting for device to reboot ({REBOOT_DELAY} seconds)...")
            await asyncio.sleep(REBOOT_DELAY)
            await _post_update_actions_interactive(args, prev_version, prev_pin_state)
    except asyncio.exceptions.CancelledError:
        pass
    except KeyboardInterrupt:
        pass

    return 0


if __name__ == "__main__":

    def go(args: CmdLnArgs):
        exit(asyncio.run(main(args)))

    tap.Parser(CmdLnArgs).bind(go).run()
