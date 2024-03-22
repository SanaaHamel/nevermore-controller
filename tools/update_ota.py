#!/bin/bash
"true" '''\'
set -eu
set -o pipefail

FILE="$(readlink -f "$0")"
ROOT_DIR="$(dirname "$FILE")"

TCP=0
NO_TMUX=0
IGNORE_KLIPPER=0
for ARG; do
  shift
  if [ "$ARG" = "--no-tmux" ]; then
    NO_TMUX=1
  elif [ "$ARG" = "--ignore-klipper" ]; then
    IGNORE_KLIPPER=1
  elif [ "$ARG" = "--tcp" ]; then
    TCP=1
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


# only run in `tmux` if we'd switch wifi (i.e. using `tcp`)
if [[ "$TCP" = 0 || "$NO_TMUX" = 1 ]]; then
  "$ROOT_DIR/setup-tool-env.bash"
  if [[ "$IGNORE_KLIPPER" = 0 ]]; then
    ensure_klipper_not_running
  fi
  "$ROOT_DIR/.venv/bin/python" "$FILE" "$@"
  exit 0
fi

if ! which tmux &>/dev/null; then
  echo "installing 'tmux'..."
  sudo apt-get install tmux -y
fi

escape() {
  printf "%q " "$@"
}

tmux new-session -A -s "nevermore-update" \
  "$(escape bash -c "$(escape "$FILE" --no-tmux "$@"); \
                     $(escape read -r -p "Press enter to continue")")"

exit 0 # required to stop shell execution here
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
from typing import Any, Callable, Coroutine, Iterable, List, Optional, Set, TypeVar

import serial_flash
import typed_argparse as tap
from aiohttp import ClientSession
from bleak import BleakClient
from bleak.backends.device import BLEDevice
from nevermore_utilities import *
from serial_flash.transport.bluetooth.spp import SppArgs
from serial_flash.transport.tcp import TcpArgs
from typing_extensions import override

if typing.TYPE_CHECKING:
    _LoggerAdapter = logging.LoggerAdapter[logging.Logger]
else:
    _LoggerAdapter = logging.LoggerAdapter

__all__ = ['CmdLnArgs', '_main']

NEVERMORE_OTA_WIFI_SSID = 'nevermore-update-ota'
NEVERMORE_OTA_WIFI_KEY = 'raccoons-love-floor-onions'

CONNECT_TO_AP_TIMEOUT = 20  # seconds
CONNECT_TO_AP_DELAY = 2  # seconds
REBOOT_DELAY = 1  # seconds

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


async def hardware_board(client: BleakClient):
    char_hardware = client.services.get_characteristic(UUID_CHAR_HARDWARE_REVISION)
    if char_hardware is None:
        logging.warning(
            "device does not have a hardware revision characteristic, assuming board is `pico_w`"
        )
        return "pico_w"

    return str(await client.read_gatt_char(char_hardware), "UTF-8")


async def software_revision(client: BleakClient):
    char_revision = client.services.get_characteristic(UUID_CHAR_SOFTWARE_REVISION)
    if char_revision is None:
        logging.warning("device does not have a software revision characteristic")
        return "<unknown>"

    HEADER = "commit: "
    DIRTY = "-dirty"

    revision = str(await client.read_gatt_char(char_revision), "UTF-8")
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


async def reboot_into_ota_mode(client: BleakClient):
    char_reboot = client.services.get_characteristic(UUID_CHAR_CONFIG_REBOOT)
    if char_reboot is None:
        logging.error(
            "Nevermore is too old to update with OTA. Manually flash the latest UF2 to it."
        )
        return None

    print(f"sending reboot-to-OTA command...")
    # b"\1" to reboot to OTA, "0" to restart
    await client.write_gatt_char(char_reboot, b"\1")
    return client.address


def _ota_ap_visible(bt_address: str) -> bool:  # type: ignore unused
    try:
        subprocess.run(
            [
                "nmcli",
                "device",
                "wifi",
                "list",
                "bssid",
                _bssid_from_bt_address(bt_address),
                "--rescan",
                "yes",
            ],
            text=True,
            check=True,
            stdout=open("/dev/null"),
        )
        return True
    except subprocess.CalledProcessError:
        return False


class ConnectToWifiAccessPoint:
    def __init__(
        self,
        bssid: Optional[str] = None,
        *,
        timeout: float = CONNECT_TO_AP_TIMEOUT,
        retry_delay: float = CONNECT_TO_AP_DELAY,
    ):
        assert 0 <= timeout
        assert 0 < retry_delay
        self._bssid = bssid
        self._timeout = timeout
        self._retry_delay = retry_delay
        self._closed = False

    async def __aenter__(self):
        print("waiting for OTA access point...")
        if self._timeout == 0:
            await asyncio.wait_for(self._connect_forever(), self._timeout)
        else:
            await self._connect_forever()

        return self

    async def __aexit__(self, *_args: Any):
        if not self._closed:
            self._closed = True
            await self._disconnect()

    async def _connect_forever(self):
        while not await self._connect_attempt():
            await asyncio.sleep(self._retry_delay)

    @abstractmethod
    async def _connect_attempt(self) -> bool:
        raise NotImplementedError()

    @abstractmethod
    async def _disconnect(self) -> None:
        raise NotImplementedError()


class ConnectToWifiAccessPointNetworkManager(ConnectToWifiAccessPoint):
    @override
    async def _connect_attempt(self):
        subprocess.check_call(["nmcli", "device", "wifi", "rescan"])

        try:
            subprocess.check_call(
                [
                    "nmcli",
                    "device",
                    "wifi",
                    "connect",
                    NEVERMORE_OTA_WIFI_SSID if self._bssid is None else self._bssid,
                    "password",
                    NEVERMORE_OTA_WIFI_KEY,
                    "name",
                    NEVERMORE_OTA_WIFI_SSID,
                ]
            )
            return True
        except subprocess.CalledProcessError as e:
            if e.returncode not in {4, 10}:
                raise

        return False

    @override
    async def _disconnect(self):
        try:
            subprocess.check_call(
                ["nmcli", "connection", "down", NEVERMORE_OTA_WIFI_SSID]
            )
        except subprocess.CalledProcessError as e:
            if e.returncode not in {10}:  # 10 -> no active connection provided
                raise


class CmdLnArgs(tap.TypedArgs):
    bt_address: Optional[str] = tap.arg(help="device's BT adddress")
    file: Optional[Path] = tap.arg(help="filepath for image")
    tag: Optional[str] = tap.arg(help="download specific release tag")
    board: Optional[str] = tap.arg(help="override/specify board (USE WITH CAUTION!)")
    # `--no-tmux` and `--ignore-klipper` are used/handled by the shell wrapper script.
    # They're listed here for `--help` and unknown-argument checking.
    no_tmux: bool = tap.arg(help="don't run in a tmux session")
    ignore_klipper: bool = tap.arg(help="don't check if klipper is running")
    unattended: bool = tap.arg(help="do not ask user for input", default=False)
    tcp: bool = tap.arg(help="connect via TCP instead of BT SPP")
    ip: str = tap.arg(
        default="192.168.4.1", help="if non-empty, connect via TCP to given IP"
    )
    port: int = tap.arg(
        default=4242, help="port use when connecting via TCP, see `--ip`"
    )


async def _update_via_tcp(args: CmdLnArgs, bssid: Optional[str]):
    if args.file is None:
        logging.error("no image to upload specified")
        return

    if bssid is None:
        logging.warning("no BSSID specified, attempting to connect by SSID...")
    else:
        print("waiting for OTA access point...")

    async with ConnectToWifiAccessPointNetworkManager(bssid):
        serial_flash.run(
            TcpArgs(
                filename=str(args.file.absolute()),
                ip=args.ip or "192.168.4.1",
                port=args.port,
            )
        )


async def _update_via_bt_spp(args: CmdLnArgs):
    if args.file is None:
        logging.error("no image to upload specified")
        return

    if args.bt_address is None:
        logging.error("no BT address specified")
        return

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


async def _report_new_version(args: CmdLnArgs, prev_version: str):
    if args.bt_address is None:
        return

    print(f"connecting to {args.bt_address} to get installed version")
    print("(this may take longer than usual)")
    print(
        "NOTE: Ignore logged exceptions about `A message handler raised an exception: 'org.bluez.Device1'.`"
    )
    print(
        "      This is caused by a bug in `bleak` but should be benign for this application."
    )

    async def go(client: BleakClient):
        curr_version = await software_revision(client)
        print(f"previous version: {prev_version}")
        print(f" current version: {curr_version}")

    await retry_if_disconnected(args.bt_address, go, connection_timeout=None)


def input_yes_no(args: CmdLnArgs, default: _A, msg: str) -> Union[_A, bool]:
    if args.unattended:
        return default

    while True:
        response = input(f"{msg} (y/n)")
        if response == "y":
            return True
        if response == "n":
            return False


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


def address_2_board_cache_add(addr: str, board: str):
    xs = address_2_board_cache_load()
    xs[addr.lower()] = board

    try:
        with open(ADDRESS_2_BOARD_CACHE_PATH, "w") as f:
            json.dump(xs, f, sort_keys=True)
    except:
        logging.warning("failed to save address-2-board cache", exc_info=True)


def guess_board(args: CmdLnArgs) -> Optional[str]:
    if args.bt_address is not None:
        board = address_2_board_cache_load().get(args.bt_address.lower())
        if board is not None:
            print(f"{args.bt_address} was previously seen using `{board}`")
            return board

    if args.unattended:
        logging.error("unattended and unable to guess board")
        return None

    print("Unable to connect to nevermore to query board. Please specify board.")
    while True:
        board = input("Board: ").strip()
        if board != "":
            return board


async def _main(args: CmdLnArgs) -> int:
    if args.bt_address is not None and not bt_address_validate(args.bt_address):
        logging.error("invalid address for `--bt-address`")
        return 1

    if args.file is not None and args.tag is not None:
        logging.error("`--tag` and `--file` are mutually exclusive")
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

    address_found = await discover_device_address(
        "Nevermore controllers",
        device_is_likely_a_nevermore,
        args.bt_address,
    )
    if args.bt_address is None:
        args.bt_address = address_found

    prev_version = "<unknown>"

    if address_found is not None:
        print(f"connecting to {address_found}")

        async def go(client: BleakClient):
            nonlocal prev_version
            prev_version = await software_revision(client)
            board = await hardware_board(client)
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
                    return

            address_2_board_cache_add(client.address, args.board)

            assert args.file is not None or release is not None
            args.file = args.file or await download_board_update(release, args.board)
            if args.file is not None:
                await reboot_into_ota_mode(client)

        await retry_if_disconnected(address_found, go, connection_timeout=None)
    else:
        print("attempting to connect to bootloader anyways...")

        if args.file is None:
            assert release is not None
            args.board = args.board or guess_board(args)
            if args.board is not None:
                args.file = await download_board_update(release, args.board)

    if args.file is None:  # no file specified or didn't manage to download an update
        return 1

    if args.tcp:
        bssid = (
            None if args.bt_address is None else _bssid_from_bt_address(args.bt_address)
        )
        await _update_via_tcp(args, bssid)
    else:
        # FIXME: Can't scan for bootloaders w/ this API b/c they're BT devices, not BLE devices.
        # address_found = await discover_device_address(
        #     "bootloaders",
        #     lambda x: x.name is not None and x.name.startswith("picowota"),
        #     args.bt_address,
        # )
        # if args.bt_address is None:
        #     args.bt_address = address_found
        await _update_via_bt_spp(args)

    print("update complete.")

    try:
        if not args.unattended:
            print("You may safely abort if the following steps take too long [ctrl-c].")
            print(f"waiting for device to reboot ({REBOOT_DELAY} seconds)...")
            await asyncio.sleep(REBOOT_DELAY)
            await _report_new_version(args, prev_version)
    except asyncio.exceptions.CancelledError:
        pass
    except KeyboardInterrupt:
        pass

    return 0


def main():
    def go(args: CmdLnArgs):
        exit(asyncio.run(_main(args)))

    tap.Parser(CmdLnArgs).bind(go).run()


if __name__ == "__main__":
    main()
