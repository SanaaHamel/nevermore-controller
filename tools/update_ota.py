#!/bin/bash
"true" '''\'
set -eu
set -o pipefail

FILE="$(readlink -f "$0")"
ROOT_DIR="$(dirname "$FILE")"

TCP=0
NO_TMUX=0
for ARG; do
  shift
  if [ "$ARG" = "--no-tmux" ]; then
    NO_TMUX=1
  elif [ "$ARG" = "--tcp" ]; then
    TCP=1
  fi
  set -- "$@" "$ARG"
done

# only run in `tmux` if we'd switch wifi (i.e. using `tcp`)
if [[ "$TCP" = 0 || "$NO_TMUX" = 1 ]]; then
  "$ROOT_DIR/setup-tool-env.bash"
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
import subprocess
from abc import abstractmethod
from dataclasses import dataclass
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Any, Callable, Coroutine, List, Optional, TypeVar, Union
from uuid import UUID

import bleak
import serial_flash
import typed_argparse as tap
from aiohttp import ClientSession
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from serial_flash.transport.bluetooth.spp import SppArgs
from serial_flash.transport.tcp import TcpArgs
from typing_extensions import override


NEVERMORE_CONTROLLER_NAMES = {"Nevermore", "Nevermore Controller"}
BOOTLOADER_NAME_PREFIX = "picowota "

NEVERMORE_OTA_WIFI_SSID = 'nevermore-update-ota'
NEVERMORE_OTA_WIFI_KEY = 'raccoons-love-floor-onions'

BT_SCAN_GATHER_ALL_TIMEOUT = 10  # seconds
CONNECT_TO_AP_TIMEOUT = 20  # seconds
CONNECT_TO_AP_DELAY = 2  # seconds
REBOOT_DELAY = 1  # seconds

OTA_UPDATE_FILENAME = "picowota_nevermore-controller-ota.uf2"

URL_RELEASES_FETCH = (
    "https://api.github.com/repos/sanaahamel/nevermore-controller/releases?per_page=1"
)

FETCH_RELEASE_HEADERS = {
    "Accept": "application/vnd.github+json",
    "X-GitHub-Api-Version": "2022-11-28",
}

DOWNLOAD_ASSET_HEADERS = {
    "Accept": "application/octet-stream",
    "X-GitHub-Api-Version": "2022-11-28",
}


def short_uuid(x: int):
    assert 0 <= x <= 0xFFFF
    return UUID(f"0000{x:04x}-0000-1000-8000-00805f9b34fb")


UUID_SERVICE_GAP = short_uuid(0x1801)
UUID_SERVICE_ENVIRONMENTAL_SENSING = short_uuid(0x181A)

UUID_CHAR_SOFTWARE_REVISION = short_uuid(0x2A28)
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


def _bssid_from_bt_address(bt_address: str):
    # The BSSID of a Pico is by default 1 less than that of its BT address.
    assert _bt_address_validate(bt_address)
    bssid = bt_address.split(':')
    bssid[-1] = f"{(int(bssid[-1], 16) - 1) & 0xFF:02x}"
    return ":".join(bssid)


@dataclass
class ReleaseInfo:
    tag: str
    assets: List[str]


async def discover_bluetooth_devices(
    filter: Callable[[BLEDevice], Coroutine[Any, Any, bool]],
    address: Optional[str] = None,
    timeout: float = BT_SCAN_GATHER_ALL_TIMEOUT,
) -> List[BLEDevice]:
    if address is not None:
        device = await BleakScanner.find_device_by_address(address, timeout=timeout)
        return [device] if device is not None else []

    return [x for x in await BleakScanner.discover(timeout=timeout) if await filter(x)]


async def fetch_latest_release() -> Optional[ReleaseInfo]:
    async with ClientSession() as session:
        async with session.get(
            URL_RELEASES_FETCH, headers=FETCH_RELEASE_HEADERS
        ) as response:
            content = await response.read()
            if response.status != 200:
                logging.error(f"fetch releases failed. http code={response.status}")
                return None

            content = json.loads(content)[0]
            return ReleaseInfo(
                tag=content["tag_name"],
                assets=[x["browser_download_url"] for x in content["assets"]],
            )


async def fetch_asset(asset_url: str):
    async with ClientSession() as session:
        async with session.get(asset_url, headers=DOWNLOAD_ASSET_HEADERS) as response:
            content = await response.read()
            if response.status != 200:
                logging.error(f"download asset failed. http code={response.status}")
                return None

            file = NamedTemporaryFile(suffix=OTA_UPDATE_FILENAME)
            file.write(content)
            return file


async def download_update():
    release = await fetch_latest_release()
    if release is None:
        return None

    assert_candidates = [x for x in release.assets if x.endswith(OTA_UPDATE_FILENAME)]
    assert len(assert_candidates) <= 1
    if not assert_candidates:
        logging.error(
            f"latest release does not have an asset named `{OTA_UPDATE_FILENAME}`"
        )
        return None

    return await fetch_asset(assert_candidates[0])


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


def _is_lost_connection_exception(e: Exception) -> bool:
    if isinstance(e, bleak.exc.BleakError):
        msg = str(e).lower()
        if "not connected" in msg:
            return True
        if "device disconnected" in msg:
            return True

    return False


async def retry_if_disconnected(
    device: Union[BLEDevice, str],
    go: Callable[[BleakClient], Coroutine[Any, Any, _A]],
    *,
    connection_timeout: Optional[float] = 10,
) -> _A:
    assert connection_timeout is None or 0 < connection_timeout
    while True:
        is_connected = False
        try:
            async with BleakClient(
                device, timeout=10 if connection_timeout is None else connection_timeout
            ) as client:
                is_connected = True
                return await go(client)
        except asyncio.TimeoutError:
            # `TimeoutError` after we've connected aren't a connection timeout
            # and must not be suppressed.
            if is_connected or connection_timeout is not None:
                raise
        except bleak.exc.BleakDeviceNotFoundError:
            if connection_timeout is not None:
                raise
        except bleak.exc.BleakError as e:
            if not _is_lost_connection_exception(e):
                raise

            logging.info("lost connection; retrying...", exc_info=e)


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
    # `--no-tmux` is used/handled by the shell wrapper script. list it here for `--help`
    no_tmux: bool = tap.arg(help="don't run in a tmux session")
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
            logging.info(f"potentially transient comm error, retrying...", exc_info=e)


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

    while True:
        try:
            async with BleakClient(args.bt_address) as client:
                curr_version = await software_revision(client)
                print(f"previous version: {prev_version}")
                print(f" current version: {curr_version}")
                break
        except TimeoutError:
            pass
        except bleak.exc.BleakError as e:
            if "not connected" not in str(e).lower():
                raise


async def _main(args: CmdLnArgs):
    if args.bt_address is not None and not _bt_address_validate(args.bt_address):
        logging.error("invalid address for `--bt-address`")
        return

    if args.file is None:
        print("downloading latest update...")
        temp_file = await download_update()
        if temp_file is None:
            return

        args.file = Path(temp_file.name)

    address_found = await discover_device_address(
        "Nevermore controllers",
        device_is_likely_a_nevermore,
        args.bt_address,
    )
    if args.bt_address is None:
        args.bt_address = address_found

    if address_found is not None:
        print(f"connecting to {address_found}")
        async with BleakClient(address_found) as client:
            prev_version = await software_revision(client)
            print(f"current revision: {prev_version}")
            await reboot_into_ota_mode(client)
    else:
        prev_version = "<unknown>"
        print("attempting to connect to bootloader anyways...")

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
    print("(You may safely abort if the following steps take too long [ctrl-c].)")

    try:
        print(f"waiting for device to reboot ({REBOOT_DELAY} seconds)...")
        await asyncio.sleep(REBOOT_DELAY)  # block b/c we need to wait for it to reboot
        await _report_new_version(args, prev_version)
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
