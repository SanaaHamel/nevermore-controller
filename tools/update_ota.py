#!/bin/sh
"true" '''\'
echo "$(dirname "$(readlink -f "$0")")"/venv/bin/python "$0" "$@"
FILE="$(readlink -f "$0")"
ROOT_DIR="$(dirname "$FILE")"
"$ROOT_DIR/setup-tool-env.bash" && "$ROOT_DIR/.venv/bin/python" "$FILE" "$@"
exit $?
'''

# Script for updating a Nevermore controller.
#
# Copyright (C) 2023       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

__doc__ = """Script for updating Nevermore controllers."""

import asyncio
import datetime
import json
import logging
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import List, Optional
from uuid import UUID

import serial_flash
import typed_argparse as tap
from aiohttp import ClientSession
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from serial_flash.transport.tcp import TcpArgs

NEVERMORE_OTA_WIFI_SSID = 'nevermore-update-ota'
NEVERMORE_OTA_WIFI_KEY = 'raccoons-love-floor-onions'

BT_SCAN_GATHER_ALL_TIMEOUT = 10  # seconds
CONNECT_TO_AP_TIMEOUT = 20  # seconds
CONNECT_TO_AP_DELAY = 2  # seconds

OTA_UPDATE_FILENAME = "nevermore-controller-ota.elf"

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

UUID_CHAR_SOFTWARE_REVISION = short_uuid(0x2B04)
UUID_CHAR_CONFIG_REBOOT = UUID("f48a18bb-e03c-4583-8006-5b54422e2045")

UUID_NEVERMORE_OTA_WIFI = UUID("f234a256-b826-4392-b79e-5106bd1e19dd")


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


@dataclass
class ReleaseInfo:
    tag: str
    published: datetime.datetime
    assets: List[str]


async def discover_controllers(
    address: Optional[str] = None, timeout: float = BT_SCAN_GATHER_ALL_TIMEOUT
) -> List[BLEDevice]:
    NAME_SHORTENED = "Nevermore"
    NAME_COMPLETE = "Nevermore Controller"

    if address is not None:
        device = await BleakScanner.find_device_by_address(address, timeout=timeout)
        return [device] if device is not None else []

    return [
        x
        for x in await BleakScanner.discover(timeout=timeout)
        if x.name in {NAME_SHORTENED, NAME_COMPLETE}
    ]


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
                published=datetime.datetime.fromisoformat(content["published_at"]),
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


async def reboot_into_ota_mode(bt_address: Optional[str]):
    print("discovering Nevermores...")
    xs = await discover_controllers(bt_address)
    if not xs:
        logging.error("no Nevermore controllers found")
        return False

    if 1 < len(xs):
        logging.error(
            "multiple Nevermore controllers found. use `--bt-address` to disambiguate"
        )
        logging.error("Nevermore controllers found:")
        xs.sort(key=lambda x: x.address)
        for x in xs:
            logging.error(x)
        return False

    print(f"connecting to {xs[0].address}")
    async with BleakClient(xs[0]) as client:
        char_reboot = client.services.get_characteristic(UUID_CHAR_CONFIG_REBOOT)
        if char_reboot is None:
            logging.error(
                "Nevermore is too old to update with OTA. Manually flash the latest UF2 to it."
            )
            return False

        print(f"sending reboot-to-OTA command...")
        # b"\1" to reboot to OTA, "0" to restart
        await client.write_gatt_char(char_reboot, b"\1")

    return True


async def _connect_to_ota_ap():
    subprocess.check_call(["nmcli", "device", "wifi", "rescan"])

    while True:
        try:
            subprocess.check_call(
                [
                    "nmcli",
                    "device",
                    "wifi",
                    "connect",
                    NEVERMORE_OTA_WIFI_SSID,
                    "password",
                    NEVERMORE_OTA_WIFI_KEY,
                    "name",
                    NEVERMORE_OTA_WIFI_SSID,
                ]
            )
            break
        except subprocess.CalledProcessError as e:
            if e.returncode != 10:
                raise

        time.sleep(CONNECT_TO_AP_DELAY)


class CmdLnArgs(tap.TypedArgs):
    bt_address: Optional[str] = tap.arg(help="device's BT adddress")
    file: Optional[Path] = tap.arg(help="filepath for image")


async def _main(args: CmdLnArgs):
    print("This program will attempt to update a Nevermore controller.")
    print("The host will disconnect from the current WiFi, and then later reconnect.")
    print("!! IF YOU'RE SSH-ING YOU **MUST** RUN THIS SCRIPT USING `nohup` !!")
    print("   e.g. `nohup ./update_ota.py ...`")
    print("-------------------------------------------------------------------------")
    print()
    # await create_connection_profile()

    if args.bt_address is not None and not _bt_address_validate(args.bt_address):
        logging.error("invalid address for `--bt-address`")
        return

    if args.file is None:
        print("downloading latest update...")
        temp_file = await download_update()
        if temp_file is None:
            return

        args.file = Path(temp_file.name)

    if await reboot_into_ota_mode(args.bt_address):
        print("waiting for OTA access point...")
        await asyncio.wait_for(_connect_to_ota_ap(), CONNECT_TO_AP_TIMEOUT)
    else:
        print("attempting to connect to OTA access point anyways")
        await asyncio.wait_for(_connect_to_ota_ap(), CONNECT_TO_AP_TIMEOUT / 4)

    try:
        tcp_args = TcpArgs(
            filename=str(args.file.absolute()), ip="192.168.4.1", port=4242
        )
        serial_flash.run(tcp_args)
    finally:
        subprocess.check_call(["nmcli", "connection", "down", NEVERMORE_OTA_WIFI_SSID])


def main():
    def go(args: CmdLnArgs):
        asyncio.run(_main(args))

    tap.Parser(CmdLnArgs).bind(go).run()


if __name__ == "__main__":
    main()
