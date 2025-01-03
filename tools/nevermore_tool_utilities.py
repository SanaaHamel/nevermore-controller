import asyncio
import logging
from typing import Any, Callable, Coroutine, Optional

from bleak import BleakClient, BleakScanner
import bleak
import serial

import typed_argparse as tap
from bleak.backends.device import BLEDevice
from nevermore_utilities import (
    bt_address_validate,
    device_is_likely_a_nevermore,
    discover_bluetooth_devices,
    Transport,
    TransportBLE,
    TransportSerial,
    is_lost_connection_exception,
    retry_if_disconnected,
)

__all__ = [
    'NevermoreToolCmdLnArgs',
    'device_address_discover',
]


class NevermoreToolCmdLnArgs(tap.TypedArgs):
    bt_address: Optional[str] = tap.arg(help="device's BT address")
    serial: Optional[str] = tap.arg(help="device's serial port")
    baud_rate: int = tap.arg(help="baud rate (if applicable)", default=115200)

    def validate(self):
        ok = True

        if self.bt_address is not None and not bt_address_validate(self.bt_address):
            logging.error("invalid address for `--bt-address`")
            ok = False

        if self.bt_address is not None and self.serial is not None:
            logging.error("`--bt-address` and `--serial` are mutually exclusive")
            ok = False

        if self.serial is not None and self.serial.endswith("-if00"):
            logging.warning(
                "!? Using interface `-if00` (typically stdio).\n"
                + "You probably meant to specify `-if02` (GATT/Bootloader Protocol)."
            )

        return ok

    async def connect(self, timeout: Optional[float] = 10) -> Optional[Transport]:
        if self.serial:
            print(f"connecting to `{self.serial}`")
            try:
                if timeout is not None:
                    timeout /= 5  # serial connections don't need as long

                return TransportSerial(
                    serial.Serial(
                        self.serial,
                        self.baud_rate,
                        timeout=timeout,
                        write_timeout=timeout,
                        exclusive=True,
                    )
                )
            except TransportSerial.TimeoutException as e:
                # if read failed it might be in bootloader mode, just return `None`
                if e.read:
                    return None
                raise
        else:
            address = await device_address_discover(
                "Nevermore controllers", device_is_likely_a_nevermore, self.bt_address
            )
            if self.bt_address is None:
                self.bt_address = address
            if self.bt_address is None:  # no candidates found or specified
                return None

            print(f"connecting to {self.bt_address}")

            async def mk_client():
                assert self.bt_address is not None
                try:
                    client = BleakClient(
                        self.bt_address, timeout=10 if timeout is None else timeout
                    )
                    await client.connect()
                    return client
                except asyncio.TimeoutError:
                    return None
                except bleak.exc.BleakDeviceNotFoundError:
                    return None
                except KeyError as e:
                    # IDK why these aren't handled in bleak, but we have to handle their crap
                    if len(e.args) == 1 and e.args[0] in {
                        'org.bluez.Device1',
                        'org.bluez.GattService1',
                    }:
                        return None
                    raise e
                except Exception as e:
                    if is_lost_connection_exception(e):
                        return None
                    raise e

            if timeout is None:
                while (client := await mk_client()) is None:
                    pass
            else:
                client = await mk_client()

            return TransportBLE(client) if client is not None else None


async def device_address_discover(
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
