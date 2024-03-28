import logging
from typing import Any, Callable, Coroutine, Optional

import typed_argparse as tap
from bleak.backends.device import BLEDevice
from nevermore_utilities import (
    bt_address_validate,
    device_is_likely_a_nevermore,
    discover_bluetooth_devices,
)

__all__ = [
    'NevermoreToolCmdLnArgs',
    'device_address_discover',
]


class NevermoreToolCmdLnArgs(tap.TypedArgs):
    bt_address: Optional[str] = tap.arg(help="device's BT address")

    def validate(self):
        ok = True

        if self.bt_address is not None and not bt_address_validate(self.bt_address):
            logging.error("invalid address for `--bt-address`")
            ok = False

        return ok

    async def bt_address_discover(self):
        address = await device_address_discover(
            "Nevermore controllers", device_is_likely_a_nevermore, self.bt_address
        )
        if self.bt_address is None:
            self.bt_address = address

        return address


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
