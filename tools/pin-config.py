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

# Script for updating the pin config of a Nevermore controller.
#
# Copyright (C) 2024       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

__doc__ = """Script for updating Nevermore controllers."""

import asyncio
import copy
import json
import logging
import os
import re
import textwrap
from pathlib import Path
from typing import Optional, Sequence, Set, TypeVar
from typing_extensions import Buffer

import construct as cs
import typed_argparse as tap
from bleak import BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
from construct_typed import DataclassMixin, DataclassStruct, EnumBase, TEnum
from construct_typed.generic_wrapper import Adapter as CsAdapter
from dataclasses_json import DataClassJsonMixin
from nevermore_tool_utilities import NevermoreToolCmdLnArgs
from nevermore_utilities import *

__all__ = ['CmdLnArgs', 'main']


PIN_CONFIG_FILE_DEFAULT = "pin-config.jsonc"
PIN_CONFIG_FILE_DEFAULT_PATH = Path(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), PIN_CONFIG_FILE_DEFAULT)
)

ALTERNATIVES_MAX = 8

_A = TypeVar("_A")
_B = TypeVar("_B")

GPIO = Optional[int]
GPIOs = List[GPIO]
# has to be `Any` to shut up `dataclasses_json` warning
# (because it doesn't consider that a field of type `None` could be populated with `None`)
Padding = Any


# customise `csfield` because the library one doesn't handle default values sanely
def csfield(
    subcon: "cs.Construct[_A, Any]",
    doc: Optional[str] = None,
    parsed: "Optional[Callable[[Any, cs.Context], None]]" = None,
) -> _A:
    orig_subcon = subcon

    # Rename subcon, if doc or parsed are available
    if (doc is not None) or (parsed is not None):
        if doc is not None:
            doc = textwrap.dedent(doc).strip("\n")
        subcon = cs.Renamed(subcon, newdocs=doc, newparsed=parsed)

    if orig_subcon.flagbuildnone is True:
        default = None
    else:
        default = dataclasses.MISSING

    # Set default values in case of special sucons
    if isinstance(orig_subcon, cs.Const):
        const_subcon: "cs.Const[Any, Any]" = orig_subcon
        default = const_subcon.value
    elif isinstance(orig_subcon, cs.Default):
        default_subcon: "cs.Default[Any, Any]" = orig_subcon
        if callable(default_subcon.value):
            default = None  # context lambda is only defined at parsing/building
        else:
            default = default_subcon.value

    # mutable values must be generated via `default_factory`
    if default.__class__.__hash__ is None:
        field = dataclasses.field(
            default_factory=lambda: copy.copy(default),
            init=True,
            metadata={"subcon": subcon},
        )
    else:
        field = dataclasses.field(
            default=default,
            init=True,
            metadata={"subcon": subcon},
        )

    return typing.cast(_A, field)


class GPIOFormat(CsAdapter[int, int, Optional[int], Optional[int]]):
    def __init__(self):
        super().__init__(cs.Int8ul)
        self.flagbuildnone = True

    # HACK: typo in official type-annotations where it uses `SubconBuildTypes` instead of `SubconParsedType`
    def _decode(self, obj: int, context: "cs.Context", path: Any):  # type: ignore
        return obj - 1 if obj != 0 else None

    def _encode(self, obj: Optional[int], context: "cs.Context", path: Any):
        return obj + 1 if obj is not None else 0


class PadArray(CsAdapter[Sequence[_A], List[_B], List[_A], List[_B]]):
    def __init__(
        self,
        subcon: "cs.Construct[Sequence[_A], List[_B]]",
        default: _B,
        filter: Optional[Callable[[_A], bool]] = None,
    ):
        super().__init__(subcon)
        self.flagbuildnone = True
        self.default = default
        self.filter: Callable[[_A], bool] = (
            filter if filter is not None else lambda x: x != self.default
        )

    # HACK: typo in official type-annotations where it uses `SubconBuildTypes` instead of `SubconParsedType`
    def _decode(self, obj: Sequence[_A], context: "cs.Context", path: Any):  # type: ignore
        return list(filter(self.filter, obj))

    def _encode(self, obj: Optional[List[_B]], context: "cs.Context", path: Any):
        obj = obj if obj is not None else []
        return list(obj) + [self.default for _ in range(ALTERNATIVES_MAX - len(obj))]


def csfield_array(
    construct: "cs.Construct[_A, _B]",
    default: _B,
    filter: Optional[Callable[[_A], bool]] = None,
) -> List[_A]:
    return csfield(
        cs.Default(PadArray(cs.Array(ALTERNATIVES_MAX, construct), default, filter), [])
    )


def csfield_gpio():
    return csfield(GPIOFormat())


def csfield_gpios():
    return csfield_array(GPIOFormat(), None)


def csfield_pad(n: int):
    return csfield(cs.Default(cs.Padding(n), None))


class NamedEnumBase(EnumBase):
    # override to allow lookup by name during construction from JSON
    @classmethod
    def _missing_(cls, value: Any):
        try:
            if isinstance(value, str):
                return cls[value]
        except KeyError:
            pass

        return super()._missing_(value)


class KindI2C(NamedEnumBase):
    generic = 0
    intake = 1
    exhaust = 2


class KindSPI(NamedEnumBase):
    generic = 0
    display = 1


@dataclass
class BusI2C(DataclassMixin):
    kind: KindI2C = csfield(TEnum(cs.Int8ul, KindI2C))
    clock: GPIO = csfield_gpio()
    data: GPIO = csfield_gpio()
    baud_rate: int = csfield(cs.Default(cs.Int32ul, 0))

    def active(self) -> bool:
        return any(x is not None for x in [self.clock, self.data])


@dataclass
class BusSPI(DataclassMixin):
    kind: KindSPI = csfield(TEnum(cs.Int8ul, KindSPI))
    clock: GPIO = csfield_gpio()
    send: GPIO = csfield_gpio()
    recv: GPIO = csfield_gpio()
    select: GPIO = csfield_gpio()
    baud_rate: int = csfield(cs.Default(cs.Int32ul, 0))

    def active(self):
        return any(
            x is not None for x in [self.clock, self.send, self.recv, self.select]
        )


@dataclass
class Pins(DataclassMixin, DataClassJsonMixin):
    i2c: List[BusI2C] = csfield_array(
        DataclassStruct(BusI2C),
        BusI2C(KindI2C.generic, baud_rate=0),
        lambda x: x.active(),
    )
    spi: List[BusSPI] = csfield_array(
        DataclassStruct(BusSPI),
        BusSPI(KindSPI.generic, baud_rate=0),
        lambda x: x.active(),
    )
    fan_pwm: GPIOs = csfield_gpios()
    fan_tachometer: GPIOs = csfield_gpios()
    neopixel_data: GPIOs = csfield_gpios()
    photocatalytic_pwm: GPIO = csfield_gpio()
    vent_servo_pwm: GPIO = csfield_gpio()
    _pad_0: Padding = csfield_pad(6)
    display_command: GPIO = csfield_gpio()
    display_reset: GPIO = csfield_gpio()
    display_brightness_pwm: GPIO = csfield_gpio()
    touch_interrupt: GPIO = csfield_gpio()
    touch_reset: GPIO = csfield_gpio()
    led_status_voc_calibration: GPIO = csfield_gpio()
    _pad_tail: Padding = csfield_pad(32 + 2)

    def to_json_pretty(self):
        def transform(x: Any) -> Any:
            if isinstance(x, dict):
                # drop `_` prefixed members (e.g. padding)
                return {
                    k: transform(v)
                    for k, v in x.items()
                    if not (isinstance(k, str) and k.startswith("_"))
                }

            if isinstance(x, list):
                return [transform(y) for y in x]

            if isinstance(x, NamedEnumBase):
                return x.name  # give str display form in JSON

            return x

        return json.dumps(transform(self.to_dict(encode_json=False)), indent=2)


PINS_FORMAT = DataclassStruct(Pins)


class CmdLnArgs(NevermoreToolCmdLnArgs):
    file: Path = tap.arg(
        help="filepath for pin config", default=PIN_CONFIG_FILE_DEFAULT_PATH
    )
    echo_current: bool = tap.arg(help="echo current pin config", default=False)
    echo_default: bool = tap.arg(help="echo default pin config", default=False)
    reset_default: bool = tap.arg(help="reset pin config to default", default=False)


def input_options(msg: str, default: str, opts: Set[str]):
    opts = copy.copy(opts)
    opts.add(default)
    entries = [
        f"{x} [def]" if x == default else x
        for x in sorted(opts, key=lambda x: (x != default, x))
    ]
    prompt = f"{msg}\n *) " + "\n *) ".join(entries) + "\n"

    while True:
        response = input(prompt).lower().strip()
        if response == "":
            return default
        if response in opts:
            return response

        print(f"Invalid response `{response}`.")
        print(f"Enter one of the options listed. e.g. `{entries[0]}`")


@dataclass
class ControllerChars:
    current: Transport.Attribute
    default: Transport.Attribute
    error_msg: Transport.Attribute
    reboot: Transport.Attribute


def pins_chars(transport: Transport):
    P = CharacteristicProperty
    return ControllerChars(
        transport(UUID_CHAR_CONFIG_PINS, {P.READ, P.WRITE}),
        transport(UUID_CHAR_CONFIG_PINS_DEFAULT),
        transport(UUID_CHAR_CONFIG_PINS_ERROR),
        transport(UUID_CHAR_CONFIG_REBOOT, {P.WRITE}),
    )


async def pins_reset(transport: Transport) -> bool:
    raw = await pins_chars(transport).default.read()
    return await pins_write(transport, raw)


async def pins_read(transport: Transport, current: bool = True) -> Optional[Pins]:
    chars = pins_chars(transport)
    raw = await (chars.current if current else chars.default).read()

    try:
        return PINS_FORMAT.parse(raw)
    except cs.ConstructError:
        logging.exception(f"failed to parse (version mismatch?)", exc_info=True)
        return None


async def pins_write(transport: Transport, pins: Union[Pins, bytes]) -> bool:
    chars = pins_chars(transport)

    if isinstance(pins, Pins):
        pins = PINS_FORMAT.build(pins)

    try:
        await chars.current.write(pins)
    except:
        logging.exception("pin write failed, fetching error msg", exc_info=True)
        msg = await chars.error_msg.read()
        logging.error(f"pin config error: {msg}")
        return False

    print("Pin config set.")
    while True:
        answer = input(f"Reboot the controller? (Y/n)").lower().strip()
        if answer == "n":
            return True

        if answer in {"y", ""}:
            await chars.reboot.write(b'\x00')
            return True


async def using(
    args: NevermoreToolCmdLnArgs, go: Callable[[Transport], Coroutine[Any, Any, _A]]
) -> Optional[_A]:
    if (transport := await args.connect()) is None:
        return None

    async with transport:
        return await go(transport)


async def pins_to_file(args: CmdLnArgs):
    opt = input_options(
        "Do you wish to download a config from the controller? [ctrl-c to abort]",
        "current",
        {"default"},
    )

    pins = await using(args, lambda x: pins_read(x, current=opt == "current"))
    if pins is None:
        return False

    with open(args.file, "x") as f:
        f.write(
            """
// Configuration will be validated before being applied.
// Invalid configurations will be reported & rejected.
//
// 'Pin #' specifies the GPIO to use, which **might not** match the pin numbering
// on the board. Triple check the board's documentation to be sure.
//
// Some pins are reserved.
// Some functions can only be bound to specific pins.
// Pins can usually be omitted or set to `null` if the functionality isn't needed.
//
// Some functionality can be bound to multiple pins (array).
// Exact semantics of this varies by function.
// Output functions usually mirror across pins.
// Input functions usually commutatively fold their input. (e.g. Tacho takes sum of pulses.)
//
"""
        )
        f.write(pins.to_json_pretty())

    print(f"Wrote config to: {args.file}")
    print("Edit the config to your liking, then re-run this tool.")
    return True


async def main(args: CmdLnArgs) -> int:
    if not args.validate():
        return 1

    if args.echo_current or args.echo_default:
        pins = await using(args, lambda x: pins_read(x, current=args.echo_current))
        if pins is None:
            return 1

        print(pins.to_json_pretty())
        return 0

    if args.reset_default:
        return 0 if await using(args, pins_reset) else 1

    try:
        with open(args.file, "r") as f:
            file_json = f.read()
    except FileNotFoundError:
        print(f"File `{args.file}` does not exist.")
        return 0 if await pins_to_file(args) else 1

    # strip off any C-like comments (dumb)
    file_json = re.sub(r"\/\/[^\n]*\n", "\n", file_json)

    try:
        pins: Pins = Pins.from_json(file_json)  # type: ignore
    except TypeError as e:
        logging.error(e, exc_info=e)
        # FIXME: This sucks and should at least list the field in question.
        print(f"Malformed config file: {args.file}")
        print("Are all the fields of the correct type?")
        return 1
    except KeyError as e:
        print(f"Malformed config file: {args.file}")
        print(f"A struct is missing field `{e.args[0]}`")
        return 1
    except ValueError as e:
        logging.error(e)
        print(f"Malformed config file: {args.file}")
        return 1

    if await using(args, lambda x: pins_write(x, pins)) != True:
        return 1

    return 0


if __name__ == "__main__":

    def go(args: CmdLnArgs):
        exit(asyncio.run(main(args)))

    tap.Parser(CmdLnArgs).bind(go).run()
