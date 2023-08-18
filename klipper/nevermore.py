# Nevermore Controller Interface
#
# Copyright (C) 2023       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

import asyncio
import dataclasses
import datetime
import enum
import logging
import threading
import typing
import weakref
from abc import abstractmethod
from dataclasses import dataclass
from enum import Enum
from threading import Thread
from typing import (
    Any,
    Callable,
    Coroutine,
    Dict,
    Generator,
    List,
    MutableMapping,
    Optional,
    Set,
    Tuple,
    TypeVar,
    Union,
    overload,
)
from uuid import UUID

import janus
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.service import BleakGATTService
from bleak.exc import BleakDBusError, BleakDeviceNotFoundError, BleakError
from configfile import ConfigWrapper
from extras.led import LEDHelper
from gcode import GCodeCommand, GCodeDispatch
from klippy import Printer
from reactor import SelectReactor
from typing_extensions import Buffer, override

if typing.TYPE_CHECKING:
    _LoggerAdapter = logging.LoggerAdapter[logging.Logger]
else:
    _LoggerAdapter = logging.LoggerAdapter

__all__ = [
    "load_config",
]

# Non-configurable constants
CONTROLLER_ADVERTISEMENT_PERIOD = 0.5  # seconds, upper bound on adverts
CONTROLLER_CONNECTION_DELAY = 5  # seconds, expected upper bound based on tests

# Derived constants
# must cover at least 2-3 advert periods
BT_SCAN_KNOWN_ADDRESS_TIMEOUT = CONTROLLER_ADVERTISEMENT_PERIOD * 2
# If we're just trying to connect to any Nevermore, make extra sure there's only one.
BT_SCAN_GATHER_ALL_TIMEOUT = BT_SCAN_KNOWN_ADDRESS_TIMEOUT * 4


_A = TypeVar("_A")
_Float = TypeVar("_Float", bound=float)

RGBW = Tuple[float, float, float, float]

# BLE Constants (inclusive)
TIMESEC16_MAX = 2**16 - 2
VOC_INDEX_MAX = 500


# Not actually provided by `bleak`. IDK why not.
class CharacteristicProperty(enum.Enum):
    BROADCAST = "broadcast"
    INDICATE = "indicate"
    NOTIFY = "notify"
    READ = "read"
    WRITE = "write"
    WRITE_NO_RESPONSE = "write-without-response"


class LogPrefixed(_LoggerAdapter):
    def __init__(
        self,
        logger: Union[logging.Logger, _LoggerAdapter],
        format: Callable[[str], str],
    ):
        super().__init__(logger, None)  # type: ignore
        self.format = format

    def process(self, msg: Any, kwargs: MutableMapping[str, Any]):
        return self.format(msg), kwargs


LOG = LogPrefixed(
    logging.getLogger(__name__),
    lambda str: f"[{datetime.datetime.now().strftime('%H:%M:%S:%f')}] {str}",
)


# (measure-time, temperature) -> None
SensorCallback = Callable[[float, float], None]


def _apply2_optional(fn: Callable[[_A, _A], _A]):
    def go(x: Optional[_A], y: Optional[_A]):
        if x is None:
            return y
        if y is None:
            return x
        return fn(x, y)

    return go


class SensorKind(Enum):
    INTAKE = 1
    EXHAUST = 2


@dataclass(frozen=True)
class SensorState:
    temperature: Optional[float] = None  # Celsius
    humidity: Optional[float] = None  # %
    pressure: Optional[float] = None  # hPa
    # HACK: Must be named `gas` b/c Fluidd only looks at `temperature`, `humidity`, `pressure`, and `gas`.
    #       Also happens to be what the bme680 reports.
    gas: Optional[int] = None  # [1, VOC_INDEX_MAX]

    def as_dict(self) -> Dict[str, float]:
        return {
            f.name: getattr(self, f.name)
            for f in dataclasses.fields(self)
            if getattr(self, f.name) is not None
        }

    def merge(
        self,
        rhs: "SensorState",
        # TFW you need rank-2 polymorphism but python typing doesn't support it
        fn: Callable[[Optional[float], Optional[float]], Optional[float]],
    ) -> "SensorState":
        x = fn(self.gas, rhs.gas)
        return SensorState(
            temperature=fn(self.temperature, rhs.temperature),
            humidity=fn(self.humidity, rhs.humidity),
            pressure=fn(self.pressure, rhs.pressure),
            gas=None if x is None else int(x),  # rank-2 poly support pls :(
        )


@dataclass
class ControllerState:
    intake: SensorState = SensorState()
    exhaust: SensorState = SensorState()
    fan_power: float = 0
    fan_tacho: float = 0

    def min(self, rhs: "ControllerState") -> "ControllerState":
        return self.merge(rhs, _apply2_optional(min))  # type: ignore needs better bounds

    def max(self, rhs: "ControllerState") -> "ControllerState":
        return self.merge(rhs, _apply2_optional(max))  # type: ignore needs better bounds

    def merge(
        self,
        rhs: "ControllerState",
        fn: Callable[[Optional[float], Optional[float]], Optional[float]],
    ) -> "ControllerState":
        return ControllerState(
            intake=self.intake.merge(rhs.intake, fn),
            exhaust=self.exhaust.merge(rhs.exhaust, fn),
            fan_power=fn(self.fan_power, rhs.fan_power) or 0,
            fan_tacho=fn(self.fan_tacho, rhs.fan_tacho) or 0,
        )


def short_uuid(x: int):
    assert 0 <= x <= 0xFFFF
    return UUID(f"0000{x:04x}-0000-1000-8000-00805f9b34fb")


UUID_SERVICE_GAP = short_uuid(0x1801)
UUID_SERVICE_ENVIRONMENTAL_SENSING = short_uuid(0x181A)
UUID_SERVICE_CONFIGURATION = UUID("b5078b20-aea3-4c37-a18f-b370c03f02a6")
UUID_SERVICE_FAN = UUID("4553d138-1d00-4b6f-bc42-955a89cf8c36")
UUID_SERVICE_WS2812 = UUID("f62918ab-33b7-4f47-9fba-8ce9de9fecbb")
UUID_SERVICE_FAN_POLICY = UUID("260a0845-e62f-48c6-aef9-04f62ff8bffd")

UUID_CHAR_PERCENT8 = short_uuid(0x2B04)
UUID_CHAR_COUNT16 = short_uuid(0x2AEA)
UUID_CHAR_TIMESEC16 = short_uuid(0x2B16)
UUID_CHAR_DATA_AGGREGATE = UUID("75134bec-dd06-49b1-bac2-c15e05fd7199")
UUID_CHAR_FAN_TACHO = UUID("03f61fe0-9fe7-4516-98e6-056de551687f")
UUID_CHAR_FAN_AGGREGATE = UUID("79cd747f-91af-49a6-95b2-5b597c683129")
UUID_CHAR_FAN_THERMAL = UUID("45d2e7d7-40c4-46a6-a160-43eb02d01e27")
UUID_CHAR_VOC_INDEX = UUID("216aa791-97d0-46ac-8752-60bbc00611e1")
UUID_CHAR_WS2812_UPDATE = UUID("5d91b6ce-7db1-4e06-b8cb-d75e7dd49aae")
UUID_CHAR_CONFIG_FLAGS64 = UUID("d4b66bf4-3d8f-4746-b6a2-8a59d2eac3ce")


def _clamp(x: _Float, min: _Float, max: _Float) -> _Float:
    if x < min:
        return min
    if max < x:
        return max
    return x


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


class BleAttrReaderNotEnoughData(Exception):
    pass


class BleAttrReader:
    def __init__(self, raw: bytes):
        self.remaining = raw

    def humidity(self):
        return self._unsigned(2, 1, -2, 0, not_known=0xFFFF)

    def percentage8(self):
        return self._unsigned(1, 1, 0, -1, not_known=0xFF)

    def pressure(self):
        return self._unsigned(4, 1, -1, 0, not_known=0xFFFFFFFF)

    def temperature(self):
        return self._signed(2, 1, -2, 0, not_known=0x8000)

    def tachometer(self):
        return int(self._unsigned(2, 1, 0, 0))

    def voc_index(self) -> Optional[int]:
        return self._as_int(self._unsigned(2, 1, 0, 0, not_known=0))

    @overload
    def _signed(self, sz: int, M: int, d: int, e: int) -> float:
        ...

    @overload
    def _signed(
        self, sz: int, M: int, d: int, e: int, *, not_known: int
    ) -> Optional[float]:
        ...

    def _signed(
        self, sz: int, M: int, d: int, e: int, *, not_known: Optional[int] = None
    ) -> Optional[float]:
        return self._consume(True, sz, M, d, e, not_known)

    @overload
    def _unsigned(self, sz: int, M: int, d: int, e: int) -> float:
        ...

    @overload
    def _unsigned(
        self, sz: int, M: int, d: int, e: int, *, not_known: int
    ) -> Optional[float]:
        ...

    def _unsigned(
        self, sz: int, M: int, d: int, e: int, *, not_known: Optional[int] = None
    ) -> Optional[float]:
        return self._consume(False, sz, M, d, e, not_known)

    def _consume(
        self,
        signed: bool,
        sz: int,
        M: int,
        d: int,
        e: int,
        not_known: Optional[int],
    ) -> Optional[float]:
        if len(self.remaining) < sz:
            raise BleAttrReaderNotEnoughData("insufficient data remaining")

        head = self.remaining[0:sz]
        self.remaining = self.remaining[sz:]

        # the constant is given as a hex literal (i.e. unsigned)
        if not_known == int.from_bytes(head, "little", signed=False):
            return None

        return self._raw_to_repr(int.from_bytes(head, "little", signed=signed), M, d, e)

    def _raw_to_repr(self, x: float, M: int, d: int, e: int) -> float:
        return x * M * (10**d) * (2**e)

    def _as_int(self, x: Optional[float]):
        return None if x is None else int(x)


class BleAttrWriter:
    def __init__(self, raw: bytes = b""):
        self.value = raw

    def temperature(self, t: Optional[float]):
        return self._presentation_format(t, 2, 2, 0x8000, signed=True)

    def percentage16_10(self, t: Optional[float]):
        return self._presentation_format(t, 2, 2, 0xFFFF, signed=False)

    def _presentation_format(
        self, t: Optional[float], size: int, digits: int, not_known: int, signed: bool
    ):
        if t is None:
            # 0xFFFF -> temperature special value: not-known
            self.value += not_known.to_bytes(size, "little")
        else:
            self.value += int(t * 10**digits).to_bytes(size, "little", signed=signed)

        return self


def parse_agg_env(reader: BleAttrReader) -> Tuple[SensorState, SensorState]:
    t_in = reader.temperature()
    t_out = reader.temperature()
    _t_mcu = reader.temperature()  # unused/ignored
    h_in = reader.humidity()
    h_out = reader.humidity()
    p_in = reader.pressure()
    p_out = reader.pressure()
    voc_in = reader.voc_index()
    voc_out = reader.voc_index()

    if p_in is not None:
        p_in /= 100  # need it in hPa instead of Pa

    if p_out is not None:
        p_out /= 100  # need it in hPa instead of Pa

    return (
        SensorState(t_in, h_in, p_in, voc_in),
        SensorState(t_out, h_out, p_out, voc_out),
    )


def require_chars(
    service: BleakGATTService,
    id: UUID,
    num: Optional[int] = None,
    props: Set[CharacteristicProperty] = {CharacteristicProperty.READ},
):
    xs = [
        x
        for x in service.characteristics
        if x.uuid == str(id)
        if all(prop.value in x.properties for prop in props)
    ]
    # chars aren't necessarily ordered. order them by handle # (as they appear in the database)
    xs = sorted(xs, key=lambda x: x.handle)

    if num is not None and len(xs) != num:
        raise Exception(
            f"{service} doesn't have exactly {num} characteristic(s) {id} with properties {props}"
        )

    return xs


def require_char(
    service: BleakGATTService,
    id: UUID,
    props: Set[CharacteristicProperty] = {CharacteristicProperty.READ},
):
    xs = require_chars(service, id, None, props)
    if len(xs) == 1:
        return xs[0]

    raise Exception(
        f"{service} has {'no' if not xs else 'multiple'} characteristic {id} with properties {props}"
    )


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
    ]  # type: ignore mistake in overload resolution


@dataclass
class LedUpdateSpan:
    begin: int
    end: int  # INVARIANT(begin < end)
    num_dirty: int = 1

    def worth_extending_to(self, i: int):
        # Theoretically we should be able to do 512 (ATT maximum).
        # Can't send more than 253 octets at once (tested).
        # FUTURE WORK: Investigate. For now it's more than enough for our use case.
        TX_MAX = 253
        HEADER_SZ = 2  # 2 octets: 1 offset, 1 length
        MAX_LENGTH = TX_MAX - HEADER_SZ
        MIN_LENGTH = 20 - HEADER_SZ  # picked ad-hoc, amortise comm overhead
        MIN_DENSITY = 0.5  # picked ad-hoc

        if i < self.end:
            return False  # already contains or can't extend `begin`

        length = (i + 1) - self.begin
        if MAX_LENGTH <= length:
            return False  # too long
        if length <= MIN_LENGTH:
            return True  # even if the density sucks, the comm overhead sucks more

        density = (self.num_dirty + 1) / length
        return MIN_DENSITY <= density

    def extend_to(self, i: int):
        if not self.worth_extending_to(i):
            return False

        self.end = i + 1
        self.num_dirty += 1
        return True

    @staticmethod
    def compute_diffs(old_data: bytearray, new_data: bytearray):
        assert len(old_data) == len(new_data)
        span: Optional[LedUpdateSpan] = None

        def cmd(span: LedUpdateSpan):
            return (span.begin, new_data[span.begin : span.end])

        for i, (old, new) in enumerate(zip(old_data, new_data)):
            if old == new or (span is not None and span.extend_to(i)):
                continue

            if span is not None:
                yield cmd(span)

            span = LedUpdateSpan(i, i + 1)

        if span is not None:
            yield cmd(span)


# Commands which aren't directly forwarded to the controller.
class PseudoCommand:
    pass


# Commands which are directly forwarded to the controller.
class Command:
    @abstractmethod
    def params(self) -> Buffer:
        raise NotImplemented


class CmdFanPowerAbstract(Command):
    percent: Optional[float]

    def params(self):
        if self.percent is None:
            return bytearray([0xFF])  # 0xFF -> percent8 special value: not-known

        p = _clamp(self.percent, 0, 1) * 100
        return int(p * 2).to_bytes(1, "little")


@dataclass(frozen=True)
class CmdFanPowerOverride(CmdFanPowerAbstract):
    percent: Optional[float]


@dataclass(frozen=True)
class CmdFanPowerAuto(CmdFanPowerAbstract):
    percent: float


@dataclass(frozen=True)
class CmdFanPowerCoeff(CmdFanPowerAbstract):
    percent: float


@dataclass(frozen=True)
class CmdFanPolicyCooldown(Command):
    value: int  # seconds

    def params(self):
        return _clamp(self.value, 0, TIMESEC16_MAX).to_bytes(2, "little")


@dataclass(frozen=True)
class CmdFanPolicyVocPassiveMax(Command):
    value: int

    def params(self):
        return _clamp(self.value, 0, VOC_INDEX_MAX).to_bytes(2, "little")


@dataclass(frozen=True)
class CmdFanPolicyVocImproveMin(Command):
    value: int

    def params(self):
        return _clamp(self.value, 0, VOC_INDEX_MAX).to_bytes(2, "little")


@dataclass(frozen=True)
class CmdFanPolicyThermalLimit(Command):
    temp_min: float
    temp_max: float
    percent: Optional[float]

    def params(self):
        return (
            BleAttrWriter()
            .temperature(self.temp_min)
            .temperature(self.temp_max)
            .percentage16_10(None if self.percent is None else self.percent * 100)
            .value
        )


@dataclass(frozen=True)
class CmdWs2812Length(Command):
    n_total_components: int

    def params(self):
        return int(self.n_total_components).to_bytes(2, "little")


@dataclass(frozen=True)
class CmdConfigFlags(Command):
    flags: int

    def params(self):
        return self.flags.to_bytes(8, "little")


class CmdFanPolicy(PseudoCommand):
    def __init__(self, config: ConfigWrapper) -> None:
        def cfg_int(key: str, min: int, max: int) -> Optional[int]:
            return config.getint(f"fan_policy_{key}", None, minval=min, maxval=max)

        self.cooldown = cfg_int("cooldown", 0, TIMESEC16_MAX)
        self.voc_passive_max = cfg_int("voc_passive_max", 0, VOC_INDEX_MAX)
        self.voc_improve_min = cfg_int("voc_improve_min", 0, VOC_INDEX_MAX)


class CmdConfiguration(PseudoCommand):
    def __init__(self, config: ConfigWrapper) -> None:
        self.flags = 0

        def cfg_flag(key: str, default: bool, flag_idx: int) -> Optional[int]:
            if config.getboolean(key, default):
                self.flags |= 1 << flag_idx

        cfg_flag("sensors_fallback", True, 0)
        cfg_flag("sensors_fallback_exhaust_mcu", False, 1)


# Special pseudo command: Due to the very high frequency of these commands, we don't
# queue them, but rather mark the pixel buffer as dirty to fold consecutive writes.
# We still technically need this command because a single pixel buffer update
# may require several updates to transfer, depending on dirty size.
# NB: With no-response-write we're technically fast enough for 24 FPS updates,
#     but let's keep this mechanism anyways.
class CmdWs2812MarkDirty(PseudoCommand):
    pass


# An `asyncio.Event` that avoid spamming the `ready` queue of a loop.
# Naively using `call_soon_threadsafe` creates an entry per call even if we're already set.
# This is almost certainly subtly broken, but works for our limited use case.
# Abuses the GIL for concurrent field access control (`_set_in_ready_queue`).
class UNSAFE_LazyAsyncioEvent(asyncio.Event):
    def __init__(self):
        super().__init__()
        self._set_in_ready_queue = False

    @override
    def clear(self):
        super().clear()
        self._set_in_ready_queue = False

    def set_threadsafe(self, loop: asyncio.AbstractEventLoop):
        if self._set_in_ready_queue:
            return
        self._set_in_ready_queue = True

        loop.call_soon_threadsafe(lambda: self.set())


# HACK: This class *heavily* abuses the GIL for data synchronisation between
# the klipper thread and the background worker thread.
class NevermoreBackgroundWorker:
    def __init__(self, nevermore: "Nevermore") -> None:
        # A weak ref allows us to end the worker if the nevermore instance is
        # ever GC'd without asking us to disconnect (for whatever reason).
        self._nevermore = weakref.ref(nevermore, lambda nevermore: self.disconnect())
        self._connected = threading.Event()
        self._led_colour_data_old = bytearray()
        self._loop = asyncio.new_event_loop()
        self._thread = Thread(target=self._worker)
        # HACK: Can't set this up yet b/c the loop isn't set.
        # It'll be available at some point before `self._connected` is set.
        self._command_queue: janus.Queue[Command] = None
        # HACK: Python < 3.10 compatibility.
        #       *ALSO* can't set this up for the same god damn reason, specifically
        #       `asyncio.Event` had a `loop` param that was removed in 3.10
        #       but everyone before that had an implicit get-current-loop in the ctor.
        self._loop_exit: UNSAFE_LazyAsyncioEvent = None
        self._led_dirty: UNSAFE_LazyAsyncioEvent = None

        self._thread.start()

    def wait_for_connection(self, timeout: Optional[float] = None) -> bool:
        # if the thread isn't running then we're already broken, don't bother waiting
        return self._thread.is_alive() and self._connected.wait(timeout)

    def disconnect(self):
        assert self._loop_exit is not None, "pre-condition violated"
        self._loop_exit.set_threadsafe(self._loop)

    # PRECONDITION: `self._connected` is set
    def send_command(self, cmd: Optional[Union[Command, PseudoCommand]]):
        assert self._command_queue is not None, "cannot send commands before connecting"
        if cmd is None:  # simplify client control flow
            return

        def send_maybe(wrapper: Callable[[_A], Command], x: Optional[_A]):
            if x is not None:
                self._command_queue.sync_q.put(wrapper(x))

        if isinstance(cmd, Command):
            self._command_queue.sync_q.put(cmd)
        elif isinstance(cmd, CmdFanPolicy):
            send_maybe(CmdFanPolicyCooldown, cmd.cooldown)
            send_maybe(CmdFanPolicyVocPassiveMax, cmd.voc_passive_max)
            send_maybe(CmdFanPolicyVocImproveMin, cmd.voc_improve_min)
        elif isinstance(cmd, CmdWs2812MarkDirty):
            self._led_dirty.set_threadsafe(self._loop)
        elif isinstance(cmd, CmdConfiguration):
            send_maybe(CmdConfigFlags, cmd.flags)
        else:
            raise Exception(f"unhandled pseudo-command {cmd}")

    def _worker(self) -> None:
        nevermore = self._nevermore()
        if nevermore is None:
            return  # Already dead and we didn't need to do anything...

        self._thread.name = nevermore.name
        device_address = nevermore.bt_address
        nevermore = None  # release reference otherwise call frame keeps it alive

        worker_log = LogPrefixed(LOG, lambda x: f"{self._thread.name} - {x}")

        async def handle_connection(device_address: Optional[str]) -> None:
            # Attempt (re)connection. Might have to do this multiple times if we lose connection.
            #
            # HACK: FIXME: Very rarely (race?) it happens that the cancel exception fires, but that
            # the exception isn't properly propagated? I'm not sure what the hell is going on, but
            # the log showed:
            # ```
            # Timeout with MCU 'mcu' (eventtime=252271.768243)
            # Transition to shutdown state: Lost communication with MCU 'mcu'
            # ...
            # _GatheringFuture exception was never retrieved
            # future: <_GatheringFuture finished exception=CancelledError()>
            # cmd = await self._command_queue.async_q.get()
            # ...
            # [09:37:00:001317] nevermore - discovered controller 28:CD:C1:09:64:8F
            # [09:37:01:811708] nevermore - connected to controller 28:CD:C1:09:64:8F
            # ```
            # So a cancel was raised, `_worker_using` exited, but the `main`
            # task never was cancelled?
            #
            # As a hack/workaround, check that the exit isn't set.
            # This should never be true, since the canceller task should cancel
            # us when it fires, but apparently I've a bug in here.
            while not self._loop_exit.is_set():
                try:
                    scan_timeout = (
                        BT_SCAN_GATHER_ALL_TIMEOUT
                        if device_address is None
                        else BT_SCAN_KNOWN_ADDRESS_TIMEOUT
                    )
                    devices = await discover_controllers(device_address, scan_timeout)
                    if not devices:
                        continue  # no devices found, try again

                    if 1 < len(devices):  # multiple devices found
                        worker_log.error(
                            f"multiple nevermore controllers discovered.\n"
                            f"specify which to use by setting `bt_address: <insert-address-here>` in your klipper config.\n"
                            f"discovered controllers (ordered by signal strength):\n"
                            f"\taddress           | signal strength\n"
                            f"\t-----------------------------------\n"
                            f"\t"
                            + "\n\t".join(
                                f"{x.address} | {x.rssi} dBm"
                                for x in sorted(devices, key=lambda x: -x.rssi)
                            )
                        )
                        return  # don't bother trying again. abort & let the user deal with it

                    # remember the discovered device's address, just in case another
                    # controller wanders into range and we need to reconnect to this one
                    if device_address is None:
                        device_address = devices[0].address

                    worker_log.info(f"discovered controller {devices[0].address}")
                    async with BleakClient(devices[0]) as client:
                        await self._worker_using(worker_log, client)
                except TimeoutError:
                    pass  # ignore, keep trying to (re)connect
                except BleakDeviceNotFoundError:
                    pass  # ignore, keep trying to (re)connect
                except BleakDBusError as e:
                    # potentially caused by noisy environments or poor timing
                    transient = (e.dbus_error == "org.bluez.Error.NotConnected") or (
                        e.dbus_error == "org.bluez.Error.Failed"
                        and e.dbus_error_details == "Software caused connection abort"
                    )
                    if not transient:
                        raise
                except BleakError as e:
                    if e.args[0] == "Not connected":
                        # don't be noisy about it, it happens
                        worker_log.info("connection lost. attempting reconnection...")
                    else:
                        # be noisy about it, something unexpected happened.
                        # try to recovery by resetting the connection.
                        # This sucks, but there's huge variety of errors that
                        # can happen due to a lost connection, and I can't think
                        # of a good way to recognise them with this API.
                        # TODO: Log this in the console area. Ostensibly you can
                        #       do this with `GCode::response_info`, but I don't
                        #       know if there are rules/invariants about this.
                        #       (e.g. only the active GCode/command may write)
                        worker_log.exception("BT error - attempting reconnection...")

        async def go():
            # set this up ASAP once we're in an asyncio loop
            self._command_queue = janus.Queue()
            # HACK: Python < 3.10 compatibility - See comment in `__init__`.
            self._loop_exit = UNSAFE_LazyAsyncioEvent()
            self._led_dirty = UNSAFE_LazyAsyncioEvent()

            main = asyncio.create_task(handle_connection(device_address))

            async def canceller():
                await self._loop_exit.wait()
                main.cancel()

            await asyncio.gather(main, canceller())

        try:
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(go())
        except asyncio.CancelledError:
            worker_log.info("disconnecting")
        except:
            worker_log.exception("worker failed")

    async def _worker_using(
        self, log: Union[logging.Logger, _LoggerAdapter], client: BleakClient
    ):
        log.info(f"connected to controller {client.address}")

        await client.get_services()  # fetch and cache services

        def require(id: UUID):
            x = client.services.get_service(id)
            if x is None:
                raise Exception(f"{client.address} doesn't have required service {id}")
            return x

        service_config = require(UUID_SERVICE_CONFIGURATION)
        service_env = require(UUID_SERVICE_ENVIRONMENTAL_SENSING)
        service_fan = require(UUID_SERVICE_FAN)
        service_fan_policy = require(UUID_SERVICE_FAN_POLICY)
        service_ws2812 = require(UUID_SERVICE_WS2812)

        P = CharacteristicProperty
        aggregate_env = require_char(service_env, UUID_CHAR_DATA_AGGREGATE, {P.NOTIFY})
        aggregate_fan = require_char(service_fan, UUID_CHAR_FAN_AGGREGATE, {P.NOTIFY})
        # HACK: it's the first one in the list (ordered by handle #). this is brittle.
        fan_power_override, fan_power_auto, fan_power_coeff = require_chars(
            service_fan, UUID_CHAR_PERCENT8, 3, {P.WRITE}
        )
        ws2812_length = require_char(service_ws2812, UUID_CHAR_COUNT16, {P.WRITE})
        ws2812_update = require_char(
            service_ws2812, UUID_CHAR_WS2812_UPDATE, {P.WRITE_NO_RESPONSE}
        )
        fan_policy_cooldown = require_char(
            service_fan_policy, UUID_CHAR_TIMESEC16, {P.WRITE}
        )
        fan_policy_voc_passive_max, fan_policy_voc_improve_min = require_chars(
            service_fan_policy, UUID_CHAR_VOC_INDEX, 2, {P.WRITE}
        )
        fan_thermal_limit = require_char(
            service_fan_policy, UUID_CHAR_FAN_THERMAL, {P.WRITE}
        )
        config_flags = require_char(service_config, UUID_CHAR_CONFIG_FLAGS64, {P.WRITE})

        self._connected.set()

        # clear WS2812 diff cache, other end is in an undefined state
        self._led_colour_data_old = bytearray()

        nevermore = self._nevermore()
        if nevermore is None:
            return

        # inform frontend it should send setup/init commands
        nevermore.handle_controller_connect()
        nevermore = None  # release local ref

        async def notify(
            char: BleakGATTCharacteristic,
            callback: Callable[["Nevermore", BleAttrReader], Any],
        ):
            def go(_: BleakGATTCharacteristic, params: bytearray):
                nevermore = self._nevermore()
                if nevermore is not None:  # if frontend is dead -> nothing to do
                    callback(nevermore, BleAttrReader(params))

            await client.start_notify(char, go)

        def notify_env(nevermore: "Nevermore", params: BleAttrReader):
            (intake, exhaust) = parse_agg_env(params)
            # HACK: Abuse GIL to keep this thread-safe
            nevermore.state.intake = intake
            nevermore.state.exhaust = exhaust
            nevermore.state_stats_update()

        def notify_fan(nevermore: "Nevermore", params: BleAttrReader):
            # HACK: Abuse GIL to keep this thread-safe
            # show the current fan power even if it isn't overridden
            nevermore.state.fan_power = (params.percentage8() or 0) / 100.0
            nevermore.state.fan_tacho = params.tachometer()
            nevermore.state_stats_update()

        async def handle_commands():
            cmd = await self._command_queue.async_q.get()
            if isinstance(cmd, CmdFanPowerOverride):
                char = fan_power_override
            elif isinstance(cmd, CmdFanPowerAuto):
                char = fan_power_auto
            elif isinstance(cmd, CmdFanPowerCoeff):
                char = fan_power_coeff
            elif isinstance(cmd, CmdFanPolicyCooldown):
                char = fan_policy_cooldown
            elif isinstance(cmd, CmdFanPolicyVocPassiveMax):
                char = fan_policy_voc_passive_max
            elif isinstance(cmd, CmdFanPolicyVocImproveMin):
                char = fan_policy_voc_improve_min
            elif isinstance(cmd, CmdFanPolicyThermalLimit):
                char = fan_thermal_limit
            elif isinstance(cmd, CmdWs2812Length):
                char = ws2812_length
            elif isinstance(cmd, CmdConfigFlags):
                char = config_flags
            else:
                raise Exception(f"unhandled command {cmd}")

            try:
                await client.write_gatt_char(char, cmd.params())
            except BleakError as e:
                # consider non-fatal. don't to abort due to potentially transient error
                # special case: lost connection -> wait and attempt reconnect
                if e.args[0] == "Not connected":
                    raise

                log.exception(f"command failed cmd={cmd}")

        async def handle_led():
            await self._led_dirty.wait()
            self._led_dirty.clear()

            for offset, data in self._worker_led_diffs():
                params = bytearray([offset, len(data)]) + data
                await client.write_gatt_char(ws2812_update, params)

        async def forever(go: Callable[[], Coroutine[Any, Any, Any]]):
            while True:
                await go()

        tasks = asyncio.gather(*[forever(x) for x in [handle_commands, handle_led]])

        try:
            await notify(aggregate_env, notify_env)
            await notify(aggregate_fan, notify_fan)
            await tasks
        except EOFError:  # consider non-fatal, potentially transient
            return
        finally:
            tasks.cancel()  # kill off all active tasks if any fail

    def _worker_led_diffs(self) -> Generator[Tuple[int, bytearray], Any, None]:
        nm = self._nevermore()
        if nm is None:
            return  # frontend is dead -> nothing to do

        if self._led_colour_data_old == nm.led_colour_data:
            return  # fast-path bail

        # resize to accommodate. change of length -> just mark everything as dirty
        if len(self._led_colour_data_old) != len(nm.led_colour_data):
            self._led_colour_data_old = bytearray([x ^ 1 for x in nm.led_colour_data])

        yield from LedUpdateSpan.compute_diffs(
            self._led_colour_data_old, nm.led_colour_data
        )

        self._led_colour_data_old[:] = nm.led_colour_data


class Nevermore:
    def __init__(self, config: ConfigWrapper) -> None:
        self.name = config.get_name().split()[-1]
        self.printer: Printer = config.get_printer()
        self.reactor: SelectReactor = self.printer.get_reactor()
        self.state = ControllerState()
        self._state_min = ControllerState()
        self._state_max = ControllerState()
        self.fan = NevermoreFan(self)

        self.bt_address: Optional[str] = config.get("bt_address", None)
        if self.bt_address is not None:
            self.bt_address = self.bt_address.upper()
            if not _bt_address_validate(self.bt_address):
                raise config.error(
                    f"invalid bluetooth address for `bt_address`, given `{self.bt_address}`"
                )

        bt_scan_timeout = (
            BT_SCAN_GATHER_ALL_TIMEOUT
            if self.bt_address is None
            else BT_SCAN_KNOWN_ADDRESS_TIMEOUT
        )
        # never gonna reliable find the controller if we don't scan long enough
        connection_initial_min = CONTROLLER_CONNECTION_DELAY + bt_scan_timeout
        self.connection_initial_timeout: float = config.getfloat(
            "connection_initial_timeout",
            CONTROLLER_CONNECTION_DELAY + bt_scan_timeout * 2,
            minval=0,
        )
        if (
            self.connection_initial_timeout != 0
            and self.connection_initial_timeout < connection_initial_min
        ):
            raise config.error(
                f"`connection_initial_timeout` must either be 0 or >= {connection_initial_min}."
            )
        if self.connection_initial_timeout == 0 and self.bt_address is None:
            raise config.error(
                f"`connection_initial_timeout` cannot be 0 if `bt_address` is not specified."
            )

        # LED-specific code.
        # Ripped from `extras/neopixel.py` because the processing code is entangled with MCU transmission.
        # Modified to remove minor absurdities & fit our use case.
        led_chain_count = config.getint("led_chain_count", 0, minval=0)
        led_colour_order = config.getlist("led_colour_order", ["GRB"])
        for x in led_colour_order:
            if sorted(x) not in (sorted("RGB"), sorted("RGBW")):
                raise config.error(f"invalid `led_colour_order` '{x}'")
        if len(led_colour_order) == 1:
            led_colour_order = [led_colour_order[0]] * led_chain_count
        if len(led_colour_order) != led_chain_count:
            raise config.error("`led_colour_order` does not match `led_chain_count`")

        self.led_colour_idxs = [
            (i, "RGBW".index(c))
            for i, colour_order in enumerate(led_colour_order)
            for c in colour_order
        ]
        self.led_colour_data = bytearray(len(self.led_colour_idxs))
        self.led_helper: LEDHelper = self.printer.load_object(
            config, "led"
        ).setup_helper(config, self._led_update, led_chain_count)

        def cfg_fan_power(mk: Callable[[float], CmdFanPowerAbstract], name: str):
            x = config.getfloat(name, default=None, minval=0, maxval=1)
            return mk(x) if x is not None else None

        self._configuration = CmdConfiguration(config)
        self._fan_policy = CmdFanPolicy(config)
        self._fan_power_auto = cfg_fan_power(CmdFanPowerAuto, "fan_power_automatic")
        self._fan_power_coeff = cfg_fan_power(CmdFanPowerCoeff, "fan_power_coefficient")

        fan_thermal_min: Optional[float] = config.getfloat(
            "fan_thermal_limit_temperature_min", default=None
        )
        fan_thermal_max: Optional[float] = config.getfloat(
            "fan_thermal_limit_temperature_max", default=None
        )
        fan_thermal_coeff: Optional[float] = config.getfloat(
            "fan_thermal_limit_coefficient", default=None, maxval=1
        )
        if (
            fan_thermal_min is None
            and fan_thermal_max is None
            and fan_thermal_coeff is None
        ):
            self._fan_thermal_limit = None  # use the controller's defaults
        else:
            # TODO: unspecified values should be queried from the controller
            #       (better user-visible behaviour if defaults change)
            fan_thermal_min = 50 if fan_thermal_min is None else fan_thermal_min
            fan_thermal_max = 60 if fan_thermal_max is None else fan_thermal_max
            fan_thermal_coeff = 0 if fan_thermal_coeff is None else fan_thermal_coeff
            if fan_thermal_coeff == -1:
                fan_thermal_coeff = None
            elif fan_thermal_coeff < 0:
                raise config.error(
                    "`fan_thermal_limit_coefficient` must be in range [0, 1] or -1 (disables)"
                )

            if fan_thermal_max < fan_thermal_min:
                raise config.error(
                    "`fan_thermal_limit_temperature_min` must <= `fan_thermal_limit_temperature_max`"
                )

            self._fan_thermal_limit = CmdFanPolicyThermalLimit(
                fan_thermal_min, fan_thermal_max, fan_thermal_coeff
            )
            LOG.info(self._fan_thermal_limit)
            LOG.info(self._fan_thermal_limit.params())

        self._interface: Optional[NevermoreBackgroundWorker] = None
        self._handle_request_restart(None)

        self.printer.add_object(f"fan_generic {self.fan.name}", self.fan)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )

    def set_fan_power(self, percent: Optional[float]):
        if self._interface is not None:
            self._interface.send_command(CmdFanPowerOverride(percent))

    def state_stats_update(self):
        self._state_min = self._state_min.min(self.state)
        self._state_max = self._state_max.max(self.state)

    def _led_update(self, led_state: List[RGBW], print_time: Optional[float]) -> None:
        for i, (led, clr) in enumerate(self.led_colour_idxs):
            self.led_colour_data[i] = int(led_state[led][clr] * 255.0 + 0.5)

        if self._interface is None:
            LOG.info(f"_led_update failed: not connected")
            return

        # Background interface will handle diffing and update as appropriate
        self._interface.send_command(CmdWs2812MarkDirty())

    def _handle_connect(self) -> None:
        if self._interface is None:
            raise Exception("precondition - `self._interface` must not be None")

        self._led_update(self.led_helper.get_status()["color_data"], None)

        if (
            self.connection_initial_timeout != 0
            and not self._interface.wait_for_connection(self.connection_initial_timeout)
        ):
            self._interface.disconnect()  # the deadline was blown, it doesn't need to keep trying.
            raise self.printer.config_error("nevermore failed to connect - timed out")

    def handle_controller_connect(self) -> None:
        if self._interface is None:
            return  # defensive: potential race between shutdown notice and connection

        self._interface.send_command(self._configuration)
        self._interface.send_command(self._fan_policy)
        self._interface.send_command(self._fan_power_auto)
        self._interface.send_command(self._fan_power_coeff)
        self._interface.send_command(self._fan_thermal_limit)
        self._interface.send_command(CmdWs2812Length(len(self.led_colour_idxs)))
        self._interface.send_command(CmdWs2812MarkDirty())

    def _handle_request_restart(self, print_time: Optional[float]):
        self._handle_shutdown()
        self._interface = NevermoreBackgroundWorker(self)
        # TODO: reset fan & LED to defaults?

    def _handle_shutdown(self):
        if self._interface is not None:
            self._interface.disconnect()
            self._interface = None

    # having this method is sufficient to be visible to klipper/moonraker
    def get_status(self, eventtime: float) -> Dict[str, float]:
        data: Dict[str, float] = {}

        def add(fmt: str, k: str, v: Optional[float]):
            if v is not None:
                data[fmt.format(k)] = v

        def add_data(fmt: str, s: ControllerState):
            add(fmt, "speed", s.fan_power)
            add(fmt, "rpm", s.fan_tacho)
            for k, v in s.intake.as_dict().items():
                add(f"intake_{fmt}", k, v)
            for k, v in s.exhaust.as_dict().items():
                add(f"exhaust_{fmt}", k, v)

        add_data("{0}", self.state)
        add_data("{0}_min", self._state_min)
        add_data("{0}_max", self._state_max)
        return data


# basically ripped from `extras/fan_generic.py`
class NevermoreFan:
    cmd_SET_FAN_SPEED_help = (
        "Sets the speed of a nevermore fan (omit `SPEED` for automatic control)"
    )

    def __init__(self, nevermore: Nevermore) -> None:
        self.nevermore = nevermore
        self.printer = nevermore.printer
        self.name = f"{nevermore.name}_fan"

        gcode: GCodeDispatch = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_FAN_SPEED",
            "FAN",
            self.name,
            self.cmd_SET_FAN_SPEED,
            desc=self.cmd_SET_FAN_SPEED_help,
        )

    def get_status(self, eventtime: float) -> Dict[str, float]:
        return {
            "speed": self.nevermore.state.fan_power,
            "rpm": self.nevermore.state.fan_tacho,
        }

    def cmd_SET_FAN_SPEED(self, gcmd: GCodeCommand) -> None:
        # `None` to allow explicitly clearing override by not specifying a `SPEED` arg
        speed: Optional[float] = gcmd.get_float("SPEED", None)
        self.nevermore.set_fan_power(speed)


class NevermoreSensor:
    def __init__(self, config: ConfigWrapper) -> None:
        self.name = config.get_name().split()[-1]
        self.printer: Printer = config.get_printer()
        self.plot_voc = config.getboolean("plot_voc", False)
        try:
            self.sensor_kind = SensorKind[config.get("sensor_kind", self.name).upper()]
        except KeyError:
            raise config.error(
                "`sensor_kind` isn't `intake` or `exhaust`, nor is the sensor name"
            )

        class_name = config.get("class_name_override", "NevermoreSensor").strip()
        if len(class_name) == 0:
            raise config.error("`class_name_override` cannot be an empty string")

        self.min_temp = self.max_temp = 0.0
        self.nevermore: Optional[Nevermore] = None
        self._callback: Optional[SensorCallback] = None
        self._timer_sample = self.printer.get_reactor().register_timer(self._sample)

        self.printer.add_object(f"{class_name} {self.name}", self)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    @property
    def state(self) -> SensorState:
        if self.nevermore is None:
            return SensorState()

        if self.sensor_kind == SensorKind.INTAKE:
            return self.nevermore.state.intake
        else:
            return self.nevermore.state.exhaust

    def get_status(self, eventtime: float) -> Dict[str, float]:
        # HACK: can only plot on mainsail/fluidd if we pretend the VOC Index is a temperature
        if self.plot_voc:
            return {"temperature": self.state.gas or 0}

        return self.state.as_dict()

    # required by sensors API
    def setup_minmax(self, min_temp: float, max_temp: float) -> None:
        self.min_temp = min_temp
        self.max_temp = max_temp

    # required by sensors API
    def setup_callback(self, cb: Callable[[float, float], None]) -> None:
        self._callback = cb

    # required by sensors API
    def get_report_time_delta(self) -> float:
        return 1  # TODO: fetch from controller, no point refreshing faster than them

    def _handle_connect(self) -> None:
        self.nevermore = self.printer.lookup_object("nevermore")
        assert isinstance(self.nevermore, Nevermore)
        reactor: SelectReactor = self.printer.get_reactor()
        reactor.update_timer(self._timer_sample, reactor.NOW)

    def _sample(self, eventtime: float) -> None:
        measured_time = self.printer.get_reactor().monotonic()

        # HACK: can only plot on mainsail/fluidd if we pretend the VOC Index is a temperature
        temp = self.state.temperature
        if self.plot_voc:
            temp = self.state.gas

        if temp is not None and self._callback is not None:
            self._callback(measured_time, temp)

        return measured_time + 1  # 1s delay?


def load_config(config: ConfigWrapper):
    heaters = config.get_printer().load_object(config, "heaters")
    heaters.add_sensor_factory("NevermoreSensor", NevermoreSensor)

    return Nevermore(config)
