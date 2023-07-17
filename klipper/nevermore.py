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
from bleak import BleakClient, BleakError, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.service import BleakGATTCharacteristic, BleakGATTService
from bleak.exc import BleakDeviceNotFoundError
from configfile import ConfigWrapper
from extras.led import LEDHelper
from gcode import GCodeCommand, GCodeDispatch
from klippy import Printer
from reactor import SelectReactor
from typing_extensions import override

__all__ = [
    "load_config",
]

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


class LogPrefixed(logging.LoggerAdapter):
    def __init__(
        self,
        logger: Union[logging.Logger, logging.LoggerAdapter],
        format: Callable[[str], str],
    ):
        super().__init__(logger, None)
        self.format = format

    def process(self, msg: Any, kwargs: MutableMapping[str, Any]):
        return self.format(msg), kwargs


LOG = LogPrefixed(
    logging.getLogger(__name__),
    lambda str: f"[{datetime.datetime.now().strftime('%H:%M:%S:%f')}] {str}",
)


# How long to wait for a BLE connection to the controller (sec)
TIMEOUT_CONNECT = 10.0


# (measure-time, temperature) -> None
SensorCallback = Callable[[float, float], None]


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


@dataclass
class ControllerState:
    intake: SensorState = SensorState()
    exhaust: SensorState = SensorState()
    fan_power: float = 0
    fan_tacho: float = 0


def short_uuid(x: int):
    assert 0 <= x <= 0xFFFF
    return UUID(f"0000{x:04x}-0000-1000-8000-00805f9b34fb")


UUID_SERVICE_GAP = short_uuid(0x1801)
UUID_SERVICE_ENVIRONMENTAL_SENSING = short_uuid(0x181A)
UUID_SERVICE_FAN = UUID("4553d138-1d00-4b6f-bc42-955a89cf8c36")
UUID_SERVICE_WS2812 = UUID("f62918ab-33b7-4f47-9fba-8ce9de9fecbb")
UUID_SERVICE_FAN_POLICY = UUID("260a0845-e62f-48c6-aef9-04f62ff8bffd")

UUID_CHAR_PERCENT8 = short_uuid(0x2B04)
UUID_CHAR_COUNT16 = short_uuid(0x2AEA)
UUID_CHAR_TIMESEC16 = short_uuid(0x2B16)
UUID_CHAR_DATA_AGGREGATE = UUID("75134bec-dd06-49b1-bac2-c15e05fd7199")
UUID_CHAR_FAN_TACHO = UUID("03f61fe0-9fe7-4516-98e6-056de551687f")
UUID_CHAR_VOC_INDEX = UUID("216aa791-97d0-46ac-8752-60bbc00611e1")
UUID_CHAR_WS2812_UPDATE = UUID("5d91b6ce-7db1-4e06-b8cb-d75e7dd49aae")


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


def parse_agg_env(reader: BleAttrReader) -> Tuple[SensorState, SensorState]:
    t_in = reader.temperature()
    t_out = reader.temperature()
    t_mcu = reader.temperature()  # unused/ignored
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
    address: Optional[str] = None, timeout: float = 10
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
    def params(self) -> bytearray:
        raise NotImplemented


@dataclass(frozen=True)
class CmdFanPower(Command):
    percent: Optional[float]

    def params(self):
        if self.percent is None:
            return bytearray([0xFF])  # 0xFF -> percent8 special value: not-known

        p = _clamp(self.percent, 0, 1) * 100
        return int(p * 2).to_bytes(1, "little")


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
class CmdWs2812Length(Command):
    n_total_components: int

    def params(self):
        return int(self.n_total_components).to_bytes(2, "little")


class CmdFanPolicy(PseudoCommand):
    def __init__(self, config: ConfigWrapper) -> None:
        def cfg_int(key: str, min: int, max: int) -> Optional[int]:
            return config.getint(f"fan_policy_{key}", None, minval=min, maxval=max)

        self.cooldown = cfg_int("cooldown", 0, TIMESEC16_MAX)
        self.voc_passive_max = cfg_int("voc_passive_max", 0, VOC_INDEX_MAX)
        self.voc_improve_min = cfg_int("voc_improve_min", 0, VOC_INDEX_MAX)


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
        self._loop_exit.set_threadsafe(self._loop)

    # PRECONDITION: `self._connected` is set
    def send_command(self, cmd: Union[Command, PseudoCommand]):
        assert self._command_queue is not None, "cannot send commands before connecting"

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
            while True:
                try:
                    # TIMEOUT_CONNECT * 0.75 b/c scan must finish before main thread
                    # times out waiting for initial connection check
                    devices = await discover_controllers(
                        device_address, timeout=TIMEOUT_CONNECT * 0.75
                    )
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
                        worker_log.info(f"discovered controller {devices[0].address}")
                        device_address = devices[0].address

                    async with BleakClient(devices[0]) as client:
                        await self._worker_using(worker_log, client)
                except TimeoutError:
                    pass  # ignore, keep trying to (re)connect
                except BleakDeviceNotFoundError:
                    pass  # ignore, keep trying to (re)connect

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
            raise
        except:
            worker_log.exception("worker failed")

    async def _worker_using(
        self, log: Union[logging.Logger, logging.LoggerAdapter], client: BleakClient
    ):
        log.info(f"connected to controller {client.address}")

        await client.get_services()  # fetch and cache services

        def require(id: UUID):
            x = client.services.get_service(id)
            if x is None:
                raise Exception(f"{client.address} doesn't have required service {id}")
            return x

        service_env = require(UUID_SERVICE_ENVIRONMENTAL_SENSING)
        service_fan = require(UUID_SERVICE_FAN)
        service_fan_policy = require(UUID_SERVICE_FAN_POLICY)
        service_ws2812 = require(UUID_SERVICE_WS2812)

        P = CharacteristicProperty
        aggregate_env = require_char(service_env, UUID_CHAR_DATA_AGGREGATE, {P.NOTIFY})
        aggregate_fan = require_char(service_fan, UUID_CHAR_DATA_AGGREGATE, {P.NOTIFY})
        fan_power_override = require_char(service_fan, UUID_CHAR_PERCENT8, {P.WRITE})
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

        def notify_fan(nevermore: "Nevermore", params: BleAttrReader):
            # HACK: Abuse GIL to keep this thread-safe
            # show the current fan power even if it isn't overridden
            nevermore.state.fan_power = params.percentage8() / 100.0  # need it in [0,1]
            _ = params.percentage8()  # power-override
            nevermore.state.fan_tacho = params.tachometer()

        async def handle_commands():
            cmd = await self._command_queue.async_q.get()
            if isinstance(cmd, CmdFanPower):
                char = fan_power_override
            elif isinstance(cmd, CmdFanPolicyCooldown):
                char = fan_policy_cooldown
            elif isinstance(cmd, CmdFanPolicyVocPassiveMax):
                char = fan_policy_voc_passive_max
            elif isinstance(cmd, CmdFanPolicyVocImproveMin):
                char = fan_policy_voc_improve_min
            elif isinstance(cmd, CmdWs2812Length):
                char = ws2812_length
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
        except BleakError as e:
            # consider non-fatal. don't to abort due to potentially transient error
            # special case: lost connection -> wait and attempt reconnect
            if e.args[0] == "Not connected":
                log.info("connection lost. attempting reconnection...")
                return

            raise  # any other kind of uncaught failure -> re-raise
        except EOFError:  # consider non-fatal, potentially transient
            return
        finally:
            tasks.cancel()  # kill off all active tasks if any fail

    def _worker_led_diffs(self):
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
        self.fan = NevermoreFan(self)

        self.bt_address: Optional[str] = config.get("bt_address", None)
        if self.bt_address is not None:
            self.bt_address = self.bt_address.upper()
            if not _bt_address_validate(self.bt_address):
                raise config.error(
                    f"invalid bluetooth address for `bt_address`, given `{self.bt_address}`"
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

        self._fan_policy = CmdFanPolicy(config)
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
            self._interface.send_command(CmdFanPower(percent))

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

        if not self._interface.wait_for_connection(TIMEOUT_CONNECT):
            raise self.printer.config_error("nevermore failed to connect - timed out")

        self._led_update(self.led_helper.get_status()["color_data"], None)

    def handle_controller_connect(self) -> None:
        self._interface.send_command(self._fan_policy)
        self._interface.send_command(CmdWs2812Length(len(self.led_colour_idxs)))

    def _handle_request_restart(self, print_time: Optional[float]):
        self._handle_shutdown()
        self._interface = NevermoreBackgroundWorker(self)
        # TODO: reset fan & LED to defaults?

    def _handle_shutdown(self):
        if self._interface is not None:
            self._interface.disconnect()
            self._interface = None


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

        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def get_status(self, eventtime: float) -> Dict[str, float]:
        return {
            "speed": 0 if self.nevermore is None else self.nevermore.state.fan_power,
            "rpm": 0 if self.nevermore is None else self.nevermore.state.fan_tacho,
        }

    def _handle_connect(self) -> None:
        self.nevermore = self.printer.lookup_object("nevermore")
        assert isinstance(self.nevermore, Nevermore)

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
