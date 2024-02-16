# Nevermore Controller Interface
#
# Copyright (C) 2023       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

import asyncio
import datetime
import importlib
import importlib.util
import logging
import os
import os.path
import sys
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
    Generator,
    List,
    Optional,
    Set,
    Tuple,
    TypeVar,
    Union,
)
from uuid import UUID

import bleak
import janus
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.service import BleakGATTService
from configfile import ConfigWrapper
from extras.led import LEDHelper
from gcode import GCodeCommand, GCodeDispatch
from klippy import Printer
from reactor import SelectReactor
from typing_extensions import Buffer, override


# Commit war-crimes to load `/tools/nevermore_utilities.py`.
def _import_nevermore_utility():
    self_path = os.path.realpath(__file__)
    target_path = os.path.join(
        os.path.dirname(self_path), "..", "tools", "nevermore_utilities.py"
    )

    spec = importlib.util.spec_from_file_location("nevermore_utilities", target_path)
    assert spec, f"no spec for nevermore path {target_path}"
    assert spec.loader, f"no loader for {spec}"
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_import_nevermore_utility()
from nevermore_utilities import *

__all__ = [
    "load_config",
]

# Non-configurable constants
CONTROLLER_NOTIFY_TIMEOUT = 30  # seconds, reconnect if no updates found

# retry quickly during initial setup to minimise likelihood of failing timeout
CONTROLLER_CONNECTION_RETRY_DELAY_INITIAL = 1
# be really conservative about this because we don't want to spam the logs
CONTROLLER_CONNECTION_RETRY_DELAY_POST_CONNECT = 60

_A = TypeVar("_A")
_B = TypeVar("_B")
_Float = TypeVar("_Float", bound=float)

RGBW = Tuple[float, float, float, float]

# BLE Constants (inclusive)

LOG = LogPrefixed(
    logging.getLogger(__name__),
    lambda str: f"[{datetime.datetime.now().strftime('%H:%M:%S:%f')}] {str}",
)


# (measure-time, temperature) -> None
SensorCallback = Callable[[float, float], None]


class SensorKind(Enum):
    INTAKE = 1
    EXHAUST = 2


def _clamp(x: _Float, min: _Float, max: _Float) -> _Float:
    if x < min:
        return min
    if max < x:
        return max
    return x


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
    if address is not None:
        device = await BleakScanner.find_device_by_address(address, timeout=timeout)
        return [device] if device is not None else []

    return [
        x
        for x in await BleakScanner.discover(timeout=timeout)
        if await device_is_likely_a_nevermore(x, log=LOG)
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
class CmdFanPowerPassive(CmdFanPowerAbstract):
    percent: float


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
    min: Optional[float]
    max: Optional[float]
    percent: Optional[float]

    def params(self):
        return (
            BleAttrWriter()
            .temperature(self.min)
            .temperature(self.max)
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
    mask: int

    def params(self):
        return self.flags.to_bytes(8, "little") + self.mask.to_bytes(8, "little")


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
        self.mask = 0

        def cfg_flag(key: str, flag_idx: int):
            opt = config.getboolean(key, None)
            if opt is not None:
                self.mask |= 1 << flag_idx
                self.flags |= 1 << flag_idx if opt else 0

        cfg_flag("sensors_fallback", 0)
        cfg_flag("sensors_fallback_exhaust_mcu", 1)


@dataclass(frozen=True)
class CmdConfigVocThreshold(Command):
    value: int

    def params(self):
        return _clamp(self.value, 0, VOC_INDEX_MAX).to_bytes(2, "little")


@dataclass(frozen=True)
class CmdConfigVocThresholdOverride(Command):
    value: int

    def params(self):
        return _clamp(self.value, 0, VOC_INDEX_MAX).to_bytes(2, "little")


@dataclass(frozen=True)
class CmdConfigVocCalibrateEnabled(Command):
    value: bool

    def params(self):
        return int(self.value).to_bytes(1, "little")


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


class NevermoreForceReconnection(Exception):
    "Exception used to trigger a reconnection."
    pass


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
        self._disconnect: UNSAFE_LazyAsyncioEvent = None
        self._led_dirty: UNSAFE_LazyAsyncioEvent = None

        self._thread.start()

    def wait_for_connection(self, timeout: Optional[float] = None) -> bool:
        # if the thread isn't running then we're already broken, don't bother waiting
        return self._thread.is_alive() and self._connected.wait(timeout)

    def disconnect(self):
        assert self._disconnect is not None, "pre-condition violated"
        self._disconnect.set_threadsafe(self._loop)

    # PRECONDITION: `self._connected` is set
    def send_command(self, cmd: Optional[Union[Command, PseudoCommand]]):
        assert self._command_queue is not None, "cannot send commands before connecting"
        if cmd is None:  # simplify client control flow
            return

        def send(x: Command):
            self._command_queue.sync_q.put(x)

        def send_maybe(wrapper: Callable[[_A], Command], x: Optional[_A]):
            if x is not None:
                send(wrapper(x))

        if isinstance(cmd, Command):
            send(cmd)
        elif isinstance(cmd, CmdFanPolicy):
            send_maybe(CmdFanPolicyCooldown, cmd.cooldown)
            send_maybe(CmdFanPolicyVocPassiveMax, cmd.voc_passive_max)
            send_maybe(CmdFanPolicyVocImproveMin, cmd.voc_improve_min)
        elif isinstance(cmd, CmdWs2812MarkDirty):
            self._led_dirty.set_threadsafe(self._loop)
        elif isinstance(cmd, CmdConfiguration):
            send(CmdConfigFlags(cmd.flags, cmd.mask))
        else:
            raise Exception(f"unhandled pseudo-command {cmd}")

    def _worker(self) -> None:
        nevermore = self._nevermore()
        if nevermore is None:
            return  # Already dead and we didn't need to do anything...

        self._thread.name = nevermore.name
        device_address = nevermore.bt_address
        connection_initial_timeout = nevermore.connection_initial_timeout
        nevermore = None  # release reference otherwise call frame keeps it alive

        worker_log = LogPrefixed(LOG, lambda x: f"{self._thread.name} - {x}")
        connection_start = datetime.datetime.now()

        def exc_filter(e: Exception):
            # Be noisy about this, something unexpected happened.
            # Try to recovery by resetting the connection.
            # This sucks, but there's huge variety of errors that
            # can happen due to a lost connection, and I can't think
            # of a good way to recognise them with this API.
            # TODO: Log this in the console area. Ostensibly you can
            #       do this with `GCode::response_info`, but I don't
            #       know if there are rules/invariants about this.
            #       (e.g. only the active GCode/command may write)
            if isinstance(e, bleak.exc.BleakError) or isinstance(e, EOFError):
                worker_log.exception("attempting reconnection...", exc_info=e)
                return True

            # Raiser is responsible for any associated logging w/ this exception.
            if isinstance(e, NevermoreForceReconnection):
                return True

            return False

        async def retry():
            if self._disconnect.is_set():
                return False

            delta = (datetime.datetime.now() - connection_start).total_seconds()
            await asyncio.sleep(
                CONTROLLER_CONNECTION_RETRY_DELAY_INITIAL
                if delta <= connection_initial_timeout
                else CONTROLLER_CONNECTION_RETRY_DELAY_POST_CONNECT
            )

            return True

        async def handle_connection(device_address: Optional[str]) -> None:
            class CantInferWhichNevermoreToUse(Exception):
                pass

            async def discover_device():
                nonlocal device_address
                scan_timeout = (
                    BT_SCAN_GATHER_ALL_TIMEOUT
                    if device_address is None
                    else BT_SCAN_KNOWN_ADDRESS_TIMEOUT
                )

                while True:  # keep scanning until we find some devices...
                    devices = await discover_bluetooth_devices(
                        lambda x: device_is_likely_a_nevermore(x, log=LOG),
                        device_address,
                        scan_timeout,
                    )
                    if not not devices:
                        break

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
                    # don't bother trying again. abort & let the user deal with it
                    raise CantInferWhichNevermoreToUse()

                # remember the discovered device's address, just in case another
                # controller wanders into range and we need to reconnect to this one
                if device_address is None:
                    device_address = devices[0].address

                worker_log.info(f"discovered controller {devices[0].address}")
                return devices[0]

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
            try:
                await retry_if_disconnected(
                    discover_device,
                    lambda client: self._worker_using(worker_log, client),
                    connection_timeout=None,
                    exc_filter=exc_filter,
                    log=worker_log,
                    retry=retry,
                )
            except CantInferWhichNevermoreToUse:
                pass  # quietly fail and move on

        async def go():
            # set this up ASAP once we're in an asyncio loop
            self._command_queue = janus.Queue()
            # HACK: Python < 3.10 compatibility - See comment in `__init__`.
            self._disconnect = UNSAFE_LazyAsyncioEvent()
            self._led_dirty = UNSAFE_LazyAsyncioEvent()

            main = asyncio.create_task(handle_connection(device_address))

            async def canceller():
                await self._disconnect.wait()
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
        self, log: Union[logging.Logger, LoggerAdapter], client: BleakClient
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
        (
            fan_power_override,
            fan_power_passive,
            fan_power_auto,
            fan_power_coeff,
        ) = require_chars(service_fan, UUID_CHAR_PERCENT8, 4, {P.WRITE})
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
        config_voc_threshold, config_voc_threshold_override = require_chars(
            service_config, UUID_CHAR_VOC_INDEX, 2, {P.WRITE}
        )
        config_voc_calibrate_enabled = require_char(
            service_config, UUID_CHAR_CONFIG_VOC_CALIBRATE_ENABLED, {P.WRITE}
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

        last_notify_timestamp = datetime.datetime.now()

        async def notify(
            char: BleakGATTCharacteristic,
            callback: Callable[["Nevermore", BleAttrReader], Any],
        ):
            def go(_: BleakGATTCharacteristic, params: bytearray):
                nonlocal last_notify_timestamp
                last_notify_timestamp = datetime.datetime.now()

                nevermore = self._nevermore()
                if nevermore is not None:  # if frontend is dead -> nothing to do
                    callback(nevermore, BleAttrReader(params))

            await client.start_notify(char, go)

        def notify_env(nevermore: "Nevermore", params: BleAttrReader):
            (intake, exhaust) = SensorState.parse(params)
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
            elif isinstance(cmd, CmdFanPowerPassive):
                char = fan_power_passive
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
            elif isinstance(cmd, CmdConfigVocThreshold):
                char = config_voc_threshold
            elif isinstance(cmd, CmdConfigVocThresholdOverride):
                char = config_voc_threshold_override
            elif isinstance(cmd, CmdConfigVocCalibrateEnabled):
                char = config_voc_calibrate_enabled
            else:
                raise Exception(f"unhandled command {cmd}")

            try:
                await client.write_gatt_char(char, cmd.params())
            except bleak.exc.BleakError as e:
                # special case: lost connection -> wait and attempt reconnect
                if is_lost_connection_exception(e):
                    raise

                # consider non-fatal. if we're in the wrong (API error), then
                # we can do nothing but log it and limp on...
                log.exception(f"command failed cmd={cmd}")

        async def handle_led():
            await self._led_dirty.wait()
            self._led_dirty.clear()

            for offset, data in self._worker_led_diffs():
                params = bytearray([offset, len(data)]) + data
                await client.write_gatt_char(ws2812_update, params)

        async def handle_notify_timeout():
            nonlocal last_notify_timestamp
            elapsed = (datetime.datetime.now() - last_notify_timestamp).total_seconds()
            if CONTROLLER_NOTIFY_TIMEOUT < elapsed:
                log.warning(f"last notify was {elapsed} seconds ago. reconnecting...")
                raise NevermoreForceReconnection("notify timeout", elapsed)

            await asyncio.sleep(CONTROLLER_NOTIFY_TIMEOUT)

        async def forever(go: Callable[[], Coroutine[Any, Any, Any]]):
            while True:
                await go()

        tasks = asyncio.gather(
            *[forever(x) for x in [handle_commands, handle_led, handle_notify_timeout]]
        )

        try:
            await notify(aggregate_env, notify_env)
            await notify(aggregate_fan, notify_fan)
            await tasks
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
    cmd_NEVERMORE_VOC_GATING_THRESHOLD_OVERRIDE_help = "Set/clear the VOC gating threshold override (omit `THRESHOLD` to clear override)"
    cmd_NEVERMORE_VOC_CALIBRATION_help = "Set the automatic VOC calibration process"

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
            if not bt_address_validate(self.bt_address):
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

        def opt(mk: Callable[[_A], _B], x: Optional[_A]):
            return mk(x) if x is not None else None

        def cfg_fan_power(mk: Callable[[float], CmdFanPowerAbstract], name: str):
            return opt(mk, config.getfloat(name, default=None, minval=0, maxval=1))

        self._voc_gating_threshold = opt(
            CmdConfigVocThreshold,
            config.getint(
                "voc_gating_threshold", None, VOC_GATING_THRESHOLD_MIN, VOC_INDEX_MAX
            ),
        )
        self._voc_gating_threshold_override: Optional[CmdConfigVocThresholdOverride] = (
            None
        )
        # always send this command b/c I don't want to deal with support for people
        # who end up in a strange state due to a klipper reset
        self._voc_calibrate_enabled = CmdConfigVocCalibrateEnabled(True)
        self._configuration = CmdConfiguration(config)
        self._fan_policy = CmdFanPolicy(config)
        self._fan_power_passive = cfg_fan_power(CmdFanPowerPassive, "fan_power_passive")
        self._fan_power_auto = cfg_fan_power(CmdFanPowerAuto, "fan_power_automatic")
        self._fan_power_coeff = cfg_fan_power(CmdFanPowerCoeff, "fan_power_coefficient")

        self._fan_thermal_limit = CmdFanPolicyThermalLimit(
            config.getfloat("fan_thermal_limit_temperature_min", default=None),
            config.getfloat("fan_thermal_limit_temperature_max", default=None),
            config.getfloat(
                "fan_thermal_limit_coefficient", default=None, minval=0, maxval=1
            ),
        )

        if self._fan_thermal_limit.min is None != self._fan_thermal_limit.max is None:
            raise config.error(
                "`fan_thermal_limit_temperature_{min, max}` must either both be specified or neither specified"
            )

        if (self._fan_thermal_limit.max or 0) < (self._fan_thermal_limit.min or 0):
            raise config.error(
                "`fan_thermal_limit_temperature_min` must <= `fan_thermal_limit_temperature_max`"
            )

        self._interface: Optional[NevermoreBackgroundWorker] = None
        self._handle_request_restart(None)

        self.printer.add_object(f"fan_generic {self.fan.name}", self.fan)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )

        gcode: GCodeDispatch = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "NEVERMORE_VOC_GATING_THRESHOLD_OVERRIDE",
            "NEVERMORE",
            self.name,
            self.cmd_NEVERMORE_VOC_GATING_THRESHOLD_OVERRIDE,
            desc=self.cmd_NEVERMORE_VOC_GATING_THRESHOLD_OVERRIDE_help,
        )
        gcode.register_mux_command(
            "NEVERMORE_VOC_CALIBRATION",
            "NEVERMORE",
            self.name,
            self.cmd_NEVERMORE_VOC_CALIBRATION,
            desc=self.cmd_NEVERMORE_VOC_CALIBRATION_help,
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
        self._interface.send_command(self._voc_gating_threshold)
        self._interface.send_command(self._voc_gating_threshold_override)
        self._interface.send_command(self._voc_calibrate_enabled)
        self._interface.send_command(self._fan_policy)
        self._interface.send_command(self._fan_power_passive)
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
        data = self.state.as_dict()
        data.update((f"{k}_min", v) for k, v in self._state_min.as_dict().items())
        data.update((f"{k}_max", v) for k, v in self._state_max.as_dict().items())
        return data

    def cmd_NEVERMORE_VOC_GATING_THRESHOLD_OVERRIDE(self, gcmd: GCodeCommand) -> None:
        # `None` to allow explicitly clearing override by not specifying a `SPEED` arg
        self._voc_gating_threshold_override = CmdConfigVocThresholdOverride(
            gcmd.get_int(
                "THRESHOLD",
                VOC_INDEX_NOT_KNOWN,
                VOC_GATING_THRESHOLD_MIN,
                VOC_INDEX_MAX,
            )
        )
        if self._interface is not None:
            self._interface.send_command(self._voc_gating_threshold_override)

    def cmd_NEVERMORE_VOC_CALIBRATION(self, gcmd: GCodeCommand) -> None:
        self._voc_calibrate_enabled = CmdConfigVocCalibrateEnabled(
            gcmd.get_int("ENABLED", minval=0, maxval=1) == 1
        )
        if self._interface is not None:
            self._interface.send_command(self._voc_calibrate_enabled)


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
