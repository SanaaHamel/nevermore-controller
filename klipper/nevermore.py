# Nevermore Controller Interface
#
# Copyright (C) 2024       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

import asyncio
import datetime
import importlib
import importlib.util
import logging
import os
import os.path
import re
import sys
import threading
import weakref
from abc import ABCMeta, abstractmethod
from dataclasses import dataclass
from enum import Enum
from threading import Thread
from typing import (
    Any,
    Callable,
    Coroutine,
    Dict,
    Generator,
    Iterable,
    List,
    Optional,
    Set,
    Tuple,
    Type,
    TypeVar,
    Union,
)
from uuid import UUID

import bleak
import janus
import serial
from bleak import BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
from configfile import ConfigWrapper
from extras.led import LEDHelper
from extras.heaters import Heater
from gcode import GCodeCommand, GCodeDispatch
from klippy import Printer
from reactor import SelectReactor
from typing_extensions import override


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

NEVERMORE_REFRESH_DELAY = 1  # seconds, only relevant for serial connections

# Non-configurable constants
CONTROLLER_NOTIFY_TIMEOUT = 30  # seconds, reconnect if no updates found

# retry quickly during initial setup to minimise likelihood of failing timeout
CONTROLLER_CONNECTION_RETRY_DELAY_INITIAL = 1
# be really conservative about this because we don't want to spam the logs
CONTROLLER_CONNECTION_RETRY_DELAY_POST_CONNECT = 60

# we've very little time because we're executing on the main thread and cannot
# block w/o screwing over other processes
NEVERMORE_SERIAL_TIMEOUT = 0.025

_A = TypeVar("_A")
_B = TypeVar("_B")

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


# Most commands are defined in `nevermore_utilities.py`, unless they're created using Klipper data
# TODO: It's inconsistent, I know.


class CmdServoRangeFromCfg(CmdServoRange, metaclass=ABCMeta):
    def __init__(self, prefix: str, config: ConfigWrapper) -> None:
        def cfg(key: str, default: float) -> Optional[float]:
            pulse_sec = config.getfloat(
                f"{prefix}_{key}",
                default,
                above=0,
                below=NEVERMORE_SERVO_PERIOD,
            )
            return pulse_sec / NEVERMORE_SERVO_PERIOD

        super().__init__(cfg("pulse_width_min", 0.001), cfg("pulse_width_max", 0.002))


@cmd_simple(lambda x: x.servo_vent_range)
class CmdServoVentRange(CmdServoRangeFromCfg):
    def __init__(self, config: ConfigWrapper) -> None:
        super().__init__("vent_servo", config)


class CmdFanPolicy(Command):
    def __init__(self, config: ConfigWrapper) -> None:
        def cfg_int(key: str, min: int, max: int) -> Optional[int]:
            return config.getint(f"fan_policy_{key}", None, minval=min, maxval=max)

        self.cooldown = cfg_int("cooldown", 0, TIMESEC16_MAX)
        self.voc_passive_max = cfg_int("voc_passive_max", 0, VOC_INDEX_MAX)
        self.voc_improve_min = cfg_int("voc_improve_min", 0, VOC_INDEX_MAX)

    @override
    async def dispatch(self, comm: CommBindings):
        async def send_u16(char: Transport.Attribute, val: Optional[int]):
            if val is not None:
                await char.write(val.to_bytes(2, "little"))

        await send_u16(comm.fan_policy_cooldown, self.cooldown)
        await send_u16(comm.fan_policy_voc_passive_max, self.voc_passive_max)
        await send_u16(comm.fan_policy_voc_improve_min, self.voc_improve_min)


class CmdConfiguration(Command):
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

    @override
    async def dispatch(self, comm: CommBindings):
        await comm.config_flags.write(
            self.flags.to_bytes(8, "little") + self.mask.to_bytes(8, "little")
        )


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


class NevermoreInterface:
    def __init__(self, name: str):
        self.name = name
        self.log = LogPrefixed(LOG, lambda x: f"{name} - {x}")
        self._led_colour_data_old = bytearray()

    @property
    @abstractmethod
    def nevermore(self) -> Optional['Nevermore']:
        raise NotImplemented

    @property
    @abstractmethod
    def connected(self) -> bool:
        raise NotImplemented

    @abstractmethod
    def launch(self) -> None:
        raise NotImplemented

    @abstractmethod
    def wait_for_connection(self) -> bool:
        raise NotImplemented

    @abstractmethod
    def disconnect(self) -> None:
        raise NotImplemented

    @abstractmethod
    def refresh(self) -> None:
        raise NotImplemented

    # PRECONDITION: `self.connected` is True
    @abstractmethod
    def send_command(self, cmd: Optional[Union[Command, PseudoCommand]]) -> None:
        raise NotImplemented

    async def leds_update(self, comm: CommBindings):
        for offset, data in self._leds_diff():
            params = bytes([offset, len(data)]) + data
            await comm.ws2812_update.write(params)

    def _leds_diff(self) -> Generator[Tuple[int, bytearray], Any, None]:
        nevermore = self.nevermore
        if nevermore is None:
            return  # frontend is dead -> nothing to do

        led_colour_data = nevermore.led_colour_data
        if self._led_colour_data_old == led_colour_data:
            return  # fast-path bail

        # resize to accommodate. change of length -> just mark everything as dirty
        if len(self._led_colour_data_old) != len(led_colour_data):
            self._led_colour_data_old = bytearray([x ^ 1 for x in led_colour_data])

        yield from LedUpdateSpan.compute_diffs(
            self._led_colour_data_old, led_colour_data
        )

        self._led_colour_data_old[:] = led_colour_data


class NevermoreSerial(NevermoreInterface):
    def __init__(self, nevermore: "Nevermore", serial: serial.Serial) -> None:
        super().__init__(f"nevermore {nevermore.name}")
        self._nevermore = nevermore
        self._serial = serial

        original_timeout = self._serial.timeout
        self._serial.timeout = 1  # relax during connect, we're not in a rush

        try:
            self._comms = CommBindings(TransportSerial(serial))
        except Exception as e:
            self.log.exception(e, exc_info=False)
            raise e
        finally:
            self._serial.timeout = original_timeout

    @property
    @override
    def nevermore(self):
        return self._nevermore

    @property
    @override
    def connected(self):
        return self._serial.is_open

    @override
    def launch(self):
        # inform frontend it should send setup/init commands
        self.nevermore.handle_controller_connect()

    @override
    def wait_for_connection(self) -> bool:
        # nothing to wait for it should immediately be ready
        return True

    @override
    def disconnect(self):
        self._serial.close()

    @override
    def refresh(self) -> None:
        if not self.connected:
            return

        async def go():
            fan = BleAttrReader(await self._comms.aggregate_fan.read())
            self.nevermore.state.fan_power = (fan.percentage8() or 0) / 100.0
            self.nevermore.state.fan_tacho = fan.tachometer()

            (intake, exhaust) = SensorState.parse(
                BleAttrReader(await self._comms.aggregate_env.read())
            )
            self.nevermore.state.intake = intake
            self.nevermore.state.exhaust = exhaust

            self.nevermore.state_stats_update()

        try:
            asyncio.run(go())
        except Exception as e:
            self.log.exception(e)

    @override
    def send_command(self, cmd: Optional[Union[Command, PseudoCommand]]):
        if cmd is None:
            return

        try:
            if isinstance(cmd, Command):
                # this is god damn hideous, but there's no way to make async-ness parametric to the type
                # in python.
                asyncio.run(cmd.dispatch(self._comms))
            elif isinstance(cmd, CmdWs2812MarkDirty):
                asyncio.run(self.leds_update(self._comms))
            else:
                raise Exception(f"unhandled pseudo-command {cmd}")
        except Exception as e:
            self.log.exception(e)


# HACK: This class *heavily* abuses the GIL for data synchronisation between
# the klipper thread and the background worker thread.
class NevermoreBLE(NevermoreInterface):
    # HACK: Either BlueZ or Bleak is unable to handle concurrent discovery
    #       requests. Serialise that section for now.
    # FUTURE WORK:  Have a single thread service all BLE work?
    scan_lock = threading.BoundedSemaphore()

    def __init__(
        self,
        nevermore: "Nevermore",
        device_address: Optional[str],
        connection_initial_timeout: float,
    ) -> None:
        super().__init__(f"nevermore-BLE '{nevermore.name}'")

        # A weak ref allows us to end the worker if the nevermore instance is
        # ever GC'd without asking us to disconnect (for whatever reason).
        self._nevermore = weakref.ref(nevermore, lambda nevermore: self.disconnect())
        self.device_address = device_address
        self.connection_initial_timeout = connection_initial_timeout

        self._connected = threading.Event()
        self._loop = asyncio.new_event_loop()
        # HACK: Can't set this up yet b/c the loop isn't set.
        # It'll be available at some point before `self._connected` is set.
        self._command_queue: janus.Queue[Command] = None
        # HACK: Python < 3.10 compatibility.
        #       *ALSO* can't set this up for the same god damn reason, specifically
        #       `asyncio.Event` had a `loop` param that was removed in 3.10
        #       but everyone before that had an implicit get-current-loop in the ctor.
        self._disconnect: UNSAFE_LazyAsyncioEvent = None
        self._led_dirty: UNSAFE_LazyAsyncioEvent = None
        self._thread = Thread(target=self._worker)
        self._thread.name = self.name

    @property
    @override
    def nevermore(self):
        return self._nevermore()

    @property
    @override
    def connected(self):
        return self._thread.is_alive() and self._connected.is_set()

    @override
    def launch(self):
        self._thread.start()

    @override
    def wait_for_connection(self) -> bool:
        # if the thread isn't running then we're already broken, don't bother waiting
        return self._thread.is_alive() and (
            self.connection_initial_timeout == 0
            or self._connected.wait(self.connection_initial_timeout)
        )

    @override
    def disconnect(self):
        assert self._disconnect is not None, "pre-condition violated"
        self._disconnect.set_threadsafe(self._loop)

    @override
    def refresh(self) -> None:
        pass  # no-op, we do everything via notify/internal monitoring

    @override
    def send_command(self, cmd: Optional[Union[Command, PseudoCommand]]):
        assert self._command_queue is not None, "cannot send commands before connecting"
        if cmd is None:
            return

        if isinstance(cmd, Command):
            self._command_queue.sync_q.put(cmd)
        elif isinstance(cmd, CmdWs2812MarkDirty):
            self._led_dirty.set_threadsafe(self._loop)
        else:
            raise Exception(f"unhandled pseudo-command {cmd}")

    def _worker(self) -> None:
        if self.nevermore is None:
            return  # Already dead and we didn't need to do anything...

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
                self.log.exception("attempting reconnection...", exc_info=e)
                return True

            # Raiser is responsible for any associated logging w/ this exception.
            if isinstance(e, NevermoreForceReconnection):
                return True

            return False

        async def retry():
            self._connected.clear()
            if self._disconnect.is_set():
                return False

            delta = (datetime.datetime.now() - connection_start).total_seconds()
            await asyncio.sleep(
                CONTROLLER_CONNECTION_RETRY_DELAY_INITIAL
                if delta <= self.connection_initial_timeout
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
                    try:
                        self.scan_lock.acquire()
                        devices = await discover_bluetooth_devices(
                            lambda x: device_is_likely_a_nevermore(x, log=LOG),
                            device_address,
                            scan_timeout,
                        )
                    finally:
                        self.scan_lock.release()

                    if not not devices:
                        break

                if 1 < len(devices):  # multiple devices found
                    self.log.error(
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

                self.log.info(f"discovered controller {devices[0].address}")
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
                    lambda client: self._worker_using(client),
                    connection_timeout=None,
                    exc_filter=exc_filter,
                    log=self.log,
                    retry=retry,
                )
            except CantInferWhichNevermoreToUse:
                pass  # quietly fail and move on
            finally:
                self._connected.clear()

        async def go():
            # set this up ASAP once we're in an asyncio loop
            self._command_queue = janus.Queue()
            # HACK: Python < 3.10 compatibility - See comment in `__init__`.
            self._disconnect = UNSAFE_LazyAsyncioEvent()
            self._led_dirty = UNSAFE_LazyAsyncioEvent()

            main = asyncio.create_task(handle_connection(self.device_address))

            async def canceller():
                await self._disconnect.wait()
                main.cancel()

            await asyncio.gather(main, canceller())

        try:
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(go())
        except asyncio.CancelledError:
            self.log.info("disconnecting")
        except:
            self.log.exception("worker failed")

    async def _worker_using(self, client: BleakClient):
        self.log.info(f"connected to controller {client.address}")

        await client.get_services()  # ensure services have been pulled
        comm = CommBindings(TransportBLE(client))
        self._connected.set()

        # clear WS2812 diff cache, other end is in an undefined state
        self._led_colour_data_old = bytearray()

        nevermore = self.nevermore
        if nevermore is None:
            return

        # inform frontend it should send setup/init commands
        nevermore.handle_controller_connect()
        nevermore = None  # release local ref

        async def handle_commands():
            cmd = await self._command_queue.async_q.get()

            try:
                await cmd.dispatch(comm)
            except bleak.exc.BleakError as e:
                # special case: lost connection -> wait and attempt reconnect
                if is_lost_connection_exception(e):
                    raise

                # consider non-fatal. if we're in the wrong (API error), then
                # we can do nothing but log it and limp on...
                self.log.exception(f"command failed cmd={cmd}")

        async def handle_led():
            await self._led_dirty.wait()
            self._led_dirty.clear()
            await self.leds_update(comm)

        async def handle_poll():
            nevermore = self.nevermore
            if nevermore is not None:
                params = BleAttrReader(bytes(await comm.aggregate_env.read()))
                (intake, exhaust) = SensorState.parse(params)
                # HACK: Abuse GIL to keep this thread-safe
                nevermore.state.intake = intake
                nevermore.state.exhaust = exhaust

                params = BleAttrReader(bytes(await comm.aggregate_fan.read()))
                # show the current fan power even if it isn't overridden
                nevermore.state.fan_power = (params.percentage8() or 0) / 100.0
                nevermore.state.fan_tacho = params.tachometer()
                nevermore.state_stats_update()

            await asyncio.sleep(NEVERMORE_REFRESH_DELAY)

        async def forever(go: Callable[[], Coroutine[Any, Any, Any]]):
            while True:
                await go()

        tasks = asyncio.gather(
            *[forever(x) for x in [handle_commands, handle_led, handle_poll]]
        )

        try:
            await tasks
        finally:
            tasks.cancel()  # kill off all active tasks if any fail


@dataclass(frozen=True)
class NevermoreSerialInfo:
    path: str
    baudrate: int = NEVERMORE_SERIAL_BAUDRATE_DEFAULT

    @staticmethod
    def mk(config: ConfigWrapper) -> Optional['NevermoreSerialInfo']:
        path: Optional[str] = config.get("serial", None)
        if path is None:
            return None

        return NevermoreSerialInfo(
            path=path,
        )


class NevermoreBleInfo:
    def __init__(self, config: ConfigWrapper):
        self.address: Optional[str] = config.get("bt_address", None)
        if self.address is not None:
            self.address = self.address.upper()
            if not bt_address_validate(self.address):
                raise config.error(
                    f"invalid bluetooth address for `address`, given `{self.address}`"
                )

        scan_timeout = (
            BT_SCAN_GATHER_ALL_TIMEOUT
            if self.address is None
            else BT_SCAN_KNOWN_ADDRESS_TIMEOUT
        )
        # never gonna reliable find the controller if we don't scan long enough
        connection_initial_min = CONTROLLER_CONNECTION_DELAY + scan_timeout
        self.connection_initial_timeout: float = config.getfloat(
            "connection_initial_timeout",
            CONTROLLER_CONNECTION_DELAY + scan_timeout * 2,
            minval=0,
        )
        if (
            self.connection_initial_timeout != 0
            and self.connection_initial_timeout < connection_initial_min
        ):
            raise config.error(
                f"`connection_initial_timeout` must either be 0 or >= {connection_initial_min}."
            )
        if self.connection_initial_timeout == 0 and self.address is None:
            raise config.error(
                f"`connection_initial_timeout` cannot be 0 if `bt_address` is not specified."
            )


class Nevermore:
    cmd_NEVERMORE_PRINT_START_help = "`NEVERMORE_PRINT_START` is deprecated and no longer has any effect. Remove it from macros."
    cmd_NEVERMORE_PRINT_END_help = "`NEVERMORE_PRINT_END` is deprecated and no longer has any effect. Remove it from macros."
    cmd_NEVERMORE_VENT_SERVO_SET_help = (
        "Set the PWM for the vent servo. Omit `PWM` to disable the servo."
    )
    cmd_NEVERMORE_VOC_GATING_THRESHOLD_OVERRIDE_help = "Set/clear the VOC gating threshold override (omit `THRESHOLD` to clear override)"
    cmd_NEVERMORE_VOC_CALIBRATION_help = "Set the automatic VOC calibration process"
    cmd_NEVERMORE_REBOOT_help = "Reboots the controller"
    cmd_NEVERMORE_STATUS_help = "Report controller status to console"
    cmd_NEVERMORE_SENSOR_CALIBRATION_CHECKPOINT_help = (
        "Force sensors to checkpoint their calibration"
    )
    cmd_NEVERMORE_SENSOR_CALIBRATION_RESET_help = "Reset sensor calibration"
    cmd_NEVERMORE_RESET_help = "Reset settings. Do not use unless directed."

    @classmethod
    def gcode_command_names(cls):
        CMD_HELP_PATTERN = re.compile(r"^cmd_(.+)_help$")
        return {
            (m.group(1), getattr(Nevermore, f"cmd_{m.group(1)}_help"))
            for m in map(CMD_HELP_PATTERN.match, Nevermore.__dict__)
            if m is not None
        }

    def __init__(self, config: ConfigWrapper) -> None:
        self.name = config.get_name().split()[-1]
        self.printer: Printer = config.get_printer()
        self.reactor: SelectReactor = self.printer.get_reactor()
        self.state = ControllerState()
        self._state_min = ControllerState()
        self._state_max = ControllerState()
        self.fan = NevermoreFan(self)

        self.serial = NevermoreSerialInfo.mk(config)
        if self.serial is None:
            self.ble = NevermoreBleInfo(config)

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
        self.led_helper = LEDHelper(config, self._led_update, led_chain_count)

        def opt(mk: Callable[[_A], _B], x: Optional[_A]):
            return mk(x) if x is not None else None

        def cfg_fan_power(mk: Callable[[float], CommandSimplePercent], name: str):
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
        self._vent_servo_range = CmdServoVentRange(config)
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

        self._display_brightness = opt(
            CmdDisplayBrightness,
            config.getfloat("display_brightness", default=None, minval=0, maxval=1),
        )

        try:
            self._display_ui = opt(
                lambda x: CmdDisplayUI(DisplayUI[x.upper()].value),
                config.get("display_ui", default=None),
            )
        except KeyError:
            raise config.error(
                f"`display_ui` isn't one of: {', '.join(x.name for x in DisplayUI)}"
            )

        self._interface: Optional[NevermoreInterface] = None
        self._handle_request_restart(None)

        self.printer.add_object(f"fan_generic {self.fan.name}", self.fan)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )

        self._timer_refresh = self.printer.get_reactor().register_timer(
            self._handle_refresh, self.printer.get_reactor().NOW
        )

        gcode: GCodeDispatch = self.printer.lookup_object("gcode")
        for cmd, desc in Nevermore.gcode_command_names():
            gcode.register_mux_command(
                cmd, "NEVERMORE", self.name, getattr(self, f"cmd_{cmd}"), desc=desc
            )

    def set_fan_power(self, percent: Optional[float]):
        if self._interface is not None:
            self._interface.send_command(CmdFanPowerOverride(percent))

    def set_vent_servo_pwm(self, percent: Optional[float]):
        if percent is not None:
            percent = max(min(percent, 1), 0)

        if self._interface is not None:
            self._interface.send_command(CmdServoVentPWM(percent))

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

        if not self._interface.wait_for_connection():
            self._interface.disconnect()  # the deadline was blown, it doesn't need to keep trying.
            raise self.printer.config_error(
                f"nevermore '{self.name}' failed to connect - timed out"
            )

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
        self._interface.send_command(self._display_brightness)
        self._interface.send_command(self._display_ui)
        self._interface.send_command(CmdWs2812Length(len(self.led_colour_idxs)))
        self._interface.send_command(CmdWs2812MarkDirty())
        self._interface.send_command(self._vent_servo_range)

        # ensure controller suspends calibration if we reconnection mid-print
        self.send_printing_state_commands(
            NevermoreGlobal.get_or_create(self.printer).printing
        )

    def send_printing_state_commands(self, printing: bool):
        self.set_fan_power(1 if printing else 0)
        self.cmd_NEVERMORE_VOC_CALIBRATION(not printing)

    def _handle_request_restart(self, print_time: Optional[float]):
        self._handle_shutdown()

        if self.serial is not None:
            self._interface = NevermoreSerial(
                self,
                serial.Serial(
                    self.serial.path,
                    self.serial.baudrate,
                    timeout=NEVERMORE_SERIAL_TIMEOUT,
                    write_timeout=NEVERMORE_SERIAL_TIMEOUT,
                    exclusive=True,
                ),
            )
        elif self.ble is not None:
            self._interface = NevermoreBLE(
                self, self.ble.address, self.ble.connection_initial_timeout
            )
        else:
            raise Exception(f"no transport configured for {self.name}")

        self._interface.launch()

        # TODO: reset fan & LED to defaults?

    def _handle_shutdown(self):
        self.send_printing_state_commands(False)  # release fan control & calibration

        if self._interface is not None:
            self._interface.disconnect()
            self._interface = None

    def _handle_refresh(self, eventtime: float) -> None:
        if self._interface is not None and self._interface.connected:
            self._interface.refresh()

        return eventtime + NEVERMORE_REFRESH_DELAY

    # having this method is sufficient to be visible to klipper/moonraker
    def get_status(self, eventtime: float) -> Dict[str, float]:
        data = self.state.as_dict()
        data.update((f"{k}_min", v) for k, v in self._state_min.as_dict().items())
        data.update((f"{k}_max", v) for k, v in self._state_max.as_dict().items())
        data['connected'] = self._interface is not None and self._interface.connected
        return data

    def cmd_NEVERMORE_PRINT_START(self, gcmd: GCodeCommand) -> None:
        gcmd.respond_info(self.cmd_NEVERMORE_PRINT_START_help)

    def cmd_NEVERMORE_PRINT_END(self, gcmd: GCodeCommand) -> None:
        gcmd.respond_info(self.cmd_NEVERMORE_PRINT_END_help)

    def cmd_NEVERMORE_VENT_SERVO_SET(self, gcmd: GCodeCommand) -> None:
        self.set_vent_servo_pwm(gcmd.get_float("PWM", None, 0, 1))

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

    def cmd_NEVERMORE_VOC_CALIBRATION(self, gcmd: Union[GCodeCommand, bool]) -> None:
        self._voc_calibrate_enabled = CmdConfigVocCalibrateEnabled(
            gcmd
            if isinstance(gcmd, bool)
            else gcmd.get_int("ENABLED", minval=0, maxval=1) == 1
        )
        if self._interface is not None:
            self._interface.send_command(self._voc_calibrate_enabled)

    def cmd_NEVERMORE_REBOOT(self, gcmd: GCodeCommand) -> None:
        if self._interface is not None and self._interface.connected:
            self._interface.send_command(CmdConfigReboot())
        else:
            gcmd.respond_info(f"{self.name} not yet connected")

    def cmd_NEVERMORE_RESET(self, gcmd: GCodeCommand) -> None:
        if self._interface is not None:
            self._interface.send_command(
                CmdConfigReset(gcmd.get_int("FLAGS", minval=1, maxval=0xFF))
            )

    def cmd_NEVERMORE_STATUS(self, gcmd: GCodeCommand) -> None:
        if self._interface is not None and self._interface.connected:
            gcmd.respond_info(f"'{self.name}' connected")
        else:
            gcmd.respond_info(f"'{self.name}' not connected")

    def cmd_NEVERMORE_SENSOR_CALIBRATION_CHECKPOINT(self, gcmd: GCodeCommand) -> None:
        if self._interface is not None:
            self._interface.send_command(CmdConfigCheckpointSensorCalibration())

    def cmd_NEVERMORE_SENSOR_CALIBRATION_RESET(self, gcmd: GCodeCommand) -> None:
        if self._interface is not None:
            self._interface.send_command(CmdConfigResetSensorCalibration())


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
    # Nevermore sensor state has fields that shouldn't be user-displayed in most
    # cases. This is the subset we're typically interested in.
    DISPLAY_FIELDS = {"temperature", "humidity", "pressure", "gas", "gas_raw"}

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
        self.nevermore_name: str = config.get("nevermore", "nevermore")
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

        d = {k: v for k, v in self.state.as_dict().items() if k in self.DISPLAY_FIELDS}
        if "pressure" in d:  # round it for nicer display
            d["pressure"] = round(d["pressure"])
        return d

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
        nevermore = self.printer.lookup_object(self.nevermore_name, None)
        if not isinstance(nevermore, Nevermore):
            raise self.printer.config_error(
                f"`nevermore: {self.nevermore_name}` doesn't refer to a Nevermore object instance"
            )

        self.nevermore = nevermore

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


class NevermoreGlobal:
    EXTRUDER_HEATER_REGEX = re.compile(r"extruder\d*")

    @staticmethod
    def get_or_create(printer: Printer) -> "NevermoreGlobal":
        obj = printer.lookup_object("NevermoreGlobal", default=None)
        if obj is not None:
            assert isinstance(obj, NevermoreGlobal)
            return obj

        obj = NevermoreGlobal(printer)
        printer.add_object(f"NevermoreGlobal", obj)
        return obj

    def __init__(self, printer: Printer) -> None:
        self.printer = printer
        self.printing: bool = False
        self._heaters: List[Heater] = []

        reactor = printer.get_reactor()
        gcode: GCodeDispatch = printer.lookup_object("gcode")

        for cmd, desc in Nevermore.gcode_command_names():
            # python is ugly and stupid and does dynamic capture of values.
            def go(gcmd: GCodeCommand, cmd: str = cmd):
                for _, nevermore in self.nevermores():
                    getattr(nevermore, f"cmd_{cmd}")(gcmd)

            gcode.register_mux_command(cmd, "NEVERMORE", None, go, desc=desc)

        printer.register_event_handler("klippy:ready", self._handle_ready)
        reactor.register_timer(self._check_heaters, reactor.NOW)

    def _handle_ready(self) -> None:
        heaters = self.printer.lookup_object('heaters')
        self._heaters = [
            heaters.lookup_heater(name)
            for name in heaters.get_all_heaters()
            if self.EXTRUDER_HEATER_REGEX.fullmatch(name)
        ]

    def _check_heaters(self, eventtime: float) -> float:
        printing = any(heater.target_temp for heater in self._heaters)
        if self.printing != printing:
            self.printing = printing

            for _, nevermore in self.nevermores():
                nevermore.send_printing_state_commands(printing)

        return eventtime + NEVERMORE_REFRESH_DELAY

    def nevermores(self) -> List[Tuple[str, Nevermore]]:
        return self.printer.lookup_objects("nevermore")


def load_config(config: ConfigWrapper):
    # `add_sensor_factory` & `get_or_create` are both idempotent
    NevermoreGlobal.get_or_create(config.get_printer())

    heaters = config.get_printer().load_object(config, "heaters")
    heaters.add_sensor_factory("NevermoreSensor", NevermoreSensor)

    return Nevermore(config)


# `load_config_prefix` must be defined to allow named objects, but in our case
# it does the same work as `load_config`.
def load_config_prefix(config: ConfigWrapper):
    return load_config(config)
