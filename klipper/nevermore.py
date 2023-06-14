# Nevermore Controller Interface
#
# Copyright (C) 2023       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

import asyncio
import dataclasses
import logging
import threading
import weakref
from dataclasses import dataclass
from enum import Enum
from threading import Thread
from typing import (
    Any,
    Callable,
    Coroutine,
    MutableMapping,
    Optional,
    TypeVar,
    overload,
)
from uuid import UUID

from bleak import BleakClient, BleakError
from bleak.backends.service import BleakGATTCharacteristic, BleakGATTService
from bleak.exc import BleakDeviceNotFoundError
from configfile import ConfigWrapper
from gcode import GCodeCommand, GCodeDispatch
from klippy import Printer
from reactor import SelectReactor

__all__ = [
    "load_config",
]

_Float = TypeVar("_Float", bound=float)

LOG = logging.getLogger(__name__)


# How long to wait for a BLE connection to the controller (sec)
TIMEOUT_CONNECT = 10.0
# Polling rate for controller state updates
NEVERMORE_POLL_HZ = 1


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
    voc_index: Optional[int] = None  # [1, 500]

    def as_dict(self) -> dict[str, float | int]:
        return {
            f.name: getattr(self, f.name)
            for f in dataclasses.fields(self)
            if getattr(self, f.name) is not None
        }


@dataclass(frozen=True)
class ControllerState:
    intake: SensorState = SensorState()
    exhaust: SensorState = SensorState()
    fan_power: float = 0
    fan_tacho: float = 0


class LogAdaptorPrefixed(logging.LoggerAdapter):
    def __init__(self, logger: logging.Logger, prefix: str):
        super().__init__(logger)
        self.prefix = prefix

    def process(self, msg: Any, kwargs: MutableMapping[str, Any]):
        return f"{self.prefix}{msg}", kwargs


UUID_SERVICE_GAP = UUID("00001801-0000-1000-8000-00805f9b34fb")
UUID_SERVICE_ENVIRONMENTAL_SENSING = UUID("0000181a-0000-1000-8000-00805f9b34fb")
UUID_SERVICE_FAN = UUID("4553d138-1d00-4b6f-bc42-955a89cf8c36")
UUID_SERVICE_WS2812 = UUID("f62918ab-33b7-4f47-9fba-8ce9de9fecbb")

UUID_CHAR_ENV_DATA_AGGREGATE = UUID("75134bec-dd06-49b1-bac2-c15e05fd7199")
UUID_CHAR_FAN_POWER = UUID("00002b04-0000-1000-8000-00805f9b34fb")
UUID_CHAR_FAN_TACHO = UUID("03f61fe0-9fe7-4516-98e6-056de551687f")
UUID_CHAR_WS2812_UPDATE = UUID("5d91b6ce-7db1-4e06-b8cb-d75e7dd49aae")


def _clamp(x: _Float, min: _Float, max: _Float) -> _Float:
    if x < min:
        return min
    if max < x:
        return max
    return x


# must be of the form `xx:xx:xx:xx:xx:xx`, where `x` is a hex digit (uppercase)
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
    ) -> float | Optional[float]:
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
    ) -> float | Optional[float]:
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


def _parse_env_agg(reader: BleAttrReader) -> tuple[SensorState, SensorState]:
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


class NevermoreBackgroundWorker:
    def __init__(self, nevermore: "Nevermore") -> None:
        # A weak ref allows us to end the worker if the nevermore instance is
        # ever GC'd without asking us to disconnect (for whatever reason).
        self._nevermore = weakref.ref(nevermore)
        self._address = nevermore.bt_address
        self._connected = threading.Event()
        self._loop_exit = asyncio.Event()  # not thread safe, unlike `threading.Event`
        self._loop = asyncio.new_event_loop()
        self._thread = Thread(target=self._worker)
        # `_set_fan_power` can refer to a dead conn if the worker died for whatever reason.
        # This isn't great, but it won't be considered fatal and must not interrupt a print.
        self._set_fan_power: Optional[
            Callable[[float], Coroutine[Any, Any, None]]
        ] = None

        self._thread.start()

    def wait_for_connection(self, timeout: Optional[float] = None) -> bool:
        # if the thread isn't running then we're already broken, don't bother waiting
        return self._thread.is_alive() and self._connected.wait(timeout)

    def disconnect(self):
        self._loop.call_soon_threadsafe(lambda: self._loop_exit.set())

    def set_fan_power(self, percent: float):
        if self._set_fan_power is not None:
            asyncio.run_coroutine_threadsafe(self._set_fan_power(percent), self._loop)

    def _worker(self) -> None:
        nevermore = self._nevermore()
        if nevermore is None:
            return  # Already dead and we didn't need to do anything...

        self._thread.name = nevermore.name
        nevermore = None  # release reference otherwise call frame keeps it alive

        worker_log = LogAdaptorPrefixed(
            LOG.getChild(f"worker"), f"{self._thread.name} - "
        )

        async def go() -> None:
            # Attempt (re)connection. Might have to do this multiple times if we lose connection.
            while not self._worker_should_exit():
                try:
                    async with BleakClient(
                        self._address, timeout=TIMEOUT_CONNECT
                    ) as client:
                        await self._worker_using(worker_log, client)
                except TimeoutError:
                    pass  # ignore, keep trying to (re)connect
                except BleakDeviceNotFoundError:
                    pass  # ignore, keep trying to (re)connect
                finally:
                    # Not rigorous, but at least try to clean up a dead connection.
                    self._set_fan_power = None

        try:
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(go())
        except asyncio.CancelledError:
            raise
        except:  # TODO: ignore cancellation & interrupt exception
            worker_log.exception("worker failed")

    async def _worker_using(
        self, log: logging.Logger | logging.LoggerAdapter, client: BleakClient
    ):
        log.info("connected to controller")

        await client.get_services()  # fetch and cache services

        def require(id: UUID):
            x = client.services.get_service(id)
            if x is None:
                raise Exception(f"{client} doesn't have required service {id}")
            return x

        def require_char(service: BleakGATTService, id: UUID):
            x = service.get_characteristic(id)
            if x is None:
                raise Exception(f"{service} has no characteristic {id}")
            return x

        service_env = require(UUID_SERVICE_ENVIRONMENTAL_SENSING)
        service_fan = require(UUID_SERVICE_FAN)
        service_ws2812 = require(UUID_SERVICE_WS2812)

        env_aggregate = require_char(service_env, UUID_CHAR_ENV_DATA_AGGREGATE)
        fan_power = require_char(service_fan, UUID_CHAR_FAN_POWER)
        fan_tacho = require_char(service_fan, UUID_CHAR_FAN_TACHO)
        ws2812_update = require_char(service_ws2812, UUID_CHAR_WS2812_UPDATE)

        async def set_fan_power(percent: float):
            percent = _clamp(percent, 0, 1) * 100
            try:
                await client.write_gatt_char(
                    fan_power, int(percent * 2).to_bytes(1, "little")
                )
            except BleakError:
                # consider non-fatal. don't to abort due to potentially transient error
                log.exception(f"failed to set fan-power={percent}")

        self._set_fan_power = set_fan_power
        self._connected.set()

        async def read(char: BleakGATTCharacteristic):
            return BleAttrReader(await client.read_gatt_char(char))

        # FUTURE WORK: replace polling w/ GATT notify/indicate mechanism.
        while not self._worker_should_exit():
            try:
                (intake, exhaust) = _parse_env_agg(await read(env_aggregate))
                # normalise to [0,1] range
                fan_power_ = ((await read(fan_power)).percentage8() or 0) / 100
                fan_tacho_ = (await read(fan_tacho)).tachometer()
            except BleakError as e:
                # special case: lost connection -> wait and attempt reconnect
                if e.args[0] == "Not connected":
                    log.info("connection lost. attempting reconnection...")
                    return

                log.exception(f"failed to read controller status")
                (intake, exhaust) = (SensorState(), SensorState())
                fan_power_ = 0
                fan_tacho_ = 0

            nevermore = self._nevermore()
            if nevermore is None:
                break  # nevermore instance got GC'd at some point. we're done.

            # HACK: Abuse GIL to keep this thread-safe
            nevermore.state = ControllerState(
                intake=intake,
                exhaust=exhaust,
                fan_power=fan_power_,
                fan_tacho=fan_tacho_,
            )
            nevermore = None  # release reference b/c python doesn't have nice scoping

            await asyncio.sleep(1 / NEVERMORE_POLL_HZ)

    def _worker_should_exit(self):
        return self._loop_exit.is_set() or self._nevermore() is None


class Nevermore:
    def __init__(self, config: ConfigWrapper) -> None:
        self.name = config.get_name().split()[-1]
        self.printer: Printer = config.get_printer()
        self.reactor: SelectReactor = self.printer.get_reactor()
        self.bt_address: str = config.get("bt_address").upper()
        self.state = ControllerState()
        self.fan = NevermoreFan(self)

        if not _bt_address_validate(self.bt_address):
            raise config.error(
                f"`bt_address` isn't a valid bluetooth address, given `{self.bt_address}`"
            )

        self._interface: Optional[NevermoreBackgroundWorker] = None
        self._handle_request_restart(0)

        self.printer.add_object(f"fan_generic {self.fan.name}", self.fan)
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )

    def set_fan_power(self, percent: float):
        if self._interface is not None:
            self._interface.set_fan_power(percent)

    def _handle_connect(self) -> None:
        if self._interface is not None and not self._interface.wait_for_connection(
            TIMEOUT_CONNECT
        ):
            raise self.printer.config_error("nevermore failed to connect - timed out")

        # self.reactor.update_timer(self.sample_timer, self.reactor.NOW)
        pass

    def _handle_request_restart(self, print_time):
        self._handle_shutdown()
        self._interface = NevermoreBackgroundWorker(self)
        # TODO: reset fan & LED to defaults?

    def _handle_shutdown(self):
        if self._interface is not None:
            self._interface.disconnect()
            self._interface = None


# basically ripped from `extras/fan_generic.py`
class NevermoreFan:
    cmd_SET_FAN_SPEED_help = "Sets the speed of a fan"

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

    def get_status(self, eventtime: float) -> dict[str, float]:
        return {
            "speed": 0 if self.nevermore is None else self.nevermore.state.fan_power,
            "rpm": 0 if self.nevermore is None else self.nevermore.state.fan_tacho,
        }

    def _handle_connect(self) -> None:
        self.nevermore = self.printer.lookup_object("nevermore")
        assert isinstance(self.nevermore, Nevermore)

    def cmd_SET_FAN_SPEED(self, gcmd: GCodeCommand) -> None:
        speed = gcmd.get_float("SPEED", 0.0)
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

    def get_status(self, eventtime: float) -> dict[str, float]:
        # HACK: can only plot on mainsail/fluidd if we pretend the VOC Index is a temperature
        if self.plot_voc:
            return {"temperature": self.state.voc_index or 0}

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
            temp = self.state.voc_index

        if temp is not None and self._callback is not None:
            self._callback(measured_time, temp)

        return measured_time + 1  # 1s delay?


def load_config(config: ConfigWrapper):
    heaters = config.get_printer().load_object(config, "heaters")
    heaters.add_sensor_factory("NevermoreSensor", NevermoreSensor)

    return Nevermore(config)
