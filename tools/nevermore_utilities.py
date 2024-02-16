# Common library for BT comm. Used by tools and the klipper module.
#
# Copyright (C) 2023       Sanaa Hamel
#
# This file may be distributed under the terms of the GNU AGPLv3 license.

import asyncio
import dataclasses
import enum
import logging
import typing
from dataclasses import dataclass
from typing import (
    Any,
    Callable,
    Coroutine,
    Dict,
    List,
    MutableMapping,
    Optional,
    Tuple,
    TypeVar,
    Union,
    overload,
)
from uuid import UUID

import bleak
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

if typing.TYPE_CHECKING:
    LoggerAdapter = logging.LoggerAdapter[logging.Logger]
else:
    LoggerAdapter = logging.LoggerAdapter

_A = TypeVar("_A")


NEVERMORE_CONTROLLER_NAMES = {"Nevermore", "Nevermore Controller"}
BOOTLOADER_NAME_PREFIX = "picowota "

TIMESEC16_MAX = 2**16 - 2
VOC_INDEX_MAX = 500
VOC_RAW_MAX = 2**16 - 2


# Non-configurable constants
CONTROLLER_ADVERTISEMENT_PERIOD = 0.5  # seconds, upper bound on adverts
CONTROLLER_CONNECTION_DELAY = 5  # seconds, expected upper bound based on tests

# Derived constants
# must cover at least 2-3 advert periods
BT_SCAN_KNOWN_ADDRESS_TIMEOUT = CONTROLLER_ADVERTISEMENT_PERIOD * 2
BT_SCAN_GATHER_ALL_TIMEOUT = BT_SCAN_KNOWN_ADDRESS_TIMEOUT * 5  # seconds


# Not actually provided by `bleak`. IDK why not.
class CharacteristicProperty(enum.Enum):
    BROADCAST = "broadcast"
    INDICATE = "indicate"
    NOTIFY = "notify"
    READ = "read"
    WRITE = "write"
    WRITE_NO_RESPONSE = "write-without-response"


class LogPrefixed(LoggerAdapter):
    def __init__(
        self,
        logger: Union[logging.Logger, LoggerAdapter],
        format: Callable[[str], str],
    ):
        super().__init__(logger, None)  # type: ignore
        self.format = format

    def process(self, msg: Any, kwargs: MutableMapping[str, Any]):
        return self.format(msg), kwargs


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
UUID_CHAR_SOFTWARE_REVISION = short_uuid(0x2A28)
UUID_CHAR_DATA_AGGREGATE = UUID("75134bec-dd06-49b1-bac2-c15e05fd7199")
UUID_CHAR_FAN_TACHO = UUID("03f61fe0-9fe7-4516-98e6-056de551687f")
UUID_CHAR_FAN_AGGREGATE = UUID("79cd747f-91af-49a6-95b2-5b597c683129")
UUID_CHAR_FAN_THERMAL = UUID("45d2e7d7-40c4-46a6-a160-43eb02d01e27")
UUID_CHAR_VOC_INDEX = UUID("216aa791-97d0-46ac-8752-60bbc00611e1")
UUID_CHAR_WS2812_UPDATE = UUID("5d91b6ce-7db1-4e06-b8cb-d75e7dd49aae")
UUID_CHAR_CONFIG_FLAGS64 = UUID("d4b66bf4-3d8f-4746-b6a2-8a59d2eac3ce")
UUID_CHAR_CONFIG_REBOOT = UUID("f48a18bb-e03c-4583-8006-5b54422e2045")


# must be of the form `xx:xx:xx:xx:xx:xx`, where `x` is a hex digit (uppercase)
# FUTURE WORK: Won't work on MacOS. It uses UUIDs to abstract/hide the BT address.
def bt_address_validate(addr: str):
    octets = addr.split(":")
    if len(octets) != 6:
        return False
    if not all(len(octet) == 2 for octet in octets):
        return False
    if not all(x in "0123456789ABCEDF" for octet in octets for x in octet):
        return False

    return True


def is_lost_connection_exception(e: Exception, is_connecting: bool = False) -> bool:
    if isinstance(e, bleak.exc.BleakDBusError):
        # potentially caused by noisy environments or poor timing
        if e.dbus_error == "org.bluez.Error.NotConnected":
            return True

        ANY_TIME_DBUS_ERRORS = {
            ("org.bluez.Error.Failed", "br-connection-canceled"),
            ("org.bluez.Error.Failed", "Software caused connection abort"),
            ("org.bluez.Error.Failed", "Operation already in progress"),
        }

        CONNECTING_DBUS_ERRORS = {
            ("org.bluez.Error.Failed", "Operation already in progress"),
        }

        if (e.dbus_error, e.dbus_error_details) in ANY_TIME_DBUS_ERRORS:
            return True

        if (
            is_connecting
            and (e.dbus_error, e.dbus_error_details) in CONNECTING_DBUS_ERRORS
        ):
            return True

    if isinstance(e, bleak.exc.BleakError):
        msg = str(e).lower()
        if "not connected" in msg:
            return True
        if "device disconnected" in msg:
            return True

    return False


# TODO: This function is much more generalised than it needs to be here.
#       Reason: It is also used by `klipper.py`, but I haven't had the time or
#       energy to extract them to a shared library.
@overload
async def retry_if_disconnected(
    device: Union[
        Callable[[], Coroutine[Any, Any, Union[BLEDevice, str]]], BLEDevice, str
    ],
    go: Callable[[BleakClient], Coroutine[Any, Any, _A]],
    *,
    connection_timeout: Optional[float] = 10,
    exc_filter: Callable[[Exception], bool] = lambda e: False,
    log: Union[logging.Logger, LoggerAdapter] = logging.root,
    retry: None = None,
) -> _A:
    pass


@overload
async def retry_if_disconnected(
    device: Union[
        Callable[[], Coroutine[Any, Any, Union[BLEDevice, str]]], BLEDevice, str
    ],
    go: Callable[[BleakClient], Coroutine[Any, Any, _A]],
    *,
    connection_timeout: Optional[float] = 10,
    exc_filter: Callable[[Exception], bool] = lambda e: False,
    log: Union[logging.Logger, LoggerAdapter] = logging.root,
    retry: Optional[Callable[[], Union[bool, Coroutine[Any, Any, bool]]]] = None,
) -> Optional[_A]:
    pass


# TODO: There are significant chunks of duplicated code shared between
#       `nevermore.py` and the tooling scripts.
#       Ideally these should be extracted to a shared library, but the Klipper
#       path requirements makes this more complicated...
async def retry_if_disconnected(
    device: Union[
        Callable[[], Coroutine[Any, Any, Union[BLEDevice, str]]], BLEDevice, str
    ],
    go: Callable[[BleakClient], Coroutine[Any, Any, _A]],
    *,
    connection_timeout: Optional[float] = 10,
    exc_filter: Callable[[Exception], bool] = lambda e: False,
    log: Union[logging.Logger, LoggerAdapter] = logging.root,
    retry: Optional[Callable[[], Union[bool, Coroutine[Any, Any, bool]]]] = None,
) -> Optional[_A]:
    assert connection_timeout is None or 0 < connection_timeout

    while True:
        is_connected = False
        try:
            addr = device if isinstance(device, (BLEDevice, str)) else await device()
            timeout = 10 if connection_timeout is None else connection_timeout
            async with BleakClient(addr, timeout=timeout) as client:
                is_connected = True
                return await go(client)
        except asyncio.TimeoutError:
            # `TimeoutError` after we've connected aren't a connection timeout
            # and must not be suppressed.
            if is_connected or connection_timeout is not None:
                raise
        except bleak.exc.BleakDeviceNotFoundError:
            # same thing as `TimeoutError`
            if is_connected or connection_timeout is not None:
                raise
        except Exception as e:
            if is_lost_connection_exception(e, not is_connected):
                # don't be (too) noisy about it, it happens
                log.debug("connection lost.", exc_info=e)
                log.info("connection lost. attempting reconnection...")
            elif not exc_filter(e):
                raise

        reattempt = retry is None or retry()
        reattempt = reattempt if isinstance(reattempt, bool) else await reattempt
        if not reattempt:
            return None


async def device_is_likely_a_nevermore(
    device: BLEDevice, log: Union[logging.Logger, LoggerAdapter] = logging.root
) -> bool:
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
        return x.services.get_characteristic(UUID_CHAR_CONFIG_FLAGS64) is not None

    try:
        return await retry_if_disconnected(device, go, log=log) or False
    except TimeoutError:
        return False  # unable to connect to it in a timely manner, ignore the device


async def discover_bluetooth_devices(
    filter: Callable[[BLEDevice], Coroutine[Any, Any, bool]],
    address: Optional[str] = None,
    timeout: float = BT_SCAN_GATHER_ALL_TIMEOUT,
) -> List[BLEDevice]:
    if address is not None:
        device = await BleakScanner.find_device_by_address(address, timeout=timeout)
        return [device] if device is not None else []

    return [x for x in await BleakScanner.discover(timeout=timeout) if await filter(x)]


class BleAttrReaderNotEnoughData(Exception):
    pass


class BleAttrReader:
    def __init__(self, raw: bytearray):
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

    def voc_index(self) -> Optional[int]:  # [1, VOC_INDEX_MAX]
        return self._as_int(self._unsigned(2, 1, 0, 0, not_known=0))

    def voc_index_threshold(self) -> Optional[int]:  # [0, 2**16-2]
        return self._as_int(self._unsigned(2, 1, 0, 0, not_known=0xFFFF))

    def voc_raw(self) -> Optional[int]:  # [0, VOC_RAW_MAX]
        return self._as_int(self._unsigned(2, 1, 0, 0, not_known=0xFFFF))

    def gia_sigmoid(self) -> Optional[float]:  # [0, 1]
        # basically fix-16 but only the low 16 bits
        return self._unsigned(2, 1, 0, -15, not_known=0xFFFF)

    def fix16(self) -> float:  # [-2^15, 2^15-1]
        return self._unsigned(4, 1, 0, -15)

    @overload
    def _signed(self, sz: int, M: int, d: int, e: int) -> float: ...

    @overload
    def _signed(
        self, sz: int, M: int, d: int, e: int, *, not_known: int
    ) -> Optional[float]: ...

    def _signed(
        self, sz: int, M: int, d: int, e: int, *, not_known: Optional[int] = None
    ) -> Optional[float]:
        return self._consume(True, sz, M, d, e, not_known)

    @overload
    def _unsigned(self, sz: int, M: int, d: int, e: int) -> float: ...

    @overload
    def _unsigned(
        self, sz: int, M: int, d: int, e: int, *, not_known: int
    ) -> Optional[float]: ...

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


@dataclass
class FanState:
    # key names must match those used by moonraker/klipper status (e.g. `rpm` instead of `tacho`)
    speed: Optional[float] = None
    rpm: Optional[float] = None

    def as_dict(self) -> Dict[str, float]:
        return {
            f.name: getattr(self, f.name)
            for f in dataclasses.fields(self)
            if getattr(self, f.name) is not None
        }

    @staticmethod
    def parse(reader: BleAttrReader) -> "FanState":
        x = FanState(
            speed=reader.percentage8(),
            rpm=reader.tachometer(),
        )
        if x.speed is not None:
            x.speed /= 100  # remap [0, 1] to match moonraker's data
        return x


@dataclass
class GIAState:
    # key names must match those used by moonraker/klipper status (e.g. `gas` instead of `voc`)
    # (except for `exhaust` suffix)
    mean: Optional[int]  # [0, VOC_RAW_MAX]
    var: Optional[int]  # [0, VOC_RAW_MAX]
    gamma_mean: Optional[float]  # [0, 1]
    gamma_var: Optional[float]  # [0, 1]
    gating_mean: Optional[float]  # [0, 1]
    gating_var: Optional[float]  # [0, 1]
    threshold_gating_mean: Optional[int]  # [1, VOC_INDEX_MAX + some extra]
    threshold_gating_var: Optional[int]  # [1, VOC_INDEX_MAX + some extra]

    @staticmethod
    def parse(reader: BleAttrReader) -> "GIAState":
        return GIAState(
            mean=reader.voc_raw(),
            var=reader.voc_raw(),
            gamma_mean=reader.gia_sigmoid(),
            gamma_var=reader.gia_sigmoid(),
            gating_mean=reader.gia_sigmoid(),
            gating_var=reader.gia_sigmoid(),
            threshold_gating_mean=reader.voc_index_threshold(),
            threshold_gating_var=reader.voc_index_threshold(),
        )


@dataclass
class VOCRawBreakdown:
    humidity: Optional[int]  # [0, VOC_RAW_MAX]
    temperature: Optional[int]  # [0, VOC_RAW_MAX]
    uncompensated: Optional[int]  # [0, VOC_RAW_MAX]

    @staticmethod
    def parse(reader: BleAttrReader) -> "VOCRawBreakdown":
        return VOCRawBreakdown(
            humidity=reader.voc_raw(),
            temperature=reader.voc_raw(),
            uncompensated=reader.voc_raw(),
        )


@dataclass(frozen=True)
class SensorState:
    temperature: Optional[float] = None  # Celsius
    humidity: Optional[float] = None  # %
    pressure: Optional[float] = None  # hPa
    # HACK: Must be named `gas` b/c Fluidd only looks at `temperature`, `humidity`, `pressure`, and `gas`.
    #       Also happens to be what the bme680 reports.
    gas: Optional[int] = None  # [1, VOC_INDEX_MAX]
    gas_raw: Optional[int] = None  # [0, VOC_RAW_MAX]
    gas_raw_gia: Optional[GIAState] = None
    gas_raw_breakdown: Optional[VOCRawBreakdown] = None

    def as_dict(self) -> Dict[str, float]:
        d = {
            f.name: getattr(self, f.name)
            for f in dataclasses.fields(self)
            if isinstance(getattr(self, f.name), (int, float))
        }

        def add_subfields(prefix: str, x: Any):
            if x is not None:
                d.update(
                    (f"{prefix}{f.name}", getattr(x, f.name))
                    for f in dataclasses.fields(x)
                    if getattr(x, f.name) is not None
                )

        add_subfields("gas_raw_", self.gas_raw_gia)
        add_subfields("gas_raw_", self.gas_raw_breakdown)
        return d

    @staticmethod
    def parse(reader: BleAttrReader) -> "Tuple[SensorState, SensorState]":
        t_in = reader.temperature()
        t_out = reader.temperature()
        _t_mcu = reader.temperature()  # unused/ignored
        h_in = reader.humidity()
        h_out = reader.humidity()
        p_in = reader.pressure()
        p_out = reader.pressure()
        voc_in = reader.voc_index()
        voc_out = reader.voc_index()
        voc_raw_in = reader.voc_raw()
        voc_raw_out = reader.voc_raw()
        gia_in = GIAState.parse(reader)
        gia_out = GIAState.parse(reader)
        breakdown_in = VOCRawBreakdown.parse(reader) if reader.remaining else None
        breakdown_out = VOCRawBreakdown.parse(reader) if reader.remaining else None

        if p_in is not None:
            p_in /= 100  # need it in hPa instead of Pa

        if p_out is not None:
            p_out /= 100  # need it in hPa instead of Pa

        return (
            SensorState(t_in, h_in, p_in, voc_in, voc_raw_in, gia_in, breakdown_in),
            SensorState(
                t_out, h_out, p_out, voc_out, voc_raw_out, gia_out, breakdown_out
            ),
        )


@dataclass
class ControllerState:
    intake: SensorState = SensorState()
    exhaust: SensorState = SensorState()
    fan_power: float = 0
    fan_tacho: float = 0

    def min(self, rhs: "ControllerState") -> "ControllerState":
        return self.merge(rhs, min)

    def max(self, rhs: "ControllerState") -> "ControllerState":
        return self.merge(rhs, max)

    def merge(
        self,
        rhs: "ControllerState",
        fn: Callable[[float, float], float],
    ) -> "ControllerState":
        # TODO: figure out how (if possible?) to type bound a dataclass kind
        # TODO: PRECONDITION: all fields are `<: float` or dataclasses satisfying this precondition
        def go(
            x: _A,
            y: _A,
        ) -> _A:
            if x is None:
                return x
            if y is None:
                return y

            if isinstance(x, (int, float)):
                return fn(x, y)  # type: ignore

            return x.__class__(
                **{
                    f.name: go(getattr(x, f.name), getattr(y, f.name))
                    for f in dataclasses.fields(x)  # type: ignore
                }
            )

        return go(self, rhs)

    def as_dict(self) -> Dict[str, float]:
        data: Dict[str, float] = {}

        def add(fmt: str, k: str, v: Optional[float]):
            if v is not None:
                data[fmt.format(k)] = v

        add("{0}", "speed", self.fan_power)
        add("{0}", "rpm", self.fan_tacho)
        for k, v in self.intake.as_dict().items():
            add("intake_{0}", k, v)
        for k, v in self.exhaust.as_dict().items():
            add("exhaust_{0}", k, v)

        return data
