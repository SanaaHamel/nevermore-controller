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
from abc import ABCMeta, abstractmethod
from dataclasses import dataclass
from typing import (
    Any,
    Callable,
    Coroutine,
    Dict,
    Iterable,
    List,
    MutableMapping,
    Optional,
    Set,
    Tuple,
    Type,
    TypeVar,
    Union,
    overload,
)
from uuid import UUID

import bleak
import serial
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.service import BleakGATTService
from typing_extensions import override

if typing.TYPE_CHECKING:
    LoggerAdapter = logging.LoggerAdapter[logging.Logger]
else:
    LoggerAdapter = logging.LoggerAdapter

_A = TypeVar("_A")
_Float = TypeVar("_Float", bound=float)


NEVERMORE_CONTROLLER_NAMES = {"Nevermore", "Nevermore Controller"}
BOOTLOADER_NAME_PREFIX = "picowota "

TIMESEC16_MAX = 2**16 - 2
VOC_INDEX_MAX = 500
VOC_INDEX_NOT_KNOWN = 0
VOC_RAW_MAX = 2**16 - 2

VOC_GATING_THRESHOLD_MIN = 175


# Non-configurable constants
CONTROLLER_ADVERTISEMENT_PERIOD = 0.5  # seconds, upper bound on adverts
CONTROLLER_CONNECTION_DELAY = 5  # seconds, expected upper bound based on tests
NEVERMORE_SERIAL_BAUDRATE_DEFAULT = 115200

NEVERMORE_SERVO_PERIOD = 1 / 50

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
    WRITE_WITHOUT_RESPONSE = "write-without-response"


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
UUID_SERVICE_PRIMARY = short_uuid(0x2800)
UUID_SERVICE_ENVIRONMENTAL_SENSING = short_uuid(0x181A)
UUID_SERVICE_CONFIGURATION = UUID("b5078b20-aea3-4c37-a18f-b370c03f02a6")
UUID_SERVICE_FAN = UUID("4553d138-1d00-4b6f-bc42-955a89cf8c36")
UUID_SERVICE_WS2812 = UUID("f62918ab-33b7-4f47-9fba-8ce9de9fecbb")
UUID_SERVICE_FAN_POLICY = UUID("260a0845-e62f-48c6-aef9-04f62ff8bffd")
UUID_SERVICE_DISPLAY = UUID("7be8ac4b-7eb4-4e09-b134-91a46b622832")
UUID_SERVICE_PHOTOCATALYTIC = UUID("de44dd71-2400-4cd1-a3f3-9fb00c4697d7")
UUID_SERVICE_SERVO = UUID("8959bb3e-3063-4a46-9b9a-69fdbc327f1c")

UUID_CHAR_GATT = short_uuid(0x2803)
UUID_CHAR_PERCENT8 = short_uuid(0x2B04)
UUID_CHAR_COUNT16 = short_uuid(0x2AEA)
UUID_CHAR_TIMESEC16 = short_uuid(0x2B16)
UUID_CHAR_SERIAL_NUMBER = short_uuid(0x2A25)
UUID_CHAR_HARDWARE_REVISION = short_uuid(0x2A27)
UUID_CHAR_SOFTWARE_REVISION = short_uuid(0x2A28)
UUID_CHAR_PERCENT16_8 = UUID("0543a134-244f-405b-9d43-0351a5336ef7")

UUID_CHAR_DATA_AGGREGATE = UUID("75134bec-dd06-49b1-bac2-c15e05fd7199")
UUID_CHAR_FAN_TACHO = UUID("03f61fe0-9fe7-4516-98e6-056de551687f")
UUID_CHAR_FAN_AGGREGATE = UUID("79cd747f-91af-49a6-95b2-5b597c683129")
UUID_CHAR_FAN_THERMAL = UUID("45d2e7d7-40c4-46a6-a160-43eb02d01e27")
UUID_CHAR_VOC_INDEX = UUID("216aa791-97d0-46ac-8752-60bbc00611e1")
UUID_CHAR_WS2812_UPDATE = UUID("5d91b6ce-7db1-4e06-b8cb-d75e7dd49aae")
UUID_CHAR_SERVO_RANGE = UUID("9c327c7f-188f-4345-950f-bd586f13f324")
UUID_CHAR_CONFIG_FLAGS64 = UUID("d4b66bf4-3d8f-4746-b6a2-8a59d2eac3ce")
UUID_CHAR_CONFIG_REBOOT = UUID("f48a18bb-e03c-4583-8006-5b54422e2045")
UUID_CHAR_CONFIG_RESET = UUID("f2810b13-8cd7-4d6f-bb1b-e276db7fadbf")
UUID_CHAR_CONFIG_CHECKPOINT_SENSOR_CALIBRATION = UUID(
    "a84b00c0-7102-4cc6-a4ea-a65050502d3f"
)
UUID_CHAR_CONFIG_RESET_SENSOR_CALIBRATION = UUID("75bf055c-02be-466f-8c7d-6ebc72078048")
UUID_CHAR_CONFIG_VOC_CALIBRATE_ENABLED = UUID("ee786ac0-7700-47dd-b7de-9958f96303f2")
UUID_CHAR_DISPLAY_UI = UUID("86a25d55-1893-4d01-8ea8-8970f622c243")
UUID_CHAR_CONFIG_PINS = UUID("2e9410cb-30fd-4b2c-8c95-934226a9ba29")
UUID_CHAR_CONFIG_PINS_ERROR = UUID("0f6d7c4b-c30c-45b2-b32a-0e5b130429f0")
UUID_CHAR_CONFIG_PINS_DEFAULT = UUID("5b1dc210-6a51-4cf9-bda7-085604199856")


class DisplayUI(enum.Enum):
    GC9A01_CLASSIC = 0
    GC9A01_SMALL_PLOT = 1
    GC9A01_NO_PLOT = 2


@dataclass(frozen=True)
class BtstackDatabaseAttr:
    # relevant subset of flags defined in `compile_gatt.py`
    class Flags(enum.IntFlag):
        BROADCAST = 0x01
        READ = 0x02
        WRITE_WITHOUT_RESPONSE = 0x04
        WRITE = 0x08
        NOTIFY = 0x10
        INDICATE = 0x20
        EXTENDED_PROPERTIES = 0x80  # FUTURE WORK: handle?
        DYNAMIC = 0x100  # btstack custom
        LONG_UUID = 0x200  # btstack custom

        @classmethod
        def from_prop(cls, prop: CharacteristicProperty):
            return {
                CharacteristicProperty.BROADCAST: cls.BROADCAST,
                CharacteristicProperty.READ: cls.READ,
                CharacteristicProperty.WRITE: cls.WRITE,
                CharacteristicProperty.WRITE_WITHOUT_RESPONSE: cls.WRITE_WITHOUT_RESPONSE,
                CharacteristicProperty.NOTIFY: cls.NOTIFY,
                CharacteristicProperty.INDICATE: cls.INDICATE,
            }[prop]

    flags: Flags
    handle: int
    uuid: UUID
    value: Optional[bytes] = None  # None IFF `flags & DYNAMIC`

    @staticmethod
    def parse(data: memoryview) -> 'BtstackDatabaseAttr':
        if len(data) < 4:
            raise ValueError("malformed attr missing required fields")

        flags = BtstackDatabaseAttr.Flags(
            int.from_bytes(data[:2], 'little', signed=False)
        )
        handle = int.from_bytes(data[2:4], 'little', signed=False)

        uuid_len = 16 if flags & BtstackDatabaseAttr.Flags.LONG_UUID else 2
        uuid_raw = data[4 : 4 + uuid_len]
        if len(uuid_raw) != uuid_len:
            raise ValueError(f'malformed attr too short for uuid (size {len(data)})')

        if flags & BtstackDatabaseAttr.Flags.LONG_UUID:
            uuid = UUID(bytes=bytes(reversed(uuid_raw)))
        else:
            uuid = short_uuid(int.from_bytes(uuid_raw, 'little', signed=False))

        value_raw = data[4 + uuid_len :]
        value = None if flags & BtstackDatabaseAttr.Flags.DYNAMIC else bytes(value_raw)
        return BtstackDatabaseAttr(flags, handle, uuid, value)

    @staticmethod
    def parse_database(db: bytes) -> Iterable['BtstackDatabaseAttr']:
        if len(db) < 1:
            raise ValueError('invalid minimum length')
        if db[0] != 1:
            raise ValueError(f'unhandled db version {db[0]}')

        data = memoryview(db)[1:]
        while True:
            if len(data) < 2:
                raise ValueError('data too short for valid entry')

            size = int.from_bytes(data[:2], 'little', signed=False)
            if size == 0:
                return

            subset = data[2:size]
            data = data[size:]
            if len(subset) < size - 2:
                raise ValueError(
                    f"truncated attr w/ size {size - 2} ({len(subset)} avail)"
                )

            yield BtstackDatabaseAttr.parse(subset)


class Transport:
    class AttrNotFound(Exception):
        def __init__(self, num: int, uuid: UUID, props: Set[CharacteristicProperty]):
            super().__init__(
                f"doesn't have exactly {num} characteristic(s) {uuid} w/ properties {props}"
            )
            self.num = num
            self.uuid = uuid
            self.props = props

    class ServiceNotFound(Exception):
        def __init__(self, uuid: UUID):
            super().__init__(f"doesn't have exactly 1 service for {uuid}")
            self.uuid = uuid

    class Attribute:
        @abstractmethod
        async def write(self, blob: bytes) -> None:
            raise NotImplementedError

        @abstractmethod
        async def read(self) -> bytes:
            raise NotImplementedError

        @abstractmethod
        def has_property(self, prop: CharacteristicProperty) -> bool:
            raise NotImplementedError

        @property
        @abstractmethod
        def handle(self) -> int:
            raise NotImplementedError

    class Service:
        # POST CONDITION: ordered by handle #
        @abstractmethod
        def all(self, uuid: Optional[UUID] = None) -> Iterable['Transport.Attribute']:
            raise NotImplementedError

        def many(
            self,
            uuid: UUID,
            num: Optional[int] = None,
            props: Set[CharacteristicProperty] = {CharacteristicProperty.READ},
        ) -> List['Transport.Attribute']:
            xs = [x for x in self.all(uuid) if all(x.has_property(p) for p in props)]

            if num is not None and len(xs) != num:
                raise Transport.AttrNotFound(num, uuid, props)

            return xs

        def __call__(
            self,
            uuid: UUID,
            props: Set[CharacteristicProperty] = {CharacteristicProperty.READ},
        ) -> 'Transport.Attribute':
            return self.many(uuid, 1, props)[0]

    # May only be used if this instance owns the underlying transport mechanism
    @abstractmethod
    async def close(self) -> None:
        raise NotImplementedError

    @property
    @abstractmethod
    def all(self) -> Iterable['Transport.Service']:
        raise NotImplementedError

    @abstractmethod
    def service(self, uuid: UUID) -> Service:
        raise NotImplementedError

    def __call__(
        self,
        uuid: UUID,
        props: Set[CharacteristicProperty] = {CharacteristicProperty.READ},
    ) -> 'Transport.Attribute':
        xs = [attr for service in self.all for attr in service.many(uuid, None, props)]
        if len(xs) != 1:
            raise Transport.AttrNotFound(1, uuid, props)

        return xs[0]

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.close()


@dataclass(frozen=True)
class TransportBLE(Transport):
    client: BleakClient

    @dataclass(frozen=True)
    class Attribute(Transport.Attribute):
        client: BleakClient
        inner: BleakGATTCharacteristic

        async def write(self, blob: bytes):
            await self.client.write_gatt_char(self.inner, blob)

        async def read(self):
            return bytes(await self.client.read_gatt_char(self.inner))

        @property
        def uuid(self):
            return self.inner.uuid

        def has_property(self, prop: CharacteristicProperty) -> bool:
            return prop.value in self.inner.properties

        @property
        @override
        def handle(self):
            return self.inner.handle

    @dataclass(frozen=True)
    class Service(Transport.Service):
        client: BleakClient
        inner: BleakGATTService

        @override
        def all(self, uuid: Optional[UUID] = None):
            # FUTURE WORK: O(n) for UUID lookup, which goes O(n^2) in total work
            uuid_str = None if uuid is None else str(uuid)
            return sorted(
                (
                    TransportBLE.Attribute(self.client, x)
                    for x in self.inner.characteristics
                    if uuid is None or x.uuid == uuid_str
                ),
                key=lambda x: x.handle,
            )

    @property
    @override
    def all(self):
        for x in self.client.services:
            yield TransportBLE.Service(self.client, x)

    @override
    def service(self, uuid: UUID) -> 'Service':
        x = self.client.services.get_service(uuid)
        if x is None:
            raise Transport.ServiceNotFound(uuid)

        return TransportBLE.Service(self.client, x)

    @override
    async def close(self) -> None:
        await self.client.disconnect()


class TransportSerial(Transport):

    CMD_DB_READ = 0xFA
    CMD_ATTR_READ = 0xFB
    CMD_ATTR_WRITE = 0xFC

    class TimeoutException(Exception):
        def __init__(self, *, read: bool):
            super().__init__(f"{'read' if read else 'write'} timed out")
            self.read = read

    @dataclass(frozen=True)
    class Attribute(Transport.Attribute):
        transport: 'TransportSerial'
        attr: BtstackDatabaseAttr

        async def write(self, blob: bytes):
            if 0xFFFF < len(blob):
                raise ValueError(f"payload too big size={len(blob)}")

            self.transport._read_clear()
            self.transport._write(
                bytes([TransportSerial.CMD_ATTR_WRITE])
                + self.attr.handle.to_bytes(2, 'little')
                + len(blob).to_bytes(2, 'little')
                + blob
            )

            self.transport._read_until(TransportSerial.CMD_ATTR_WRITE)
            handle = self.transport._read_uint(2)
            okay = self.transport._read_uint(1)

            if handle != self.handle:
                raise Exception(f"replied w/ handle {handle}, expected {self.handle}")
            if not okay:
                raise Exception(f"write failed w/ handle {handle}")

        async def read(self):
            if not self.attr.flags & BtstackDatabaseAttr.Flags.DYNAMIC:
                assert self.attr.value is not None
                return self.attr.value

            self.transport._read_clear()
            self.transport._write(
                bytes([TransportSerial.CMD_ATTR_READ])
                + self.attr.handle.to_bytes(2, 'little')
            )

            self.transport._read_until(TransportSerial.CMD_ATTR_READ)
            handle = self.transport._read_uint(2)
            okay = self.transport._read_uint(1)
            data = self.transport._read_payload()
            if handle != self.handle:
                raise Exception(f"replied w/ handle {handle}, expected {self.handle}")
            if not okay:
                raise Exception(f"read failed w/ handle {handle}")

            return data

        def has_property(self, prop: CharacteristicProperty):
            return (self.attr.flags & self.attr.flags.from_prop(prop)) != 0

        @property
        @override
        def handle(self):
            return self.attr.handle

    @dataclass(frozen=True)
    class Service(Transport.Service):
        attr: BtstackDatabaseAttr
        children: List['TransportSerial.Attribute']

        @property
        def uuid(self):
            assert self.attr.value is not None
            if len(self.attr.value) == 2:
                return short_uuid(int.from_bytes(self.attr.value, 'little'))
            else:
                assert len(self.attr.value) == 16
                return UUID(bytes=bytes(reversed(self.attr.value)))

        @override
        def all(self, uuid: Optional[UUID] = None):
            # FUTURE WORK: O(n) for UUID lookup, which goes O(n^2) in total work
            return sorted(
                (x for x in self.children if uuid is None or x.attr.uuid == uuid),
                key=lambda x: x.handle,
            )

    def __init__(self, serial: serial.Serial):
        self.services: List[TransportSerial.Service] = []
        self.serial = serial

        self._read_clear()  # drop everything that might be cached by the OS

        # fetch DB
        self._write(bytes([self.CMD_DB_READ]))
        self._read_until(self.CMD_DB_READ)
        database = self._read_payload()

        attrs = list(BtstackDatabaseAttr.parse_database(database))
        handle_2_index = {attrs[i].handle: i for i in range(len(attrs))}
        # decorate everything with GATT information, not just attr info
        for attr in attrs:
            if attr.uuid == UUID_CHAR_GATT:
                # flags (1), handle (2), uuid (2 or 16)
                assert attr.value is not None and len(attr.value) - 3 in [2, 16]
                gatt_flags = BtstackDatabaseAttr.Flags(attr.value[0])
                handle = int.from_bytes(attr.value[1:2], 'little')
                assert (
                    handle in handle_2_index
                ), f"malformed attr db references unknown handle {handle}"
                index = handle_2_index[handle]
                attrs[index] = dataclasses.replace(
                    attrs[index], flags=attrs[index].flags | gatt_flags
                )

        # partition into services
        for attr in attrs:
            if attr.uuid == UUID_SERVICE_PRIMARY:
                self.services.append(TransportSerial.Service(attr, []))
            elif not self.services:
                continue  # ignore attrib, it isn't part of a primary service
            else:
                self.services[-1].children.append(TransportSerial.Attribute(self, attr))

    @property
    @override
    def all(self):
        return self.services

    @override
    def service(self, uuid: UUID):
        xs = [x for x in self.services if x.uuid == uuid]
        if len(xs) != 1:
            raise Transport.ServiceNotFound(uuid)

        return xs[0]

    @override
    async def close(self) -> None:
        self.serial.close()

    def _write(self, data: bytes):
        if self.serial.write(data) != len(data):  # timed out / failed
            raise TransportSerial.TimeoutException(read=False)

    def _read_uint(self, n) -> int:
        data = self.serial.read(n)
        if len(data) < n:  # timed out / failed
            raise TransportSerial.TimeoutException(read=True)

        return int.from_bytes(data, 'little', signed=False)

    def _read_until(self, cmd: int):
        # incorrect type annotation. `serial.Timeout` can be construct w/ `None`
        timeout = serial.Timeout(self.serial.timeout)
        cmd_bytes = bytes([cmd])
        while True:
            if self.serial.read(1) == cmd_bytes:
                break

            if timeout.expired():
                raise TransportSerial.TimeoutException(read=True)

    def _read_payload(self) -> bytes:
        size = self._read_uint(2)
        data = self.serial.read(size)
        if len(data) != size:
            raise TransportSerial.TimeoutException(read=True)

        return data

    def _read_clear(self) -> None:
        self.serial.read_all()  # flush any pending junk


class CommBindings:
    def __init__(self, transport: Transport):
        P = CharacteristicProperty

        self.config = transport.service(UUID_SERVICE_CONFIGURATION)
        self.env = transport.service(UUID_SERVICE_ENVIRONMENTAL_SENSING)
        self.fan = transport.service(UUID_SERVICE_FAN)
        self.fan_policy = transport.service(UUID_SERVICE_FAN_POLICY)
        self.ws2812 = transport.service(UUID_SERVICE_WS2812)
        self.display = transport.service(UUID_SERVICE_DISPLAY)
        self.servo = transport.service(UUID_SERVICE_SERVO)

        self.aggregate_env = self.env(UUID_CHAR_DATA_AGGREGATE, {P.READ, P.NOTIFY})
        self.aggregate_fan = self.fan(UUID_CHAR_FAN_AGGREGATE, {P.READ, P.NOTIFY})
        # HACK: it's the first one in the list (ordered by handle #). this is brittle.
        (
            self.fan_power_override,
            self.fan_power_passive,
            self.fan_power_auto,
            self.fan_power_coeff,
        ) = self.fan.many(UUID_CHAR_PERCENT8, 4, {P.WRITE})
        self.ws2812_length = self.ws2812(UUID_CHAR_COUNT16, {P.WRITE})
        self.ws2812_update = self.ws2812(
            UUID_CHAR_WS2812_UPDATE, {P.WRITE_WITHOUT_RESPONSE}
        )
        self.fan_policy_cooldown = self.fan_policy(UUID_CHAR_TIMESEC16, {P.WRITE})
        self.fan_policy_voc_passive_max, self.fan_policy_voc_improve_min = (
            self.fan_policy.many(UUID_CHAR_VOC_INDEX, 2, {P.WRITE})
        )
        self.fan_thermal_limit = self.fan_policy(UUID_CHAR_FAN_THERMAL, {P.WRITE})
        self.config_flags = self.config(UUID_CHAR_CONFIG_FLAGS64, {P.WRITE})
        self.config_reboot = self.config(UUID_CHAR_CONFIG_REBOOT, {P.WRITE})
        self.config_reset = self.config(UUID_CHAR_CONFIG_RESET, {P.WRITE})
        self.config_checkpoint_sensor_calibration = self.config(
            UUID_CHAR_CONFIG_CHECKPOINT_SENSOR_CALIBRATION, {P.WRITE}
        )
        self.config_reset_sensor_calibration = self.config(
            UUID_CHAR_CONFIG_RESET_SENSOR_CALIBRATION, {P.WRITE}
        )
        self.config_voc_threshold, self.config_voc_threshold_override = (
            self.config.many(UUID_CHAR_VOC_INDEX, 2, {P.WRITE})
        )
        self.config_voc_calibrate_enabled = self.config(
            UUID_CHAR_CONFIG_VOC_CALIBRATE_ENABLED, {P.WRITE}
        )
        self.display_brightness = self.display(UUID_CHAR_PERCENT8, {P.WRITE})
        self.display_ui = self.display(UUID_CHAR_DISPLAY_UI, {P.WRITE})
        self.servo_vent_range = self.servo(UUID_CHAR_SERVO_RANGE, {P.WRITE})
        self.servo_vent_pwm = self.servo(UUID_CHAR_PERCENT16_8, {P.WRITE})


def _clamp(x: _Float, min: _Float, max: _Float) -> _Float:
    if x < min:
        return min
    if max < x:
        return max
    return x


class Command:
    @abstractmethod
    async def dispatch(self, comm: CommBindings):
        raise NotImplementedError


class CommandSimple(Command):
    @abstractmethod
    def params(self) -> bytes:
        raise NotImplementedError


class CommandSimplePercent(CommandSimple):
    percent: Optional[float]

    @override
    def params(self) -> bytes:
        if self.percent is None:
            return bytes([0xFF])  # 0xFF -> percent8 special value: not-known

        p = _clamp(self.percent, 0, 1) * 100
        return int(p * 2).to_bytes(1, "little")


def cmd_simple(dispatcher: Callable[[CommBindings], Transport.Attribute]):
    def wrap(cls: Type[CommandSimple]):
        class Derived(dataclass(frozen=True)(cls)):
            async def dispatch(self, comm: CommBindings):
                await dispatcher(comm).write(self.params())

        return Derived

    return wrap


@cmd_simple(lambda x: x.fan_power_override)
class CmdFanPowerOverride(CommandSimplePercent):
    percent: Optional[float]


@cmd_simple(lambda x: x.fan_power_passive)
class CmdFanPowerPassive(CommandSimplePercent):
    percent: float


@cmd_simple(lambda x: x.fan_power_auto)
class CmdFanPowerAuto(CommandSimplePercent):
    percent: float


@cmd_simple(lambda x: x.fan_power_coeff)
class CmdFanPowerCoeff(CommandSimplePercent):
    percent: float


@cmd_simple(lambda x: x.fan_thermal_limit)
class CmdFanPolicyThermalLimit(CommandSimple):
    min: Optional[float]
    max: Optional[float]
    percent: Optional[float]

    @override
    def params(self) -> bytes:
        return (
            BleAttrWriter()
            .temperature(self.min)
            .temperature(self.max)
            .percentage16_10(None if self.percent is None else self.percent * 100)
            .value
        )


@cmd_simple(lambda x: x.ws2812_length)
class CmdWs2812Length(CommandSimple):
    n_total_components: int

    @override
    def params(self):
        return int(self.n_total_components).to_bytes(2, "little")


@dataclass(frozen=True)
class CmdServoRange(CommandSimple, metaclass=ABCMeta):
    start: Optional[float]
    end: Optional[float]

    @override
    def params(self):
        return (
            BleAttrWriter()
            .percentage16_10(None if self.start is None else self.start * 100)
            .percentage16_10(None if self.end is None else self.end * 100)
            .value
        )


class CmdServoPWM(CommandSimple, metaclass=ABCMeta):
    percent: Optional[float]

    @override
    def params(self):
        return (
            BleAttrWriter()
            .percentage16_10(None if self.percent is None else self.percent * 100)
            .value
        )


@cmd_simple(lambda x: x.servo_vent_pwm)
class CmdServoVentPWM(CmdServoPWM):
    percent: Optional[float]


@cmd_simple(lambda x: x.config_reboot)
class CmdConfigReboot(CommandSimple):
    @override
    def params(self):
        return (0).to_bytes(1, "little")


@cmd_simple(lambda x: x.config_reset)
class CmdConfigReset(CommandSimple):
    flags: int

    @override
    def params(self):
        return self.flags.to_bytes(1, "little")


@cmd_simple(lambda x: x.config_checkpoint_sensor_calibration)
class CmdConfigCheckpointSensorCalibration(CommandSimple):
    @override
    def params(self):
        return b""


@cmd_simple(lambda x: x.config_reset_sensor_calibration)
class CmdConfigResetSensorCalibration(CommandSimple):
    @override
    def params(self):
        return b""


@cmd_simple(lambda x: x.config_voc_threshold)
class CmdConfigVocThreshold(CommandSimple):
    value: int

    @override
    def params(self):
        return _clamp(self.value, 0, VOC_INDEX_MAX).to_bytes(2, "little")


@cmd_simple(lambda x: x.config_voc_threshold_override)
class CmdConfigVocThresholdOverride(CommandSimple):
    value: int

    @override
    def params(self):
        return _clamp(self.value, 0, VOC_INDEX_MAX).to_bytes(2, "little")


@cmd_simple(lambda x: x.config_voc_calibrate_enabled)
class CmdConfigVocCalibrateEnabled(CommandSimple):
    value: bool

    @override
    def params(self):
        return int(self.value).to_bytes(1, "little")


@cmd_simple(lambda x: x.display_brightness)
class CmdDisplayBrightness(CommandSimple):
    percent: float

    @override
    def params(self):
        p = _clamp(self.percent, 0, 1) * 100
        return int(p * 2).to_bytes(1, "little")


@cmd_simple(lambda x: x.display_ui)
class CmdDisplayUI(CommandSimple):
    value: int

    @override
    def params(self):
        return self.value.to_bytes(1, "little")


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

    if isinstance(e, KeyError):
        # HACK: bleak is a pile of shit and will occasionally seem to index a
        #       path that isn't in one of its dicts.
        #       e.g. '/org/bluez/hci0/dev_28_CD_C1_0B_7B_64/service0082'
        if "/org/bluez/hci" in str(e).lower():
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
