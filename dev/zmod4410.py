# Micropython toy for reverse engineering the ZMOD4410 I2C API.
#
# DO NOT USE. For reference only.
#
# No serious attempt was made to make this pretty.
#


# Command field:
#   Type:   1 octet bitfield
#       Command Format:
#                       Run Sequencer
#                      / Continuous
#                     //
#                   0bRC??'????
#
#       Run Sequencer:  When 1, will execute the sequencer.
#                       Can set to 0 at any time to abort the current execution.
#                       SIDE EFFECT: Status.Running will be set to the same value.
#       Continuous:     When 1, will loop the sequencer back to start instead of
#                       stopping when the sequencer hits a STOP step.
#                       SIDE EFFECT: Status.SleepAlarmEnabled will be set to the same value.
#
#                       SDK for IAQ1 (which uses continuous) polls status every 50ms
#                       until status reports its on the last step (racy?), then reads the
#                       result.
#
#   Examples seen:
#       0x80    Most common form.
#       0xC0    Seen in a few configs.
#
# Command 0xE0 induces 0xC0 / 0xC1 state.
# Command 0x60 induces 0x41 (was previously 0x01); can cancel out w/ cmd 0x00
# Command 0x40 induces 0x41 (was previously 0x01); can cancel out w/ cmd 0x00
# Command 0x20 is no-op (was previously 0x01);
# Command 0xA0 induces 0x80 (was previously 0x01); then quickly moved to 0x01 (nominal execution?)
#

# Status field:
#   Type:   1 octet bitfield
#       Command Format:
#                        Running
#                       / SleepAlarmEnabled
#                      // Alarm
#                     ///
#                   0bRCA?'????
#
#       Running:                1 IFF the sequencer is running.
#       Sleep Alarm Enabled:    1 IFF continuous mode is enabled. (See Command.)
#                               Name based on SDK constant. It is a misleading name?
#       Alarm:                  Unknown, mentioned in documentation.
#

# Config notes:
#   IAQ1/Odor use same configs
#   IAQ1/Odor and RAQ use *almost* the same configs (RAQ has slightly longer delay)
#   IAQ2/rel-IAQ/Sulfur-Odor use same configs
#   IAQ2/... and PBAQ use *almost* the same configs
#

# Delay:
#   u2 BE, 0.2ms delay per tick.
#       Determined by linear regression over `range(100) * 100` and wall-clock.
#
#   TODO: Doesn't look quite right.
#           RAQ w/ cmd=0x80 should have an execution time of ~4900ms.
#           In practice it takes ~4100ms. (????)
#
#   Unknown how long is spent heating. Programmable?
#   Likely heating is done at specified power for entire delay.
#       (Based on `cleaning` profile.)

# Measure:
#   Type:   array 8 of 1 octet commands
#       Command Format:
#                       source?
#                      /       Unk2
#                     /       /
#                   0bSSS?'??11
#
#       Source:
#           0b000   heater ADC
#           0b001   heater resistance (how is this different than heater ADC?)
#                       Can be map to a celsius value.
#                           Starts at ~ambient.
#                           Rises w/ repeat measurements.
#                           Goes back to ambient when idled for a while.
#                           Device temperature?
#           0b010   ??? used by IAQ2/PBAQ
#                       Returns 0?
#           0b011   ??? Not used.
#                       Returns 65535 ( 0xFFFF )
#           0b100   ??? Not used.
#                       Returns something. Varies lightly.
#                       Monotonically increases w/ repeat measures, drops when idle.
#                           Similar to heater-res?
#           0b101   ??? used by IAQ2/PBAQ
#                       Returns something. Varies lightly.
#                       Monotonically increases slowly w/ repeat measures, drops when idle.
#                           Lower magnitude than 0b100
#           0b110   mox/adc min
#                       Varies slightly between repeat measures. (?!)
#                           e.g. 3445, 3445, 3444, 3443, 3441, 3440, 3440, 3438
#                                3445, 3444, 3443, 3442, 3442, 3440, 3440, 3441
#                       Not hardcoded/nvm? Is this updated live?
#           0b111   mox/adc max
#                       Also varies slightly. Not nearly as much as mox-min.
#                           e.g. 62305, 62305, 62304, 62305, 62304, 62304, 62304, 62304
#
#       Unk2:
#           0b11    Unknown. Always set to 11.
#
#   Examples Seen:
#     0x03  0b....'..11     heater ADC, almost certainly
#     0x23  0b..1.'..11     ??? used by many
#     0x43  0b.1..'..11     ??? used by IAQ2/PBAQ
#     0xA3  0b1.1.'..11     ??? used by IAQ2/PBAQ
#     0xC3  0b11..'..11     mox/adc min, almost certainly
#     0xE3  0b111.'..11     mox/adc max, almost certainly
#
#
# >>> prep_exec(zmod4410_init, measure=b'\x03\x13')
#     [60063, 60620]
#                                  !!!
# >>> init,   xC0,   xC1,   xC2,   xC3,   xC4,   xC8,   xD0
#            2976,  3248,  3380,  3446,  3008,  2720,  3040
# >>> init,   xE0,   xE1,   xE2,   xE3,   xE4,   xE8,   xF0
#           62816, 62512, 62376, 62304, 62784, 62816, 62816

# Sequencer:
#   type is seq of u2 (BE?), array length 16.
#       Command Format:
#                   Stop Seq  Measure Idx  Heater Idx
#                      /       / Delay Idx/
#                     /       /   /      /
#                   0bS...'???M'MMDD'DHHH
#
#       Unknown Bitsets:
#           0x0A__  0b_...'111_'____'____   IAQ1-LP, 2x start
#           0x03__  0b_...'.11_'____'____   IAQ2, every time
#           0x03__  0b_...'..1_'____'____   NO2_O2, 3x sporadic
#         Measure? Delay? Unknown:
#
#           0x00  0b....'....               everyone
#           0x01  0b....'...1               OAQ1    (d 3 m 1)
#           0x02  0b....'..1.               OAQ1    (d 3 m 1)
#           0x03  0b....'..11               OAQ1    (d 3 m 1)
#           0x04  0b....'.1..               OAQ1    (d 3 m 1)
#           0x08  0b....'1...               IAQ1    (d 2 m 1)
#                                           OAQ1    (d 3 m 1)
#                                           Odor    (d 2 m 1)
#                                           RAQ     (d 2 m 1)
#           0x09  0b....'1..1               IAQ1-LP (d 3 m 2)
#                                           OAQ1    (d 3 m 1)
#           0x0A  0b....'1.1.               OAQ1    (d 3 m 1)
#           0x0B  0b....'1.11               OAQ1    (d 3 m 1)
#           0x0C  0b....'11..               OAQ1    (d 3 m 1)
#           0x10  0b...1'....               OAQ1    (d 3 m 1)
#           0x11  0b...1'...1               OAQ1    (d 3 m 1)
#           0x12  0b...1'..1.               OAQ1    (d 3 m 1)
#           0x13  0b...1'..11               OAQ1    (d 3 m 1)
#           0x14  0b...1'.1..               OAQ1    (d 3 m 1)
#           0x40  0b.1..'....               init    (d 1 m 2) # explore this one
#                                           IAQ1-LP (d 3 m 2)
#           0x41  0b.1..'...1               IAQ1-LP (d 3 m 2)
#                                           OAQ2    (d 1 m 2)
#           0x49  0b.1..'1..1               IAQ1-LP (d 3 m 2)
#                                           IAQ2    (d 4 m 4)
#           0x4A  0b.1..'1.1.               IAQ2    (d 4 m 4)
#           0x4B  0b.1..'1.11               IAQ2    (d 4 m 4)
#           0x4C  0b.1..'11..               IAQ2    (d 4 m 4)
#           0x4D  0b.1..'11.1               IAQ2    (d 4 m 4)
#           0x4E  0b.1..'111.               IAQ2    (d 4 m 4)
#           0x51  0b.1.1'...1               IAQ1-LP (d 3 m 2)
#           0x57  0b.1.1'.111               IAQ2    (d 4 m 4)
#           0x59  0b.1.1'1..1               IAQ2    (d 4 m 4)
#           0x5F  0b.1.1'1111               PBAQ    (d 5 m 4)
#           0x61  0b.11.'...1               PBAQ    (d 5 m 4)
#           0x97  0b1..1'.111               IAQ2    (d 4 m 4)
#           0x97  0b1..1'.111               IAQ2    (d 4 m 4)
#
#   *All* sequencers start with 0x00 0x00
#     Removing 0x00 0x00 seems to yield junk?
#   *All* sequencers end with 0x8? 0x?? (Usually 0x80 0x??.)
#     Commands after first 0x8? 0x?? seem to be ignored.
#   Null sequencer length w/ cmd=0x80 -> hangs? or at last takes a *long* time.
#     Cleaning basically does this.
#     OAQ2 almost does this.
#     Delay check in code suggests it'll take at least 5+ min?
#     `init()` / cmd = 0 does *not* appear to abort the sequencer if prev sequencer was null w/ cmd 0x80
#       TODO: verify. inconsistent w/ current understanding
#   Sequencer len is *almost* always the same as the result length.
#     Semi-common to skip first 2 octets and just read afterwards. (If accounted, matches seq length)
#     IAQ1-LP is the only one that doesn't: Seq len 28, skips first 20 result, reads 2. (Ignores last 6?)
#


import math
import struct
import time

# from typing import Iterable, Optional, Union
from collections import namedtuple
import machine
from machine import Pin


R_ProductID = 0x00  # u2 BE
# !!!!!! HOLE [0x2, 0x20)  30 octets
R_DeviceConfig = 0x20  # struct sz 6
# AKA 'General', but I haven't seen anyone use it other than for product data.
#   size seems to vary between algorithm profiles (5, 6, 7, 9, 10 all seen)
#   ZMOD4410    6, 7 octets
#   ZMOD4450    5 octets
#   ZMOD4510    9, 10 octets
#
#   unk6 used by calc-iaq-2nd (some sort of `pow`'d coefficient)
R_ProductData = 0x26
# !!!!!! HOLE ?? [0x30, 0x38)  (might not be hole if product data is longer)
# Used by cleaning, including cleaning status.
# Looks like cleaning status can be reset (officially it shouldn't/can't).
# 2 octets. Might have some fun stuff. Haven't fully examined
R_Cleaning_Unk38 = 0x38  # 2 octets?
R_TrackingNum = 0x3A  # 6 octets (is this in NVM storage?)
R_Cmd_Heater = 0x40
R_Cmd_Delay = 0x50
# 1 octet commands, 8 max (based on reg map & deduced seq cmd encoding)
R_Cmd_Measure = 0x60
# at most 32 long? maybe? known reg map says at most 43
# no known config w/ longer than 32, several w/ exactly 32
# Hangs w/ `b'\0x00' * 32 + b'0x80 0x00'`, so almost certainly max 32.
R_Cmd_Sequencer = 0x68  # u2 BE, 16 max
# !!!!!! HOLE [0x88, 0x8C)  4 octets
# only seen used in cleaning-finalise.
#   check API error; if err return err
#   read R_Cleaning_Finalise_Unk8C (1 octet ); if 0x00 -> error -10
#   read status; if status < 0 return -1 (artifact? check for high bit?)
#   unk38 <- read R_Cleaning_Unk38 (2 octets)
#   unk38[1] = 1    (mark as cleaned?)
#   nvm-write unk38 back to R_Cleaning_Unk38 (with R_NonVolatileMem_UnkB8 wrapper)
#   retry 3:
#     write 0x00 to R_Cleaning_Finalise_Unk8C
R_Cleaning_Finalise_Unk8C = 0x8C  # 1 octet?
# !!!!!! HOLE [0x8C, 0x93)  6 octets
R_Cmd = 0x93  # struct sz 1
R_Status = 0x94  # struct sz 1
R_Result = 0x97  # u2 BE, 16 max, pop by sequencer
R_DeviceErr = 0xB7
# only seen used in `_cleaning_nvm_write`:
#   1) write 0xEA, wait 50ms
#   2) write 2 octets to `R_Cleaning_Unk38`
#   3) write 0x13, wait 50ms
R_NonVolatileMem_UnkB8 = 0xB8  # 1 octet?

# These are taken from SDK headers, but aren't referred to really anywhere else.
# IDK why some bits overlap.
DeviceErr_PowerOnReset = 0x80
DeviceErr_AccessConflict = 0x40  # what is an access conflict?

Status_SequencerRunning = 0b1000_0000
Status_SleepTimerEnabled = 0b0100_0000
Status_Alarm = 0b0010_0000
Status_LastSeqStepMask = 0b0001_1111

# REMAINING MYSTERIES:
#   Status Alarm:
#       WTF is this?
#   Device Config:
#       Bunch of fields that need documentation/proper names.
#   Heater:
#       What are these units?
#   Delay:
#       Seems to be pretty well understood.
#       Problem: Predicted timings don't seem to perfectly match measured ones.
#   Measure:
#       Unknown sources: 1/2/5
#           Are 3/4 also sources?
#       WTF are the 0x3 low bits everyone has set?
#       What do the other bits do? They seem to sometimes have some sort of effect.
#   Sequencer:
#       Bunch of mystery bits that are set.
#       Bunch of bits that are never used AFAICT.
#
#   IAQ1 LP:
#       Second-to-last ADC result is *much* higher than other results, but is
#       measuring the same source as its last few peers.
#           e.g. [4696, 3884, 3438, 3441, 3444, 3448, 3453, 3460, 15971, 3466]
#                                                                 ^^^^^
#                                                                 ??? Why?
#

Cmd_Abort = 0
Cmd_Execute = 0b1000_0000
Cmd_Continuous = 0b0100_0000


class DevConfig:
    def __init__(self, raw: bytes):
        assert len(raw) == 6
        (
            self.mox_scaler,  # TODO: is `* 1000` part of constant or artifact of mox scale equation
            self.unk1,
            self.heater_scaler,
            self.heater_map_m,  # seems some values baked into
            self.heater_map_b,  # IAQ2 algo uses `* 0.5f`
        ) = struct.unpack('>BBHBB', raw)

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return (
            f"DeviceConfig:\n"
            f"\tmox_scaler:    {self.mox_scaler:2}\n"
            f"\tunk1:          0x{self.unk1:02X} ({self.unk1})\n"
            f"\theater_scaler: {self.heater_scaler}\n"
            f"\theater_m:      {self.heater_map_m}\n"
            f"\theater_b:      {self.heater_map_b}"
        )

    # PRECONDITION: `x` was reinterpreted as signed
    def heater_value(self, x: int):
        # y = -(self.heater_scaler * ((self.heater_map_m + 640.0) * (self.heater_map_b + x) - 512000.0)) / 12288000.0
        x2 = self.heater_map_b + x
        y = (self.heater_map_m + 640.0) * x2
        y /= -12288000.0
        y += 1 / 24
        y *= self.heater_scaler
        # print(x, y, self._heater_value_official(x))
        assert 0 <= y <= 4096
        y = int(y) & 0xFFFF
        assert y == self.heater_value_official(x)
        return y

    # computed based of IAQ2 impl, first result (from \x23)
    # TODO: are these constants dependent on the delay/setup?
    def heater_resistence(self, src_1: int):
        assert 0 <= src_1 <= 0xFFFF
        return src_1 * 0.0014579 + 15.90909

    # not exactly sure what this does
    def iaq2_heater_temp_est(self, src_1: int):
        assert 0 <= src_1 <= 0xFFFF
        x = self.heater_resistence(src_1)
        y = (x * 655.36) / self.heater_scaler
        y = y / 0.00322 - 310.559  # map to celsius?
        y += self.heater_map_b / 2 - 5.76
        return y

    def heater_value_official(self, x: int):
        return int(self._heater_value_official(x)) & 0xFFFF

    def _heater_value_official(self, x: int):
        y = (
            -(
                self.heater_scaler
                * ((self.heater_map_m + 640.0) * (self.heater_map_b + x) - 512000.0)
            )
            / 12288000.0
        )
        assert 0 <= y <= 4096
        return y


def status_pprint(status: int):
    assert 0 <= status <= 0xFF
    FLAGS = {
        "RUNNING": Status_SequencerRunning,
        "SLEEP_ALARM_ENABLED": Status_SleepTimerEnabled,
        "ALARM": Status_Alarm,
    }

    return f"Last Step: {status & Status_LastSeqStepMask:2} {', '.join(k for k, mask in FLAGS.items() if status & mask)}"


def delay_pprint(delay: int):
    assert 0 <= delay <= 0xFFFF
    return f'{round(delay * 0.2, 2):7.1f}ms'


def measure_pprint(m: int):
    assert 0 <= m <= 0xFF
    rest = m & 0x1F
    source = m >> 5
    unusual = '' if rest == 0x03 else f'(?? Raw = 0x{m:02X})'
    if source == 0b000:
        name = 'ADC'
    elif source == 0b001:
        name = 'H-RES?'  # heater resistence?
    elif source == 0b010:
        name = '0x0?'  # constants?
    elif source == 0b011:
        name = '0xFFFF?'  # constants?
    elif source == 0b110:
        name = 'MOX MIN'
    elif source == 0b111:
        name = 'MOX MAX'
    else:
        name = f'UNK {source}'
    return f'{name:7}{unusual}'


class SeqCmd:
    @staticmethod
    def from_bytes(xs: "Union[CmdCfg, bytes]"):
        if type(xs) == bytes:
            return [SeqCmd(int.from_bytes(x)) for x in chunk(xs, 2)]
        else:
            return SeqCmd.from_bytes(xs.sequencer)

    @staticmethod
    def pprint(xs: "Union[Iterable[SeqCmd], CmdCfg, bytes]"):
        if type(xs) == bytes or type(xs) == CmdCfg:
            xs = SeqCmd.from_bytes(xs)
        for x in xs:
            bs = x.raw.to_bytes(2)
            print(f"b'\\x{bs[0]:02X}\\x{bs[1]:02X}' # {x}")

    MASK_STOP = 0b1000_0000_0000_0000
    MASK_M = 0b0000_0001_1100_0000
    MASK_D = 0b0000_0000_0011_1000
    MASK_H = 0b0000_0000_0000_0111
    MASK_UNKNOWN = 0xFFFF & ~(MASK_STOP | MASK_H | MASK_D | MASK_M)
    MASK_UNKNOWN_KNOWN = 0b0000_1110_0000_0000
    MASK_UNKNOWN_UNKNOWN = 0b0111_0000_0000_0000
    assert (MASK_UNKNOWN_KNOWN & MASK_UNKNOWN_UNKNOWN) == 0
    assert (MASK_UNKNOWN_KNOWN | MASK_UNKNOWN_UNKNOWN) == MASK_UNKNOWN

    def __init__(self, raw: int):
        assert 0 <= raw < 0xFFFF
        self.raw = raw
        self.stop = (raw & self.MASK_STOP) == self.MASK_STOP
        self.h_idx = (raw & self.MASK_H) >> 0
        self.d_idx = (raw & self.MASK_D) >> 3
        self.m_idx = (raw & self.MASK_M) >> 6
        self.unknown = (raw & self.MASK_UNKNOWN) >> 9

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        unk = f' unk=0x{self.unknown:02X}' if self.unknown else ''
        stop = ' STOP' if self.stop else ''
        return f"{self.h_idx} {self.d_idx} {self.m_idx}{unk}{stop}"


class CmdCfg:
    def __init__(
        self,
        *,
        command=Cmd_Execute,
        result=R_Result,
        result_len=None,
        heater,
        delay,
        measure,
        sequencer,
    ):
        result_len = (
            result_len
            if result_len is not None
            else (len(sequencer) - (result - R_Result))
        )
        assert command <= 0xFF
        assert R_Result <= result <= 0xFF
        assert result_len <= 32 - (result - R_Result)
        assert len(heater) <= 8
        assert len(delay) <= 16
        assert len(measure) <= 8
        assert len(sequencer) <= 32
        self.command = command
        self.result = result
        self.result_len = result_len
        # fixup b/c python interpets as unsigned const
        self.heater = [struct.unpack('>h', x.to_bytes(2))[0] for x in heater]
        self.delay = delay
        self.measure = measure
        self.sequencer = sequencer
        assert (result - R_Result) + result_len <= len(
            sequencer
        ), "hypothesis violated: seq-len upper bounds result-len"

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return self.pprint()

    def pprint(self, dev_config: "Optional[DevConfig]" = None):
        delay = [int.from_bytes(x) for x in chunk(self.delay, 2)]

        def pprint(cmd: SeqCmd):
            h = self.heater[cmd.h_idx]
            unk = f"   seq-unk=0x{cmd.unknown:02X}" if cmd.unknown else ""
            stop = f"   STOP" if cmd.stop else ""
            heat = f'0x{h & 0xFFFF:04X} ({struct.unpack('>h', h.to_bytes(2))[0]:#6})'
            if dev_config is not None:
                heat += f' {dev_config.heater_value(h):4}'
            return f"heat {heat}   {delay_pprint(delay[cmd.d_idx])}    {measure_pprint(self.measure[cmd.m_idx])}{unk}{stop}"

        return '\n'.join(pprint(cmd) for cmd in SeqCmd.from_bytes(self))


def validate_all(fn, suppress=False):
    okay = True
    for name, cfg in zmod4410_CONFIGS.items():
        print(name)
        try:
            fn(cfg)
        except AssertionError as e:
            okay = False
            if not suppress:
                print(f"{name}\t\t{e}")
    return okay


def validate(suppress=False):
    return validate_all(check, suppress=suppress)


def chunk(xs, n: int):
    assert 0 < n
    assert len(xs) % n == 0
    return [xs[i : i + n] for i in range(0, len(xs), n)]


def check(cfg: CmdCfg):
    assert len(cfg.heater) <= 8, "too many heater"
    assert len(cfg.delay) <= 16, "too many delay"  # *2 b/c stored flat as octets
    assert len(cfg.delay) % 2 == 0, "odd length delay"
    assert len(cfg.measure) <= 8, "too many measure"
    assert len(cfg.sequencer) <= 32, "too many seq"
    assert len(cfg.sequencer) % 2 == 0, "odd length seq"
    seq = SeqCmd.from_bytes(cfg.sequencer)
    assert seq[0].raw == 0, "seq doesn't begin with 0x00 0x00"
    seq_stop_idx = None
    for seq_stop_idx, cmd in enumerate(seq):
        if cmd.stop:
            break
    assert (
        seq_stop_idx == len(seq) - 1
    ), f"seq stop idx {seq_stop_idx} isn't at end {len(seq) - 1}"
    h_used = set()
    d_used = set()
    m_used = set()
    for i, cmd in enumerate(seq):
        assert not (
            cmd.raw & SeqCmd.MASK_UNKNOWN_UNKNOWN
        ), f"seq[{i}] has unknown unknown 0x{cmd.raw & SeqCmd.MASK_UNKNOWN_UNKNOWN:04x}"
        if cmd.unknown:
            print(f"\tseq[{i}] unknown bits 0x{cmd.unknown:02x}")
        assert cmd.h_idx < len(
            cfg.heater
        ), f"heater OOR {cmd.h_idx} of {len(cfg.heater)}"
        assert cmd.d_idx < (
            len(cfg.delay) // 2
        ), f"delay OOR {cmd.d_idx} of {len(cfg.delay) // 2}"
        assert cmd.m_idx < len(
            cfg.measure
        ), f"measure OOR {cmd.m_idx} of {len(cfg.measure)}"
        h_used.add(cmd.h_idx)
        d_used.add(cmd.d_idx)
        m_used.add(cmd.m_idx)
    for i in range(len(cfg.heater)):
        assert i in h_used, f"heater not used {i}"
    for i in range(len(cfg.delay) // 2):
        assert i in d_used, f"delay not used {i}"
    for i in range(len(cfg.measure)):
        assert i in m_used, f"measure not used {i}"


zmod4410_init = CmdCfg(
    command=Cmd_Execute,
    result=R_Result,
    result_len=4,
    heater=[0x0050],
    delay=b'\x00\x28',
    measure=b'\xc3\xe3',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x80\x40'  # 0 0 1 STOP
    ),
)

zmod4410_measure_iaq1 = CmdCfg(
    command=Cmd_Execute | Cmd_Continuous,
    result=R_Result,
    result_len=4,  # only last 2 are used
    heater=[0xFDA8],
    delay=b'\x20\x04' b'\x20\x04',
    measure=b'\x03',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x80\x08'  # 0 1 0 STOP
    ),
)

zmod4410_measure_iaq1_lp = CmdCfg(
    command=Cmd_Execute,
    result=R_Result,
    result_len=20,  # only last 2 are used
    heater=[0x0050, 0xFDA8],
    delay=b'\x00\xcd' b'\x01\x9a' b'\x03\x34',
    measure=b'\x23\x03',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x0a\x41'  # 1 0 1 unk=0x05
        b'\x0a\x41'  # 1 0 1 unk=0x05
        b'\x00\x41'  # 1 0 1
        b'\x00\x41'  # 1 0 1
        b'\x00\x49'  # 1 1 1
        b'\x00\x49'  # 1 1 1
        b'\x00\x51'  # 1 2 1
        b'\x00\x09'  # 1 1 0
        b'\x00\x49'  # 1 1 1
        b'\x00\x40'  # 0 0 1
        b'\x00\x40'  # 0 0 1
        b'\x00\x40'  # 0 0 1        <- weird, has much higher value than others
        b'\x80\x40'  # 0 0 1 STOP
    ),
)

zmod4410_measure_iaq2 = CmdCfg(
    command=Cmd_Execute,
    result=R_Result,
    result_len=32,
    heater=[0x0050, 0xFF38, 0xFED4, 0xFE70, 0xFE0C, 0xFDA8, 0xFD44, 0xFCE0],
    delay=b'\x00\x52' b'\x02\x67' b'\x00\xcd' b'\x03\x34',
    measure=b'\x23\x03\xa3\x43',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x06\x49'  # 1 1 1 unk=0x03
        b'\x06\x4a'  # 2 1 1 unk=0x03
        b'\x06\x4b'  # 3 1 1 unk=0x03
        b'\x06\x4c'  # 4 1 1 unk=0x03
        b'\x06\x4d'  # 5 1 1 unk=0x03
        b'\x06\x4e'  # 6 1 1 unk=0x03
        b'\x06\x97'  # 7 2 2 unk=0x03
        b'\x06\xd7'  # 7 2 3 unk=0x03
        b'\x06\x57'  # 7 2 1 unk=0x03
        b'\x06\x4e'  # 6 1 1 unk=0x03
        b'\x06\x4d'  # 5 1 1 unk=0x03
        b'\x06\x4c'  # 4 1 1 unk=0x03
        b'\x06\x4b'  # 3 1 1 unk=0x03
        b'\x06\x4a'  # 2 1 1 unk=0x03
        b'\x86\x59'  # 1 3 1 unk=0x03 STOP
    ),
)

# almost the same config as IAQ2
zmod4410_measure_pbaq = CmdCfg(
    command=Cmd_Execute,
    result=R_Result,
    result_len=32,
    heater=[0x0050, 0xFF38, 0xFED4, 0xFE70, 0xFE0C, 0xFDA8, 0xFD44, 0xFCE0],
    # different and slightly longer delay config vs. IAQ2
    delay=b'\x00\x14' b'\x00\x9c' b'\x00\x31' b'\x00\x39' b'\x00\xcd',
    measure=b'\x23\x03\xa3\x43',
    # almost identical to IAQ2 -> PBAQ: 0x06 0x57 -> 0x06 0x5F,  0x86 0x59 -> 0x86 0x61
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x06\x49'  # 1 1 1 unk=0x03
        b'\x06\x4a'  # 2 1 1 unk=0x03
        b'\x06\x4b'  # 3 1 1 unk=0x03
        b'\x06\x4c'  # 4 1 1 unk=0x03
        b'\x06\x4d'  # 5 1 1 unk=0x03
        b'\x06\x4e'  # 6 1 1 unk=0x03
        b'\x06\x97'  # 7 2 2 unk=0x03
        b'\x06\xd7'  # 7 2 3 unk=0x03
        b'\x06\x5f'  # 7 3 1 unk=0x03
        b'\x06\x4e'  # 6 1 1 unk=0x03
        b'\x06\x4d'  # 5 1 1 unk=0x03
        b'\x06\x4c'  # 4 1 1 unk=0x03
        b'\x06\x4b'  # 3 1 1 unk=0x03
        b'\x06\x4a'  # 2 1 1 unk=0x03
        b'\x86\x61'  # 1 4 1 unk=0x03 STOP
    ),
)

zmod4410_measure_no2_o3 = CmdCfg(
    command=Cmd_Execute,
    result=R_Result,
    result_len=32,
    heater=[0x0050, 0xFF06, 0xFEA2, 0xFE3E],
    # different and slightly longer delay config vs. IAQ2
    delay=b'\x00\x10' b'\x00\x52' b'\x3f\x66' b'\x00\x42',
    measure=b'\x23\x03',
    # almost identical to IAQ2: IAQ2 0x06 0x57 -> PBAQ 0x06 0x5F\x86 0x59 -> 0x86 0x61
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x02\x41'  # 1 0 1 unk=0x01
        b'\x00\x41'  # 1 0 1
        b'\x00\x41'  # 1 0 1
        b'\x00\x49'  # 1 1 1
        b'\x00\x50'  # 0 2 1
        b'\x02\x42'  # 2 0 1 unk=0x01
        b'\x00\x42'  # 2 0 1
        b'\x00\x42'  # 2 0 1
        b'\x00\x4a'  # 2 1 1
        b'\x00\x50'  # 0 2 1
        b'\x02\x43'  # 3 0 1 unk=0x01
        b'\x00\x43'  # 3 0 1
        b'\x00\x43'  # 3 0 1
        b'\x00\x43'  # 3 0 1
        b'\x80\x5b'  # 3 3 1 STOP
    ),
)

zmod4410_measure_oaq1 = CmdCfg(
    command=Cmd_Execute,
    result=R_Result,
    result_len=30,
    heater=[0xFE48, 0xFE16, 0xFDE4, 0xFDB2, 0xFD80],
    delay=b'\x20\x05' b'\xa0\x18' b'\xc0\x1c',
    measure=b'\x03',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x00\x08'  # 0 1 0
        b'\x00\x10'  # 0 2 0
        b'\x00\x01'  # 1 0 0
        b'\x00\x09'  # 1 1 0
        b'\x00\x11'  # 1 2 0
        b'\x00\x02'  # 2 0 0
        b'\x00\x0a'  # 2 1 0
        b'\x00\x12'  # 2 2 0
        b'\x00\x03'  # 3 0 0
        b'\x00\x0b'  # 3 1 0
        b'\x00\x13'  # 3 2 0
        b'\x00\x04'  # 4 0 0
        b'\x00\x0c'  # 4 1 0
        b'\x80\x14'  # 4 2 0 STOP
    ),
)

zmod4410_measure_oaq2 = CmdCfg(
    command=Cmd_Execute,
    result=R_Result,
    result_len=18,
    heater=[0x0050, 0xFE70],
    delay=b'\x00\x10',  # insanely short time??
    measure=b'\x23\x03',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x06\x41'  # 1 0 1 unk=0x03
        b'\x06\x41'  # 1 0 1 unk=0x03
        b'\x06\x41'  # 1 0 1 unk=0x03
        b'\x06\x41'  # 1 0 1 unk=0x03
        b'\x06\x41'  # 1 0 1 unk=0x03
        b'\x06\x41'  # 1 0 1 unk=0x03
        b'\x06\x41'  # 1 0 1 unk=0x03
        b'\x86\x41'  # 1 0 1 unk=0x03 STOP
    ),
)

zmod4410_measure_odor = CmdCfg(
    command=Cmd_Execute | Cmd_Continuous,
    result=R_Result,
    result_len=4,
    heater=[0xFDA8],
    delay=b'\x20\x04' b'\x20\x04',  # only difference w/ raq: slightly diff delay
    measure=b'\x03',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x80\x08'  # 0 1 0 STOP
    ),
)

zmod4410_measure_raq = CmdCfg(
    command=Cmd_Execute | Cmd_Continuous,
    result=R_Result,
    result_len=4,
    heater=[0xFDA8],
    delay=b'\x20\x04' b'\x40\x09',
    measure=b'\x03',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x80\x08'  # 0 1 0 STOP
    ),
)

# Hidden in lib clean binary object.
# Apparently this is rough on the device, and shouldn't be done more than 1 or 2 times?
zmod4410_cleaning_DONT_USE = CmdCfg(
    command=Cmd_Execute,
    result=R_Result,
    result_len=2,
    heater=[0xFBB4],
    delay=b'\xc0\x1c',
    measure=b'\x03',
    sequencer=b'\x00' * 18 + b'\x80\x00',
)


zmod4410_measure_experimental = CmdCfg(
    command=Cmd_Execute,
    result=R_Result,
    result_len=4,
    heater=[0x0050],  # very low / no power for heater
    delay=b'\x20\x04' b'\x20\x04',
    measure=b'\x03',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x80\x08'  # 0 1 0 STOP
    ),
)

zmod4410_toy_measure_sources = CmdCfg(
    heater=[0x0050],
    delay=b'\x00\x28',
    measure=b'\x03\x13\x23\x43\x63\x83\xa3\xc3',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x00\x40'  # 0 0 1
        b'\x00\x80'  # 0 0 2
        b'\x00\xc0'  # 0 0 3
        b'\x01\x00'  # 0 0 4
        b'\x01\x40'  # 0 0 5
        b'\x01\x80'  # 0 0 6
        b'\x81\xc0'  # 0 0 7 STOP
    ),
)

zmod4410_toy_measure_flags = CmdCfg(
    heater=[0x0050],
    delay=b'\x00\x28',
    measure=b'\xe0\xe1\xe2\xe3\xe4\xe5\xe6\xe7',
    sequencer=(
        b'\x00\x00'  # 0 0 0
        b'\x00\x40'  # 0 0 1
        b'\x00\x80'  # 0 0 2
        b'\x00\xc0'  # 0 0 3
        b'\x01\x00'  # 0 0 4
        b'\x01\x40'  # 0 0 5
        b'\x01\x80'  # 0 0 6
        b'\x81\xc0'  # 0 0 7 STOP
    ),
)

# based off IAQ1-LP, which seems most similar to what we want
# zmod4410_measure_custom = CmdCfg(
#     command=Cmd_80,
#     result=R_Result,
#     result_len=18,
#     heater=[0x0050, 0xFDA8],
#     delay=b'\x00\xCD' b'\x01\x9A' b'\x03\x34',
#     measure=b'\x23\x03',
#     sequencer=(
#         b'\x00\x00'  # 0 0 0
#         b'\x8A\x41'  # 1 0 1 unk=0x05
#     ),
# )

zmod4410_CONFIGS = {
    "zmod4410_init": zmod4410_init,
    "zmod4410_measure_raq": zmod4410_measure_raq,
    "zmod4410_measure_odor": zmod4410_measure_odor,
    "zmod4410_measure_iaq1": zmod4410_measure_iaq1,
    "zmod4410_measure_iaq1_lp": zmod4410_measure_iaq1_lp,
    "zmod4410_measure_iaq2": zmod4410_measure_iaq2,
    "zmod4410_measure_pbaq": zmod4410_measure_pbaq,
    "zmod4410_measure_no2_o3": zmod4410_measure_no2_o3,
    "zmod4410_cleaning_DONT_USE": zmod4410_cleaning_DONT_USE,
}

print(f"Validating configs: {validate()}")


class Device:
    ADDRESS = 0x32  # aka 50
    # Every N ms dump the status
    STATUS_DUMP_DELAY_MS = 1000
    WAIT_GRANULARITY_MS = 100

    def __init__(self, i2c):
        self.i2c = i2c
        self.mox_lo = 0
        self.mox_hi = 0xFFFF
        self.config: "Optional[DevConfig]" = None

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return (
            f"{'<no config>' if self.config is None else repr(self.config)}\n"
            f"Mox-Bounds:   [{self.mox_lo}, {self.mox_hi}]"
        )

    def read(self, reg: int, len: int) -> bytes:
        # print(f"READ: 0x{reg:02X} len={len}")
        return self.i2c.readfrom_mem(self.ADDRESS, reg, len)

    def write(self, reg: int, dat: bytes) -> None:
        # dat_str = [f"0x{x:02X}" for x in dat]
        # print(f"WRITE: 0x{reg:02X} [ {', '.join(dat_str)} ]")
        self.i2c.writeto_mem(self.ADDRESS, reg, dat)

    def exists(self) -> bool:
        try:
            self.read(R_ProductID, 2)
            return True
        except OSError as e:
            print(e)
            return False

    def setup(self) -> bool:
        print("SETUP - WAIT FOR IDLE")
        while (status := self.read(R_Status, 1)[0]) & Status_SequencerRunning:
            print(f"Status - {status_pprint(status)}")
            # self.write(R_Cmd_Sequencer, b'\x80\x00')
            self.write(R_Cmd, b'\x00')
            machine.lightsleep(300)

        print(f"product 0x{int.from_bytes(self.read(R_ProductID, 2)):04x}")
        print(
            'serial/tracking no: 0x'
            + ''.join(f'{x:02x}' for x in self.read(R_TrackingNum, 6))
        )
        print(
            'production data: '
            + ' '.join(f'0x{x:02x}' for x in self.read(R_ProductData, 9))
        )
        self.config = DevConfig(self.read(R_DeviceConfig, 6))
        print(self.config)

        while (dev_err := self.read(R_DeviceErr, 1)[0]) != 0:  # read dev-err to clear
            print(f"pre init dev err: {dev_err}")
            machine.lightsleep(300)

        while True:
            self.prepare(zmod4410_init)
            data = self.execute(zmod4410_init)
            if data is not None:
                break
            # HACK: The device can spontaneously report a Power-On-Reset.
            #       It seems legit? Config parameters all get cleared to zero, but
            #       sometimes it 'reboots' into a sequencer loop b/c the seq cmds
            #       are all 0 (no stop bit).
            #       Handle this by assuming a failure is due to a POR.
            #           1) issue an abort by clearing the run bit on command
            #              (this doesn't actually seem to work. :/)
            self.write(R_Cmd, b'\x00')  # stop stop stop
            machine.lightsleep(300)

        self.mox_lo, self.mox_hi = data
        print(f"mox_lo = {self.mox_lo}, mox_hi = {self.mox_hi}")
        assert self.mox_lo < self.mox_hi

        return True

    def prepare(
        self, cmd: CmdCfg, heater=None, delay=None, measure=None, sequencer=None
    ):
        cmd = CmdCfg(
            command=cmd.command,
            result=cmd.result,
            result_len=cmd.result_len,
            heater=heater if heater is not None else cmd.heater,
            delay=delay if delay is not None else cmd.delay,
            measure=measure if measure is not None else cmd.measure,
            sequencer=sequencer if sequencer is not None else cmd.sequencer,
        )
        check(cmd)
        heater = b''.join(self.config.heater_value(h).to_bytes(2) for h in cmd.heater)
        assert len(heater) <= 16, "heater too long"
        dev_err = self.read(R_DeviceErr, 1)[0]
        if dev_err != 0:
            print(f"PRE-PREP DEVICE ERR: 0x{dev_err:02x}")
            print("^^^^^^^^^^^^^^^^^^^^^^ !!!!!")

        self.write(R_Cmd_Heater, heater + b'\x00' * (16 - len(heater)))
        self.write(R_Cmd_Delay, cmd.delay + b'\x00' * (16 - len(cmd.delay)))
        self.write(R_Cmd_Measure, cmd.measure + b'\x00' * (8 - len(cmd.measure)))
        self.write(
            R_Cmd_Sequencer, cmd.sequencer + b'\x00' * max((32 - len(cmd.sequencer)), 0)
        )
        dev_err = self.read(R_DeviceErr, 1)[0]
        if dev_err != 0:
            print(f"POST-PREP DEVICE ERR: 0x{dev_err:02x}")
            print("^^^^^^^^^^^^^^^^^^^^^^ !!!!!")
        machine.lightsleep(50)

    def execute(self, cmd: CmdCfg, command=None, result=None, result_len=None):
        command = command if command is not None else cmd.command
        result = result if result is not None else cmd.result
        result_len = result_len if result_len is not None else cmd.result_len
        self.write(R_Cmd, command.to_bytes(1))

        t_bgn = time.ticks_ms()
        t_bgn_status_dump = t_bgn
        while True:
            try:
                status = self.read(R_Status, 1)[0]
            except OSError:
                status = Status_SequencerRunning
            if not (status & Status_SequencerRunning):
                break

            machine.lightsleep(self.WAIT_GRANULARITY_MS)
            # interested in seeing if anyone ever sets `ALARM`
            if (
                self.STATUS_DUMP_DELAY_MS
                < time.ticks_diff(time.ticks_ms(), t_bgn_status_dump)
                or status & Status_Alarm
            ):
                t_bgn_status_dump = time.ticks_ms()
                print(f"Status - {status_pprint(status)}")
                if (dev_err := self.read(R_DeviceErr, 1)[0]) != 0:
                    print(f"Dev Err - {dev_err}")

        print(f'waited ~{time.ticks_diff(time.ticks_ms(), t_bgn)}ms')

        raw = self.read(result, result_len)
        # all known instances parse it as a list of u2 BE
        data = list(struct.unpack('>' + 'H' * (len(raw) // 2), raw))

        if (dev_err := self.read(R_DeviceErr, 1)[0]) != 0:
            print(f"POST DEVICE ERR: 0x{dev_err:02x}")
            print("^^^^^^^^^^^^^^^^^^^^^^ !!!!!")
            print(f"Returned data: {data}")
            return None

        return data

    def rmox(self, adc):
        assert self.mox_lo < self.mox_hi
        # multiple rmox impls in SDK (depends on algo)
        # diff impl have diff values for out of bounds; doesn't seem to match values near limit either
        # if adc < mox_lr:
        #     return 10e2
        # if mox_er <= adc:
        #     return 10e9
        # return (dev_config.mox_scaler  * 1000) * (adc - mox_lr) / (mox_er - adc)
        adc = max(adc, self.mox_lo)
        adc = min(adc, self.mox_hi - 1)
        mox = (
            (self.config.mox_scaler * 1000) * (adc - self.mox_lo) / (self.mox_hi - adc)
        )
        return max(min(mox, 10e9), 1)

    def prep_exec(self, cfg, command=None, result=None, result_len=None, **xs):
        self.prepare(cfg, **xs)
        return self.execute(cfg, command=command, result_len=result_len)

    def raq(self, **xs):
        raw = self.prep_exec(zmod4410_measure_raq, **xs)
        if raw is None:
            return None
        return raw[-1:], raw

    def odor(self, **xs):
        raw = self.prep_exec(zmod4410_measure_odor, **xs)
        if raw is None:
            return None
        return raw[-1:], raw

    def iaq1(self, **xs):  # -> list[float | Any]:
        raw = self.prep_exec(zmod4410_measure_iaq1, **xs)
        if raw is None:
            return None
        return raw[-1:], raw

    def iaq1_lp(self, **xs):  # -> list[float | Any]:
        raw = self.prep_exec(zmod4410_measure_iaq1_lp, **xs)
        if raw is None:
            return None
        # 0 uses non std adc source (0x23)
        # 7, 8 use non std adc sources (0xA3, 0x43)
        return raw[-1:], raw

    def iaq2(self, **xs):  # -> list[float | Any]:
        raw = self.prep_exec(zmod4410_measure_iaq2, **xs)
        if raw is None:
            return None
        # 0 uses non std adc source (0x23)
        # 7, 8 use non std adc sources (0xA3, 0x43)
        return raw[1:7] + raw[9:], raw, self.config.iaq2_heater_temp_est(raw[0])

    def pbaq(self, **xs):  # -> list[float | Any]:
        raw = self.prep_exec(zmod4410_measure_pbaq, **xs)
        if raw is None:
            return None
        # TODO: rev-eng PBAQ, ensure it is the same as IAQ2 WRT to how it picks ADC samples
        return raw, raw

    def no2_o3(self, **xs):  # -> list[float | Any]:
        raw = self.prep_exec(zmod4410_measure_no2_o3, **xs)
        if raw is None:
            return None
        # TODO: rev-eng PBAQ, ensure it is the same as IAQ2 WRT to how it picks ADC samples
        # weird, IDK why it doesn't use 0, 7, 8.
        # timing maybe? perhaps these are filled with duds/discarded samples?
        return raw, raw

    def experimental(self, **xs):
        raw = self.prep_exec(zmod4410_measure_experimental, **xs)
        if raw is None:
            return None
        return raw[-1:], raw


def probe(scl: int, sda: int):
    bus = 1 if sda & 2 else 0
    i2c = machine.I2C(bus, scl=Pin(scl), sda=Pin(sda), freq=100_000)
    dev = Device(i2c)
    if not dev.exists():
        return None
    dev.setup()
    return dev


def forever_poll(dev: Device, cb, **xs):
    while True:
        try:
            r = cb(**xs)
            if r is not None:
                print(r)
                print(r[0])
                print(list(map(dev.rmox, r[0])))
        except OSError as e:
            print(e)


# dev = probe(3, 2)
# dev = probe(5, 4)
# dev = probe(9, 8)
# dev = probe(21, 20)
# assert dev is not None
