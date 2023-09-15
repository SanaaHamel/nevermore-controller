#include "ens16x.hpp"
#include "config.hpp"
#include "sdk/ble_data_types.hpp"
#include "sensors.hpp"
#include "sensors/environmental_i2c.hpp"
#include "utility/i2c_device.hpp"
#include "utility/numeric_suffixes.hpp"
#include <cstdint>
#include <optional>
#include <utility>

using namespace std;
using namespace BLE;

namespace nevermore::sensors {

static_assert(
        I2C_BAUD_RATE <= 1'000'000, "`config.hpp`'s `I2C_BAUD_RATE` is too high for ENS16x (max 1 M/s)");

namespace {

constexpr array ADDRESSES{0x52_u8, 0x53_u8};

// `DataAqiScioSense` \in [0, 500], 100 = avg over last 24h
// `DataAqiUBI`       \in [1, 5]
enum class Reg : uint8_t {
    PartID = 0x00u,            // 2 octets, R
    OpMode = 0x10u,            // 1 octet, RW
    Config = 0x11u,            // 1 octet, RW
    Command = 0x12u,           // 1 octet, RA
    TemperatureIn = 0x13u,     // 2 octets, RW
    RelHumidityIn = 0x15u,     // 2 octets, RW
    Reserved0 = 0x17u,         // 16 octets
    DeviceStatus = 0x20u,      // 1 octets, R
    DataAqiUBI = 0x21u,        // 1 octets, R
    DataTVOC = 0x22u,          // 2 octets, R
    DataECO2 = 0x24u,          // 2 octets, R
    DataAqiScioSense = 0x26u,  // 2 octets, R; documented in ENS161, but looks like its available on ENS160?
    SensorBaseline0 = 0x28u,   // 2 octets, R; undocumented, seen in reference driver
    SensorBaseline1 = 0x2Au,   // 2 octets, R; undocumented, seen in reference driver
    SensorBaseline2 = 0x2Cu,   // 2 octets, R; undocumented, seen in reference driver
    SensorBaseline3 = 0x2Eu,   // 2 octets, R; undocumented, seen in reference driver
    DataTemperature = 0x30u,   // 2 octets, R
    DataRelHumidity = 0x32u,   // 2 octets, R
    Reserved4 = 0x34u,         // 4 octets
    DataChecksum = 0x38u,      // 1 octet, R; AKA `MISR`
    GprWrite0 = 0x40u,         // 8 octets, RW
    GprRead0 = 0x48u,          // 8 octets, R
    GprRead4 = GprRead0 + 4,   // subset of `GprRead0`
};

enum class OpMode : uint8_t {
    DeepSleep = 0x00u,
    Idle = 0x01u,
    Operational = 0x02u,      // 1 hz sampling
    LowPowerGas = 0x03u,      // ENS161 documented only, not tested on ENS160, 1/ 60 hz sampling
    UltaLowPowerGas = 0x04u,  // ENS161 documented only, not tested on ENS160, 1/300 hz sampling
    Reset = 0xF0u,
};

// Commands can only be executed in Idle mode
enum class Cmd : uint8_t {
    NoOp = 0x00u,
    GetAppVersion = 0x0Eu,
    ClearGPR = 0xCCu,
    SetSeq = 0xC2u,  // undocumented, noted in reference driver for ENS160
};

// enum values must match official part IDs and must be a classed to `uint16_t`
enum class Kind : uint16_t { ENS160 = 0x0160u, ENS161 = 0x0161u };

struct [[gnu::packed]] AppVersion {
    uint8_t major, minor, revision;
};

struct [[gnu::packed]] Status {
    enum { Normal = 0, WarmUp = 1, StartUp = 2, Invalid = 3 };

    uint8_t new_gpr : 1;
    uint8_t new_data : 1;
    uint8_t validity : 2;  // 0 = normal, 1 = warm-up, 2 = startup, 3 = invalid
    uint8_t _reserved : 2;
    uint8_t error : 1;  // 0 = normal, 1 = error detected
    // (sic) "High indicates that an OPMODE is running"
    // No idea what that means. Used to think it meant an op-mode change is in
    // progress, but their reference driver doesn't care about that and I've seen
    // `statas=1` during the measurement loop. Maybe it means measure-in-progress?
    uint8_t statas : 1;
};
static_assert(sizeof(Status) == sizeof(uint8_t));

struct [[gnu::packed]] Compensation {
    uint16_t temperature;
    uint16_t humidity;
};

struct MISR {
    uint8_t expected = 0;  // mirror of DATA_MISR (0 is hardware default)

    template <typename A>
    void update(A const& value) {
        auto const* ptr = reinterpret_cast<uint8_t const*>(&value);
        auto const* end = ptr + sizeof(A);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        for (; ptr < end; ++ptr)            // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            expected = apply(expected, *ptr);
    }

private:
    static uint8_t apply(uint8_t miso, uint8_t data) {
        constexpr auto POLY = 0x1D;  // 0b00011101 = x^8+x^4+x^3+x^2+x^0 (x^8 is implicit)
        uint8_t misr_xor = ((miso << 1) ^ data) & 0xFF;
        if ((miso & 0x80) == 0) return misr_xor;

        return misr_xor ^ POLY;
    }
};

struct ENS16xSensor final : SensorPeriodicEnvI2C<Reg, "ENS16x"> {
    using SensorPeriodicEnvI2C::SensorPeriodicEnvI2C;

    Kind kind = Kind::ENS160;  // assume we're the simpler one until we test
    MISR misr;

    bool setup() {
        if (!mode(OpMode::Reset, true)) return false;
        if (!mode(OpMode::Idle)) return false;

        auto kind = read_kind();
        if (!kind) return false;
        this->kind = *kind;

        auto version = read_app_version();
        if (!version) return false;

        i2c.log("kind: %u, version: %d.%d.%d", to_underlying(this->kind), version->major, version->minor,
                version->revision);

        return mode(OpMode::Operational);
    }

    void read() override {
        Compensation compensation{
                .temperature = uint16_t(max(0., (side.compensation_humidity() + 273.15) * 64)),
                .humidity = uint16_t(side.compensation_humidity() * 512),
        };
        if (!i2c.write(Reg::TemperatureIn, compensation)) return;

        // Data* calls must be read via `read_crc` to update checksum
        auto status = read_data_verified<Status>(Reg::DeviceStatus);
        if (!status) {
            i2c.log_error("failed to fetch status");
            return;
        }
        if (!status->new_data) return;  // nothing to read

        if (status->validity == Status::Invalid) {
            i2c.log_error("invalid status for read");
            return;
        }

        // Serendipitously, this sensor also offers an arbitrary AQI value in the range of [0, 500]
        // FIXME: This is probably not suitable. 100 marks a 24h average, not an absolute value...
        auto aqi_level = read_data_verified<uint16_t>(Reg::DataAqiScioSense);
        if (!aqi_level) {
            i2c.log_error("failed to read AQI-ScioSense");
            return;
        }

        side.set(VOCIndex(clamp<uint16_t>(*aqi_level, 1, 500)));
    }

    // NB:  Spec says switching to an operational mode (i.e. {0x2, 0x3, 0x4})
    //      *must* be done from idle mode. This is not enforced/checked by this
    //      function.
    bool mode(OpMode mode, bool quiet = false) {  // NOLINT(readability-make-member-function-const)
        if (!i2c.write(Reg::OpMode, mode)) {
            if (!quiet) {
                i2c.log_error("failed to change to mode=%02x", uint8_t(mode));
            }

            return false;
        }
        // HACK:  Give the device a moment to switch.
        //        If you don't, the checksum read can fail, and without that
        //        you can't read the status.
        task_delay(ENS16x_POWER_ON_DELAY);

        // Reset does *NOT* clear/set MISR. Have to query the current state from the device.
        // Might as well do this now when we're changing modes.
        auto curr_misr = i2c.read<uint8_t>(Reg::DataChecksum);
        if (!curr_misr) {
            i2c.log_error("failed to sync checksum");
            return false;
        }
        misr.expected = *curr_misr;

        return true;
    }

    optional<Kind> read_kind() {
        auto part_id = read_data_verified<uint16_t>(Reg::PartID);
        if (!part_id) {
            i2c.log_error("failed to read part ID");
            return {};
        }

        switch ((Kind)*part_id) {
        case Kind::ENS160:  // FALL THROUGH
        case Kind::ENS161: return (Kind)*part_id; break;
        default: {
            i2c.log_error("unrecognised part ID 0x%04x", *part_id);
            return {};
        } break;
        }
    }

    optional<AppVersion> read_app_version() {
        // clear GPR to ensure new-gpr is triggered
        if (!i2c.write(Reg::Command, Cmd::ClearGPR)) {
            i2c.log_error("failed to send cmd `ClearGPR`");
            return {};
        }

        if (!i2c.write(Reg::Command, Cmd::GetAppVersion)) {
            i2c.log_error("failed to send cmd `GetAppVersion`");
            return {};
        }

        if (!status_await([](auto& x) { return x.new_gpr; })) return {};

        auto version = read_data_verified<AppVersion>(Reg::GprRead4);
        if (!version) {
            i2c.log_error("failed to read version");
            return {};
        }

        return version;
    }

    // NB:  Datasheet says registers in [0x20, 0x37] trigger a MSIR update.
    //      This is a lie. It looks like *every* read updates MISR,
    //      *except* the MSIR register itself.
    template <typename A>
    optional<A> read_data(Reg reg) {
        auto x = i2c.read<A>(reg);
        if (!x) return {};

        misr.update(*x);
        return x;
    }

    template <typename A>
    optional<A> read_data_verified(Reg reg) {
        auto x = read_data<A>(reg);
        if (x && !misr_verify()) return {};
        return x;
    }

    [[nodiscard]] optional<Status> status() {
        return read_data_verified<Status>(Reg::DeviceStatus);
    }

    template <typename F>
    bool status_await(F&& filter) {
        for (;;) {
            auto status = this->status();
            if (!status) return false;
            if (filter(*status)) return true;
            vTaskYieldWithinAPI();
        }
    }

    bool misr_verify() {
        auto actual = i2c.read<uint8_t>(Reg::DataChecksum);
        if (!actual) {
            i2c.log_error("failed to read checksum");
            return false;
        }

        if (misr.expected != actual) {
            i2c.log_warn("checksum mismatch. expected=0x%02x actual=0x%02x", misr.expected, *actual);
            misr.expected = *actual;  // sync w/ actual previous value
            return false;
        }

        return true;
    }
};

}  // namespace

unique_ptr<SensorPeriodic> ens16x(i2c_inst_t& bus, EnvironmentalFilter side) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<ENS16xSensor>(bus, address, side); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensors
