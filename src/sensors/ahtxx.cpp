#include "ahtxx.hpp"
#include "config.hpp"
#include "environmental_i2c.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/task.hpp"
#include "utility/numeric_suffixes.hpp"
#include <cstdint>
#include <optional>

using namespace std;
using namespace BLE;

// Constants, payloads, and timings were draw from Klipper's `aht10.py` module

namespace nevermore::sensors {

static_assert(
        I2C_BAUD_RATE <= 400'000, "`config.hpp`'s `I2C_BAUD_RATE` is too high for AHTxx (max 400 kbit/s)");

namespace {

constexpr auto MEASURE_READ_RETRIES = 5u;

// Reportedly AHT10 can have a address of 0x39 in certain cases?
constexpr array ADDRESSES{0x38_u8, 0x39_u8};

// From AHT21 datasheet
// AHT10 doesn't say anything about a payload, but AHT21 says to send `0x33 0x00`
constexpr array CMD_PAYLOAD_MEASURE{0x33_u8, 0x00_u8};
constexpr auto DELAY_MEASURE = 110ms;  // AHT21 spec says 250ms, Klipper does 110ms?
constexpr auto DELAY_RESET = 100ms;    // AHT10 and AHT21 spec says < 20ms, Klipper does 100ms?

// From Klippy's aht10.py
constexpr auto CMD_PAYLOAD_INIT = 0x0008_u16;   // not specified in AHT10 or AHT21 spec?
constexpr auto CMD_PAYLOAD_RESET = 0x0008_u16;  // not specified in AHT10 or AHT21 spec?
constexpr auto DELAY_KLIPPER_INIT = 100ms;

enum class Reg : uint8_t {
    Status = 0x71,
    StartMeasurement = 0xAC,
    Reset = 0xBA,
    Init_2x = 0xBE,  // AHT2x
    Init_1x = 0xE1,  // AHT1x (? yet seems to work for AHT2x devices)
};

struct [[gnu::packed]] Status {
    // AHT10 spec. `Command = 0b1x`, so `Command` and `Command1` are equiv
    enum Mode { Normal = 0, Cyclic = 1, Command = 0b10, Command1 = 0b11 };

    uint8_t _unknown0 : 2;
    uint8_t calibrated : 1;
    uint8_t _unknown1 : 1;
    uint8_t mode : 2;  // `Mode`, defined on AHT10, but not on AHT21
    uint8_t busy : 1;
};
static_assert(sizeof(Status) == sizeof(uint8_t));

struct [[gnu::packed]] State {
    Status status;
    // fields are stored in big endian order
    uint8_t humidity0;
    uint8_t humidity1;
    // layout is `0bHHHH'TTTT`
    uint8_t temperature0 : 4;
    uint8_t humidity2 : 4;
    uint8_t temperature1;
    uint8_t temperature2;
};
static_assert(sizeof(State) == 6);

struct AHTxxSensor final : SensorPeriodicEnvI2C<Reg, "AHTxx"> {
    using SensorPeriodicEnvI2C::SensorPeriodicEnvI2C;

    bool setup() {  // NOLINT(readability-make-member-function-const)
        if (!i2c.write(Reg::Init_1x, CMD_PAYLOAD_INIT)) return false;

        task_delay(DELAY_KLIPPER_INIT);
        return true;
    }

    bool reset() {  // NOLINT(readability-make-member-function-const)
        if (!i2c.write(Reg::Reset, CMD_PAYLOAD_RESET)) return false;

        task_delay(DELAY_RESET);
        return true;
    }

    void read() override {
        auto state = measure();
        if (!state) return;

        auto t_raw = state->temperature2 | (uint32_t(state->temperature1) << 8) |
                     (uint32_t(state->temperature0) << 16);
        auto h_raw =
                state->humidity2 | (uint32_t(state->humidity1) << 4) | (uint32_t(state->humidity0) << 12);
        auto t = (double(t_raw) / (1 << 20)) * 200 - 50;
        auto h = (double(h_raw) / (1 << 20)) * 100;

        side.set(Temperature(t));
        side.set(Humidity(clamp(h, 0., 100.)));
    }

    optional<State> measure() {
        if (i2c.write(Reg::StartMeasurement, CMD_PAYLOAD_MEASURE)) {
            for (unsigned i = MEASURE_READ_RETRIES; 0 < i; --i) {
                task_delay(DELAY_MEASURE);

                // AHT21 has a CRC at the end, but AHT10 (haven't checked AHT20)
                auto result = i2c.read<State>();
                if (result && !result->status.busy) return result;
            }
        }

        reset();  // just try resetting the bloody thing...
        return {};
    }
};

}  // namespace

unique_ptr<SensorPeriodic> ahtxx(I2C_Bus& bus, EnvironmentalFilter side) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<AHTxxSensor>(bus, address, side); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensors
