#include "sht4x.hpp"
#include "config.hpp"
#include "environmental_i2c.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/task.hpp"
#include "utility/numeric_suffixes.hpp"
#include <cstdint>
#include <optional>

using namespace std;
using namespace BLE;

namespace nevermore::sensors {

static_assert(I2C_BAUD_RATE_SENSOR_MAX <= 1'000'000,
        "`config.hpp`'s `I2C_BAUD_RATE_SENSOR_MAX` is too high for SHT4x (max 1 Mbit/s)");

namespace {

constexpr array ADDRESSES{0x44_u8, 0x45_u8, 0x46_u8};

enum class Cmd : uint8_t {
    MeasureHi = 0xFD,   // 6 bytes, high precision, 8.3ms delay according to datasheet
    MeasureMid = 0xF6,  // 6 bytes, medium precision, 4.5ms delay according to datasheet
    MeasureLo = 0xE0,   // 6 bytes, low precision, 1.6ms delay according to datasheet
    Serial = 0x89,      // 6 bytes, datasheet doesn't say anything about delays
    Reset = 0x94,       // no response, 1ms delay (same as power on)
    // Device has an on-board heater & commands for managing it.
    // We don't use the heater, so these aren't listed. See datasheet for details.
};

struct Measurement {
    float temperature;  // in c
    float humidity;     // relative humidity
};

struct SHT4x final : SensorPeriodicEnvI2C<Cmd, "SHT4x", 0xFF> {
    using SensorPeriodicEnvI2C::SensorPeriodicEnvI2C;
    static_assert(crc(0xEFBE_u16) == 0x92);

    bool setup() {  // NOLINT(readability-make-member-function-const)
        // datasheet doesn't say how long `serial` takes to process
        auto s = serial();
        if (!s) return false;

        i2c.log("serial 0x%08X", unsigned(*s));
        return true;
    }

    void read() override {
        auto m = measure();
        if (!m) return;

        auto _ = side.guard();
        side.set(Temperature(m->temperature));
        side.set(Humidity(clamp(m->humidity, 0.f, 100.f)));
    }

    optional<Measurement> measure() {
        auto result = read6<10ms>(Cmd::MeasureHi);
        if (!result) return {};

        auto&& [t, h] = *result;
        return Measurement{
                // NOLINTNEXTLINE(readability-math-missing-parentheses)
                .temperature = -45 + 175 * (float(t) / ((1u << 16) - 1)),
                // NOLINTNEXTLINE(readability-math-missing-parentheses)
                .humidity = -6 + 125 * (float(h) / ((1u << 16) - 1)),
        };
    }

    optional<uint32_t> serial() {
        // datasheet doesn't say how long `serial` takes to process
        auto raw = read6<10ms>(Cmd::Serial);
        if (!raw) return {};

        auto&& [a, b] = *raw;
        return uint32_t(a) << 16 | uint32_t(b);
    }

private:
    template <TaskDelayArg delay>
    optional<pair<uint16_t, uint16_t>> read6(Cmd cmd) {
        if (!i2c.touch(cmd)) return {};
        task_delay<delay>();

        struct [[gnu::packed]] Raw {
            ResponseCRC<uint16_t> a;
            ResponseCRC<uint16_t> b;
        };
        auto raw = i2c.read<Raw>();
        if (!raw) return {};
        if (!i2c.verify(raw->a)) return {};
        if (!i2c.verify(raw->b)) return {};

        // pre-emptively byteswap everything.
        // all our callers need this in little endian
        return pair{byteswap(raw->a.data), byteswap(raw->b.data)};
    }
};

}  // namespace

unique_ptr<SensorPeriodic> sht4x(I2C_Bus& bus, EnvironmentalFilter side) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<SHT4x>(bus, address, side); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensors
