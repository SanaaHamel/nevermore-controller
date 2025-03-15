#include "sgp40.hpp"
#include "config.hpp"
#include "utility/numeric_suffixes.hpp"
#include <bit>
#include <cstdint>

using namespace std;

namespace nevermore::sensorium::sensors {

// SGP40 supports std-mode 100 kbits/s and fast mode 400 kbits/s
static_assert(I2C_BAUD_RATE_SENSOR_MAX <= 400'000,
        "`config.hpp`'s `I2C_BAUD_RATE_SENSOR_MAX` is too high for SGP40 (max 400 kbit/s)");

namespace {

constexpr uint8_t ADDRESSES[]{0x59};

// clangd bug: crash if `byteswap` is used w/o `std::` prefix in enum RHS.
// SGP40 wants its cmds in BE order
enum class Cmd : uint16_t {
    SGP40_SELF_TEST = std::byteswap(0x280E_u16),      // available in all modes, doesn't change mode
    SGP40_MEASURE = std::byteswap(0x260F_u16),        // transitions to measure mode
    SGP4x_HEATER_OFF = std::byteswap(0x3615_u16),     // transitions to idle mode
    SGP4x_SERIAL_NUMBER = std::byteswap(0x3682_u16),  // only available when in idle mode
};

constexpr uint16_t to_tick(double n, double min, double max) {
    return uint16_t((clamp(n, min, max) - min) / (max - min) * UINT16_MAX);
}
static_assert(to_tick(50, 0, 100) == 0x8000 - 1, "humidity check");  // -1 b/c of truncation
static_assert(to_tick(25, -45, 130) == 0x6666, "temperature check");

struct SGP40 final : SensorI2C<Cmd, "SGP40", 0xFF> {
    using SensorI2C::SensorI2C;

    struct [[gnu::packed]] MeasureParams {
        uint16_t humidity;
        uint8_t crc0;
        uint16_t temperature;
        uint8_t crc1;
    };

    bool setup() {
        if (!i2c.touch(Cmd::SGP4x_HEATER_OFF)) return false;

        i2c.log("detected");
        return true;
    }

    bool issue(EnvState const& state) override {
        return measure(state.temperature(), state.humidity());
    }

    pair<EnvState, optional<uint16_t>> readback() override {
        auto response = i2c.read_crc<uint16_t>();
        if (!response) return {};

        return {{}, byteswap(*response)};
    }

    [[nodiscard]] bool measure(float temperature, float humidity) const {
        uint16_t temperature_tick = byteswap(to_tick(temperature, -45, 130));
        uint16_t humidity_tick = byteswap(to_tick(humidity, 0, 100));
        MeasureParams params{
                .humidity = humidity_tick,
                .crc0 = crc8(humidity_tick, 0xFF),
                .temperature = temperature_tick,
                .crc1 = crc8(temperature_tick, 0xFF),
        };
        return i2c.write(Cmd::SGP40_MEASURE, params);
    }
};

}  // namespace

unique_ptr<Sensor> sgp40(Pins::BusI2C const& pins) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<SGP40>(pins, address); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensorium::sensors
