#include "sgp40.hpp"
#include "config.hpp"
#include "lib/sensirion_gas_index_algorithm.h"
#include "sdk/i2c.hpp"
#include "sensors.hpp"
#include "sensors/async_sensor.hpp"
#include "sensors/environmental.hpp"
#include "sensors/environmental_i2c.hpp"
#include "utility/numeric_suffixes.hpp"
#include "utility/packed_tuple.hpp"
#include <bit>
#include <cstdint>

using namespace std;
using namespace BLE;

namespace nevermore::sensors {

// SGP40 supports std-mode 100 kbits/s and fast mode 400 kbits/s
static_assert(
        I2C_BAUD_RATE <= 400 * 1000, "`config.hpp`'s `I2C_BAUD_RATE` is too high for SGP40 (max 400 kbit/s)");

namespace {

constexpr uint8_t ADDRESSES[]{0x59};

constexpr uint16_t SELF_TEST_OK = 0x00D4;  // byte-swapped

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

struct SGP40 final : SensorPeriodicEnvI2C<Cmd, "SGP40", 0xFF> {
    using SensorPeriodicEnvI2C::SensorPeriodicEnvI2C;

    GasIndexAlgorithmParams gas_index_algorithm{};

    bool setup() {
        GasIndexAlgorithm_init(&gas_index_algorithm, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
        if (!i2c.touch(Cmd::SGP4x_HEATER_OFF)) {
            // silently fail, likely there's no device on this address...
            return false;
        }

        if (self_test() != SELF_TEST_OK) return false;

        return true;
    }

    void read() override {
        if (!measure(side.compensation_temperature(), side.compensation_humidity())) return;

        task_delay(320ms);

        auto response = i2c.read_crc<uint16_t>();
        if (!response) return;

        auto voc_raw = byteswap(*response);
        side.set(VOCRaw(voc_raw));
        if (side.was_voc_breakdown_measurement()) return;

        // ~330 us during steady-state, ~30 us during startup blackout
        int32_t gas_index{};
        GasIndexAlgorithm_process(&gas_index_algorithm, voc_raw, &gas_index);
        assert(0 <= gas_index && gas_index <= 500 && "result out of range?");
        side.set(GIAState(gas_index_algorithm));
        side.set(VOCIndex(gas_index));
    }

    [[nodiscard]] bool measure(double temperature, double humidity) const {
        uint16_t temperature_tick = byteswap(to_tick(temperature, -45, 130));
        uint16_t humidity_tick = byteswap(to_tick(humidity, 0, 100));
        PackedTuple params{
                humidity_tick, crc8(humidity_tick, 0xFF), temperature_tick, crc8(temperature_tick, 0xFF)};
        if (!i2c.write(Cmd::SGP40_MEASURE, params)) return false;

        task_delay(320ms);
        return true;
    }

    [[nodiscard]] optional<uint16_t> self_test() const {
        // spec says max delay of 320ms
        auto const result = i2c.read_crc<uint16_t>(Cmd::SGP40_SELF_TEST, 320ms);
        if (!result) {
            i2c.log_error("self-test request failed");
        } else if (*result != SELF_TEST_OK) {
            i2c.log_error("self-test failed, result 0x%04x (expected 0x%04x)", *result, SELF_TEST_OK);
        }

        return result;
    }
};

}  // namespace

unique_ptr<SensorPeriodic> sgp40(I2C_Bus& bus, EnvironmentalFilter side) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<SGP40>(bus, address, side); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensors
