#include "sgp40.hpp"
#include "config.hpp"
#include "lib/sensirion_gas_index_algorithm.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/i2c.hpp"
#include "sensors.hpp"
#include "sensors/async_sensor.hpp"
#include "sensors/environmental.hpp"
#include "sensors/gas_index.hpp"
#include "utility/numeric_suffixes.hpp"
#include "utility/packed_tuple.hpp"
#include <bit>
#include <cstdint>
#include <cstdio>
#include <utility>

#define DBG_SGP40_TEMP_HUMIDITY_BREAKDOWN 0
#if DBG_SGP40_TEMP_HUMIDITY_BREAKDOWN
#include <array>
#endif

using namespace std;
using namespace BLE;

namespace nevermore::sensors {

// SGP40 supports std-mode 100 kbits/s and fast mode 400 kbits/s
static_assert(
        I2C_BAUD_RATE <= 400 * 1000, "`config.hpp`'s `I2C_BAUD_RATE` is too high for SGP40 (max 400 kbit/s)");

namespace {

constexpr uint8_t SGP40_ADDRESS = 0x59;

// clangd bug: crash if `byteswap` is used w/o `std::` prefix in enum RHS.
// SGP40 wants its cmds in BE order
enum class Cmd : uint16_t {
    SGP40_SELF_TEST = std::byteswap(0x280E_u16),      // available in all modes, doesn't change mode
    SGP40_MEASURE = std::byteswap(0x260F_u16),        // transitions to measure mode
    SGP4x_HEATER_OFF = std::byteswap(0x3615_u16),     // transitions to idle mode
    SGP4x_SERIAL_NUMBER = std::byteswap(0x3682_u16),  // only available when in idle mode
};

bool sgp4x_heater_off(i2c_inst_t& bus) {
    return i2c_write("SGP40", bus, SGP40_ADDRESS, Cmd::SGP4x_HEATER_OFF);
}

// returns true IIF self-test passed. any error (I2C or self-test) -> false
bool sgp40_self_test(i2c_inst_t& bus) {
    if (!i2c_write("SGP40", bus, SGP40_ADDRESS, Cmd::SGP40_SELF_TEST)) return false;

    task_delay(320ms);  // spec says max delay of 320ms

    auto response = i2c_read_blocking_crc<0xFF, uint16_t>("SGP40", bus, SGP40_ADDRESS);
    if (!response) return false;

    auto code = byteswap(get<0>(*response));
    switch (code) {
    case 0xD400: return true;   // tests passed
    case 0x4B00: return false;  // tests failed
    }

    printf("WARN - SGP40 - unexpected response code from self-test: 0x%02x\n", int(code));
    return false;
}

constexpr uint16_t to_tick(double n, double min, double max) {
    return uint16_t((clamp(n, min, max) - min) / (max - min) * UINT16_MAX);
}
static_assert(to_tick(50, 0, 100) == 0x8000 - 1, "humidity check");  // -1 b/c of truncation
static_assert(to_tick(25, -45, 130) == 0x6666, "temperature check");

bool sgp40_measure_issue(i2c_inst_t& bus, double temperature, double humidity) {
    uint16_t temperature_tick = byteswap(to_tick(temperature, -45, 130));
    uint16_t humidity_tick = byteswap(to_tick(humidity, 0, 100));
    PackedTuple cmd{Cmd::SGP40_MEASURE, humidity_tick, crc8(humidity_tick, 0xFF), temperature_tick,
            crc8(temperature_tick, 0xFF)};
    return i2c_write("SGP40", bus, SGP40_ADDRESS, cmd);
}

optional<uint16_t> sgp40_measure_read(i2c_inst_t& bus) {
    auto response = i2c_read_blocking_crc<0xFF, uint16_t>("SGP40", bus, SGP40_ADDRESS);
    if (!response) return false;

    auto&& [voc_raw] = *response;
    return byteswap(voc_raw);
}

bool sgp40_exists(i2c_inst_t& bus) {
    return sgp4x_heater_off(bus);  // FUTURE WORK: better way of doing this?
}

struct SGP40 final : SensorPeriodic {
    i2c_inst_t& bus;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    GasIndexAlgorithmParams gas_index_algorithm{};
    EnvironmentalFilter side;

    SGP40(i2c_inst_t& bus, EnvironmentalFilter side) : bus(bus), side(side) {
        GasIndexAlgorithm_init(&gas_index_algorithm, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    }

    [[nodiscard]] char const* name() const override {
        return "SGP40";
    }

    void read() override {
        if (!sgp40_measure_issue(bus, side.compensation_temperature(), side.compensation_humidity())) return;

        task_delay(320ms);

        auto voc_raw = sgp40_measure_read(bus);
        if (!voc_raw) return;

        side.set(VOCRaw(*voc_raw));
        if (side.was_voc_breakdown_measurement()) return;

        // ~330 us during steady-state, ~30 us during startup blackout
        int32_t gas_index{};
        GasIndexAlgorithm_process(&gas_index_algorithm, *voc_raw, &gas_index);
        assert(0 <= gas_index && gas_index <= 500 && "result out of range?");
        side.set(GIAState(gas_index_algorithm));
        side.set(VOCIndex(gas_index));
    }
};

}  // namespace

unique_ptr<SensorPeriodic> sgp40(i2c_inst_t& bus, EnvironmentalFilter side) {
    if (!sgp40_exists(bus)) return {};  // nothing found
    if (!sgp40_self_test(bus)) {
        printf("ERR - SGP40 - failed self-test\n");
        return {};
    }

    return make_unique<SGP40>(bus, side);
}

}  // namespace nevermore::sensors
