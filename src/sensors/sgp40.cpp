#include "sgp40.hpp"
#include "config.hpp"
#include "lib/sensirion_gas_index_algorithm.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/i2c.hpp"
#include "sdk/timer.hpp"
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
    return 2 == i2c_write_blocking(bus, SGP40_ADDRESS, Cmd::SGP4x_HEATER_OFF);
}

// returns true IIF self-test passed. any error (I2C or self-test) -> false
bool sgp40_self_test(i2c_inst_t& bus) {
    if (2 != i2c_write_blocking(bus, SGP40_ADDRESS, Cmd::SGP40_SELF_TEST)) return false;

    sleep(320ms);  // spec says max delay of 320ms

    auto response = i2c_read_blocking_crc<0xFF, uint8_t, uint8_t>(bus, SGP40_ADDRESS);
    if (!response) return false;

    auto&& [code, _] = *response;
    switch (code) {
    case 0xD4: return true;   // tests passed
    case 0x4B: return false;  // tests failed
    }

    printf("WARN - unexpected response code from SGP40 self-test: 0x%02x\n", int(code));
    return false;
}

bool sgp40_measure_issue(i2c_inst_t& bus, double temperature, double humidity) {
    auto to_tick = [](double n, double min, double max) {
        return byteswap(uint16_t((clamp(n, min, max) - min) / (max - min) * UINT16_MAX));
    };

    auto temperature_tick = to_tick(temperature, -45, 130);
    auto humidity_tick = to_tick(humidity, 0, 100);
    PackedTuple cmd{Cmd::SGP40_MEASURE, temperature_tick, crc8(temperature_tick, 0xFF), humidity_tick,
            crc8(humidity_tick, 0xFF)};
    return sizeof(cmd) == i2c_write_blocking(bus, SGP40_ADDRESS, cmd);
}

bool sgp40_measure_issue(
        i2c_inst_t& bus, BLE::Temperature const& temperature, BLE::Humidity const& humidity) {
    return sgp40_measure_issue(bus, temperature.value_or(25), humidity.value_or(50));
}

optional<uint16_t> sgp40_measure_read(i2c_inst_t& bus) {
    auto response = i2c_read_blocking_crc<0xFF, uint16_t>(bus, SGP40_ADDRESS);
    if (!response) return false;

    auto&& [voc_raw] = *response;
    return byteswap(voc_raw);
}

bool sgp40_exists(i2c_inst_t& bus) {
    return sgp4x_heater_off(bus);  // FUTURE WORK: better way of doing this?
}

struct SGP40 final : SensorDelayedResponse {
    i2c_inst_t& bus;               // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    EnvironmentalSensorData data;  // tiny bit wasteful, but terser to manage
    GasIndexAlgorithmParams gas_index_algorithm{};

#if DBG_SGP40_TEMP_HUMIDITY_BREAKDOWN
    uint8_t state = 0;
    array<uint16_t, 4> history{};

    [[nodiscard]] chrono::milliseconds update_period() const override {
        return SENSOR_UPDATE_PERIOD / 4;
    }
#endif

    SGP40(i2c_inst_t& bus, EnvironmentalSensorData data) : bus(bus), data(std::move(data)) {
        GasIndexAlgorithm_init(&gas_index_algorithm, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    }

    [[nodiscard]] char const* name() const override {
        return "SGP40";
    }

    [[nodiscard]] chrono::milliseconds read_delay() const override {
        return 320ms;
    }

    bool issue() override {
#if DBG_SGP40_TEMP_HUMIDITY_BREAKDOWN
        switch (state) {
        case 0: break;
        case 1: return sgp40_measure_issue(bus, get<BLE::Temperature&>(data), BLE::NOT_KNOWN);
        case 2: return sgp40_measure_issue(bus, BLE::NOT_KNOWN, get<BLE::Humidity&>(data));
        case 3: return sgp40_measure_issue(bus, BLE::NOT_KNOWN, BLE::NOT_KNOWN);
        }
#endif

        return sgp40_measure_issue(bus, get<BLE::Temperature&>(data), get<BLE::Humidity&>(data));
    }

    void read() override {
        auto voc_raw = sgp40_measure_read(bus);
        if (!voc_raw) {
            printf("SGP40 - read back failed\n");
            return;
        }

#if DBG_SGP40_TEMP_HUMIDITY_BREAKDOWN
        state = (state + 1) % 4;
        history.at(state) = *voc_raw;
        if (state == 3) {
            printf("SGP40 - both=% 7d  temp=% 7d  humid=% 7d  none=% 7d\n", history[0], history[1],
                    history[2], history[3]);
            printf("                      temp=% 7d  humid=% 7d  none=% 7d\n", history[1] - history[0],
                    history[2] - history[0], history[3] - history[0]);
        }
#endif

        // ~330 us during steady-state, ~30 us during startup blackout
        int32_t gas_index{};
        GasIndexAlgorithm_process(&gas_index_algorithm, *voc_raw, &gas_index);
        assert(0 <= gas_index && gas_index <= 500 && "result out of range?");
        if (gas_index == 0) return;  // 0 -> index not available

        get<nevermore::sensors::VOCIndex&>(data) = gas_index;
    }
};

}  // namespace

unique_ptr<SensorPeriodic> sgp40(i2c_inst_t& bus, EnvironmentalSensorData state) {
    if (!sgp40_exists(bus)) return {};  // nothing found
    if (!sgp40_self_test(bus)) {
        printf("Found SGP40, but failed self-test\n");
        return {};
    }

    return make_unique<SGP40>(bus, state);
}
