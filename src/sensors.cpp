#include "sensors.hpp"
#include "gatt.hpp"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/timer.hpp"
#include "sensors/async_sensor.hpp"
#include "sensors/bme280.hpp"
#include "sensors/htu2xd.hpp"
#include "sensors/sgp40.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <vector>

using EnvironmentService::VOCIndex;

namespace {

constexpr uint32_t ADC_CHANNEL_TEMP_SENSOR = 4;

constexpr auto SENSOR_POWER_ON_DELAY = std::max({
        BME280_POWER_ON_DELAY,
        HTU21D_POWER_ON_DELAY,
        SGP40_POWER_ON_DELAY,
});

using VecSensors = std::vector<std::unique_ptr<Sensor>>;

VecSensors g_sensors_intake;
VecSensors g_sensors_exhaust;

struct McuTemperature final : SensorPeriodic {
    [[nodiscard]] char const* name() const override {
        return "MCU Temperature";
    }

    void read() override {
        g_advertise_data.environment_service_data.temperature_mcu = measure();
    }

private:
    static double measure() {
        // ref https://github.com/raspberrypi/pico-micropython-examples/blob/master/adc/temperature.py
        constexpr auto SCALE_COEFFICIENT = 3.3 / 65535;
        constexpr uint32_t BITS = 12;

        adc_select_input(ADC_CHANNEL_TEMP_SENSOR);
        uint32_t raw32 = adc_read();
        // Scale raw reading to 16 bit value using a Taylor expansion (for 8 <= bits <= 16)
        uint16_t raw16 = raw32 << (16 - BITS) | raw32 >> (2 * BITS - 16);
        auto reading = raw16 * SCALE_COEFFICIENT;
        // The temp sensor measures the Vbe voltage of a biased bipolar diode, connected to ADC channel 4.
        // Typically, Vbe = 0.706V at 27c, with a slope of -1.721mV (0.001721) per degree.
        auto deg_c = 27 - (reading - 0.706) / 0.001721;
        return deg_c;
    }
} g_mcu_temperature_sensor;

VecSensors sensors_init_bus(async_context_t& ctx_async, i2c_inst_t* bus, Sensor::Data state) {
    VecSensors sensors;
    auto probe_for = [&](auto p) {
        if (!p) return;
        printf("Found %s\n", p->name());
        p->register_(ctx_async);
        sensors.push_back(std::move(p));
    };

    // order matters since they'll be updated in whatever order they were found/probed for
    probe_for(htu2xd(bus, state));
    probe_for(bme280(bus, state));
    probe_for(sgp40(bus, state));

    if (sensors.empty()) printf("!! No sensors found?\n");
    return sensors;
}

}  // namespace

bool sensors_init(async_context_t& ctx_async, EnvironmentService::ServiceData& state) {
    adc_select_input(ADC_CHANNEL_TEMP_SENSOR);
    adc_set_temp_sensor_enabled(true);
    g_mcu_temperature_sensor.register_(ctx_async);

    printf("Waiting %u ms for sensor init\n", unsigned(SENSOR_POWER_ON_DELAY / 1ms));
    busy_wait(SENSOR_POWER_ON_DELAY);

    printf("I2C0 - initializing sensors...\n");
    g_sensors_intake = sensors_init_bus(ctx_async, i2c0,
            {state.temperature_intake, state.humidity_intake, state.pressure_intake, state.voc_index_intake});
    printf("I2C1 - initializing sensors...\n");
    g_sensors_exhaust = sensors_init_bus(ctx_async, i2c1,
            {state.temperature_exhaust, state.humidity_exhaust, state.pressure_exhaust,
                    state.voc_index_exhaust});

    // wait again b/c probing might be implemented by sending a reset command to the sensor
    busy_wait(SENSOR_POWER_ON_DELAY);

    return true;
}
