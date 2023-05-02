#include "sensors.hpp"
#include "gatt.hpp"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/timer.hpp"
#include "sensors/async_sensor.hpp"
#include "sensors/htu2xd.hpp"
#include "sensors/sgp40.hpp"
#include "utility/misc.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <vector>

using EnvironmentService::VOCIndex;

namespace {

constexpr uint32_t ADC_CHANNEL_TEMPSENSOR = 4;

constexpr auto SENSOR_POWER_ON_DELAY = std::max({HTU21D_POWER_ON_DELAY, SGP40_POWER_ON_DELAY});

using VecSensors = std::vector<std::unique_ptr<Sensor>>;

VecSensors g_sensors_intake;
VecSensors g_sensors_exhaust;

double mcu_temperature() {
    // ref https://github.com/raspberrypi/pico-micropython-examples/blob/master/adc/temperature.py
    constexpr auto SCALE_COEFFICIENT = 3.3 / 65535;
    constexpr uint32_t BITS = 12;

    adc_select_input(ADC_CHANNEL_TEMPSENSOR);
    uint32_t raw32 = adc_read();
    // Scale raw reading to 16 bit value using a Taylor expansion (for 8 <= bits <= 16)
    uint16_t raw16 = raw32 << (16 - BITS) | raw32 >> (2 * BITS - 16);
    auto reading = raw16 * SCALE_COEFFICIENT;
    // The temperature sensor measures the Vbe voltage of a biased bipolar diode, connected to the fifth ADC
    // channel Typically, Vbe = 0.706V at 27 degrees C, with a slope of -1.721mV (0.001721) per degree.
    auto deg_c = 27 - (reading - 0.706) / 0.001721;
    // printf("MCU temp = %f\n", deg_c);
    return deg_c;
}

void DBG_i2c_scan(i2c_inst_t* bus) {
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        if (i2c_address_reserved(addr)) {
            printf(" ");
        } else {
            uint8_t rxdata;
            auto ret = i2c_read_blocking(bus, addr, &rxdata, 1, false);
            printf(ret < 0 ? "." : "@");
        }

        printf(addr % 16 == 15 ? "\n" : "  ");
    }
}

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
    probe_for(sgp40(bus, state));

    if (sensors.empty()) printf("!! No sensors found?\n");
    return sensors;
}

}  // namespace

bool sensors_init(async_context_t& ctx_async, EnvironmentService::ServiceData& state) {
    adc_select_input(ADC_CHANNEL_TEMPSENSOR);
    adc_set_temp_sensor_enabled(true);

    printf("Waiting %u ms for sensor init\n", unsigned(SENSOR_POWER_ON_DELAY / 1ms));
    busy_wait(SENSOR_POWER_ON_DELAY);

    DBG_i2c_scan(i2c0);

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
