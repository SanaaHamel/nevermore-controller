#include "sensors.hpp"
#include "hardware/adc.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/i2c_hw.hpp"
#include "sdk/i2c_pio.hpp"
#include "sensors/ahtxx.hpp"
#include "sensors/async_sensor.hpp"
#include "sensors/bme280.hpp"
#include "sensors/bme68x.hpp"
#include "sensors/bmp280.hpp"
#include "sensors/cst816s.hpp"
#include "sensors/ens16x.hpp"
#include "sensors/environmental.hpp"
#include "sensors/htu2xd.hpp"
#include "sensors/sgp30.hpp"
#include "sensors/sgp40.hpp"
#include <algorithm>
#include <cstdio>
#include <vector>

using namespace std;

namespace nevermore::sensors {

Sensors g_sensors;
Config g_config;

namespace {

constexpr uint32_t ADC_CHANNEL_TEMP_SENSOR = 4;

constexpr auto SENSOR_POWER_ON_DELAY = max({
        AHTxx_POWER_ON_DELAY,
        BME280_POWER_ON_DELAY,
        BME68x_POWER_ON_DELAY,
        BMP280_POWER_ON_DELAY,
        ENS16x_POWER_ON_DELAY,
        HTU21D_POWER_ON_DELAY,
        SGP30_POWER_ON_DELAY,
        SGP40_POWER_ON_DELAY,
});

using VecSensors = vector<unique_ptr<Sensor>>;

VecSensors g_sensor_devices;

struct McuTemperature final : SensorPeriodic {
    [[nodiscard]] char const* name() const override {
        return "MCU Temperature";
    }

    void read() override {
        nevermore::sensors::g_sensors.temperature_mcu = measure();
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

VecSensors sensors_init_bus(I2C_Bus& bus, optional<EnvironmentalFilter::Kind> state) {
    VecSensors sensors;
    auto add = [&](auto p) {
        if (!p) return;
        printf("Found %s\n", p->name());
        p->start();
        sensors.push_back(std::move(p));
    };

    if (state) {
        auto add_env = [&](auto&& fn) { add(fn(bus, *state)); };
        add_env(ahtxx);
        add_env(bme280);
        add_env(bme68x);
        add_env(bmp280);
        add_env(ens16x);
        add_env(htu2xd);
        add_env(sgp30);
        add_env(sgp40);
    }

    add(CST816S::mk(bus));

    return sensors;
}

template <typename F>
void foreach_sensor_bus(F&& go) {
    for (auto const& bus_pins : Pins::active().i2c) {
        if (!bus_pins) continue;

        optional<EnvironmentalFilter::Kind> kind;
        switch (bus_pins.kind) {
        default: break;
        case Pins::BusI2C::Kind::intake: kind = EnvironmentalFilter::Kind::Intake; break;
        case Pins::BusI2C::Kind::exhaust: kind = EnvironmentalFilter::Kind::Exhaust; break;
        }

        I2C_Bus* bus;
        if (auto hw = bus_pins.hardware_bus_num()) {
            assert(hw <= 1);
            bus = &i2c.at(*hw);
        } else {
            assert(false && "PIO I2C not impl");
            continue;
        }

        go(*bus, kind);
    }
}

}  // namespace

Sensors Sensors::with_fallbacks(Config const& config) const {
    EnvironmentalFilter intake{EnvironmentalFilter::Kind::Intake};
    EnvironmentalFilter exhaust{EnvironmentalFilter::Kind::Exhaust};
    auto apply = [&]<typename A>(A& x, EnvironmentalFilter side) { x = side.get<A>(*this, config); };
    Sensors sensors = *this;
    apply(sensors.temperature_intake, intake);
    apply(sensors.humidity_intake, intake);
    apply(sensors.pressure_intake, intake);
    apply(sensors.voc_index_intake, intake);
    apply(sensors.temperature_exhaust, exhaust);
    apply(sensors.humidity_exhaust, exhaust);
    apply(sensors.pressure_exhaust, exhaust);
    apply(sensors.voc_index_exhaust, exhaust);
    return sensors;
}

bool init() {
    adc_select_input(ADC_CHANNEL_TEMP_SENSOR);
    adc_set_temp_sensor_enabled(true);
    g_mcu_temperature_sensor.start();

    // Explicitly reset b/c we may be restarting the program w/o power cycling the device.
    CST816S::reset_all();
    CST816S::register_isr();

    printf("Waiting %u ms for sensor init\n", unsigned(SENSOR_POWER_ON_DELAY / 1ms));
    task_delay<SENSOR_POWER_ON_DELAY>();

    foreach_sensor_bus([](auto&& bus, auto&& kind) {
        printf("%s - initializing sensors...\n", bus.name());
        auto xs = sensors_init_bus(bus, kind);
        if (xs.empty()) bus.log_warn("N/A", 0, "!! No sensors found?");

        std::move(begin(xs), end(xs), back_inserter(g_sensor_devices));
    });

    // honestly if we low on space or are getting fragmentation issues we might
    // as well just reserve a `sizeof(P*) * 32` block and call it a day.
    g_sensor_devices.shrink_to_fit();

    // wait again b/c probing might be implemented by sending a reset command to the sensor
    task_delay<SENSOR_POWER_ON_DELAY>();

    return true;
}

void calibrations_reset() {
    vTaskSuspendAll();
    {
        printf("resetting sensor calibrations\n");

        for (auto&& device : g_sensor_devices)
            device->calibration_reset();
    }
    xTaskResumeAll();
}

void calibrations_force_checkpoint() {
    vTaskSuspendAll();
    {
        printf("checkpointing sensor calibrations\n");

        for (auto&& device : g_sensor_devices)
            device->calibration_force_checkpoint();
    }
    xTaskResumeAll();
}

}  // namespace nevermore::sensors
