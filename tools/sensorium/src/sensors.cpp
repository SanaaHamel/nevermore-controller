#include "sensors.hpp"
#include "sdk/i2c.hpp"
#include "sensors/ags10.hpp"
#include "sensors/sgp30.hpp"
#include "sensors/sgp40.hpp"
#include "utility/i2c_pins.hpp"
#include <algorithm>
#include <cstdio>
#include <ranges>
#include <vector>

using namespace std;

namespace nevermore::sensorium::sensors {

namespace {

constexpr auto SENSOR_POWER_ON_DELAY = max({
        AGS10_POWER_ON_DELAY,
        SGP30_POWER_ON_DELAY,
        SGP40_POWER_ON_DELAY,
});

constexpr auto SENSOR_READ_DELAY = max({
        AGS10_READ_DELAY,
        SGP30_READ_DELAY,
        SGP40_READ_DELAY,
});

using VecSensors = vector<unique_ptr<Sensor>>;

VecSensors g_sensor_devices;

VecSensors sensors_on_bus(Pins::BusI2C const& pins) {
    VecSensors sensors;
    auto add = [&](auto&& fn) {
        if (auto p = fn(pins)) {
            sensors.push_back(std::move(p));
        }
    };

    add(ags10);
    add(sgp30);
    add(sgp40);

    return sensors;
}

}  // namespace

bool init() {
    printf("Waiting %u ms for sensor init\n", unsigned(SENSOR_POWER_ON_DELAY / 1ms));
    sleep(SENSOR_POWER_ON_DELAY);

    for (auto& pins : i2c_pins()) {
        auto xs = Sensor::using_(pins, [&]() { return sensors_on_bus(pins); });
        ranges::move(xs, back_inserter(g_sensor_devices));
    }

    printf("%u sensors discovered\n", unsigned(g_sensor_devices.size()));

    // wait again b/c probing might be implemented by sending a reset command to the sensor
    sleep(SENSOR_POWER_ON_DELAY);
    return true;
}

void poll_issue(EnvState const& state) {
    for (auto&& sensor : g_sensor_devices)
        sensor->using_([&]() { sensor->issue(state); });  // TODO: handle issue errors?
}

EnvState poll_readback() {
    EnvState global;
    for (auto&& sensor : g_sensor_devices) {
        sensor->using_([&]() {
            auto [env, value] = sensor->readback();
            if (value) sensor->log_reading(*value);

            global = global.or_else(env);
        });
    }

    return global;
}

std::chrono::milliseconds poll_readback_delay() {
    return SENSOR_READ_DELAY;
}

}  // namespace nevermore::sensorium::sensors
