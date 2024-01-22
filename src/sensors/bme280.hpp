#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include <memory>

namespace nevermore {

struct I2C_Bus;

namespace sensors {

using namespace std::literals::chrono_literals;

constexpr auto BME280_POWER_ON_DELAY = 2ms;

std::unique_ptr<SensorPeriodic> bme280(I2C_Bus&, EnvironmentalFilter);

}  // namespace sensors
}  // namespace nevermore
