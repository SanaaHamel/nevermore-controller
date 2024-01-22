#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include <memory>

namespace nevermore {

struct I2C_Bus;

namespace sensors {

using namespace std::literals::chrono_literals;

// TODO: verify delay
constexpr auto BME68x_POWER_ON_DELAY = 10ms;

std::unique_ptr<SensorPeriodic> bme68x(I2C_Bus&, EnvironmentalFilter);

}  // namespace sensors
}  // namespace nevermore
