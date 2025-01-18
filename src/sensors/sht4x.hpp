#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include <memory>

namespace nevermore {

struct I2C_Bus;

namespace sensors {

using namespace std::literals::chrono_literals;

constexpr auto SHT4x_POWER_ON_DELAY = 1ms;

std::unique_ptr<SensorPeriodic> sht4x(I2C_Bus&, EnvironmentalFilter);

}  // namespace sensors
}  // namespace nevermore
