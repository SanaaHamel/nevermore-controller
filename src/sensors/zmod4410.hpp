#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include <memory>

namespace nevermore {

struct I2C_Bus;

namespace sensors {

using namespace std::literals::chrono_literals;

constexpr auto ZMOD4410_POWER_ON_DELAY = 1ms;

std::unique_ptr<SensorPeriodic> zmod4410(I2C_Bus&, EnvironmentalFilter);

}  // namespace sensors
}  // namespace nevermore
