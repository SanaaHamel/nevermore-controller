#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include <memory>

namespace nevermore {

struct I2C_Bus;

namespace sensors {

using namespace std::literals::chrono_literals;

constexpr auto ENS16x_POWER_ON_DELAY = 10ms;

// ENS160 and ENS161
std::unique_ptr<SensorPeriodic> ens16x(I2C_Bus&, EnvironmentalFilter);

}  // namespace sensors
}  // namespace nevermore
