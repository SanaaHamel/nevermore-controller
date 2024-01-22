#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include <memory>

namespace nevermore {

struct I2C_Bus;

namespace sensors {

using namespace std::literals::chrono_literals;

// really slow to start up
constexpr auto HTU21D_POWER_ON_DELAY = 15ms;

std::unique_ptr<SensorPeriodic> htu2xd(I2C_Bus&, EnvironmentalFilter);

}  // namespace sensors
}  // namespace nevermore
