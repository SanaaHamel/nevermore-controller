#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include <memory>

namespace nevermore {

struct I2C_Bus;

namespace sensors {

using namespace std::literals::chrono_literals;

// spec is slightly better: 0.6ms. Round up to keep `ms` units.
constexpr auto SGP40_POWER_ON_DELAY = 1ms;

std::unique_ptr<SensorPeriodic> sgp40(I2C_Bus&, EnvironmentalFilter);

}  // namespace sensors
}  // namespace nevermore
