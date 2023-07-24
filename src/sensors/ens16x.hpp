#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include "hardware/i2c.h"
#include <chrono>
#include <memory>

namespace nevermore::sensors {

using namespace std::literals::chrono_literals;

constexpr auto ENS16x_POWER_ON_DELAY = 10ms;

// ENS160 and ENS161
std::unique_ptr<SensorPeriodic> ens16x(i2c_inst_t&, EnvironmentalFilter);

}  // namespace nevermore::sensors
