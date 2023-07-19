#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include "hardware/i2c.h"
#include <chrono>
#include <memory>

namespace nevermore::sensors {

using namespace std::literals::chrono_literals;

// TODO: verify delay
constexpr auto BME68x_POWER_ON_DELAY = 10ms;

std::unique_ptr<SensorPeriodic> bme68x(i2c_inst_t&, EnvironmentalFilter);

}  // namespace nevermore::sensors
