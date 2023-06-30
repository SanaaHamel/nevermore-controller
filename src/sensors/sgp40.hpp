#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include "hardware/i2c.h"
#include <chrono>
#include <memory>

using namespace std::literals::chrono_literals;

// spec is slightly better: 0.6ms. Round up to keep `ms` units.
constexpr auto SGP40_POWER_ON_DELAY = 1ms;

std::unique_ptr<SensorPeriodic> sgp40(i2c_inst_t&, EnvironmentalSensorData state);
