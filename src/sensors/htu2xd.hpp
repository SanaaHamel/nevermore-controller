#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include "hardware/i2c.h"
#include <chrono>
#include <memory>

using namespace std::literals::chrono_literals;

// really slow to start up
constexpr auto HTU21D_POWER_ON_DELAY = 15ms;

std::unique_ptr<SensorPeriodic> htu2xd(i2c_inst_t*, EnvironmentalSensorData state);
