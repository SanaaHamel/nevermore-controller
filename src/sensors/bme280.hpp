#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include "hardware/i2c.h"
#include <chrono>
#include <memory>

using namespace std::literals::chrono_literals;

constexpr auto BME280_POWER_ON_DELAY = 2ms;

std::unique_ptr<SensorPeriodic> bme280(i2c_inst_t*, EnvironmentalSensorData state);
