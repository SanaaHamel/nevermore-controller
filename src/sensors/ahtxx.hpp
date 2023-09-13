#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include "hardware/i2c.h"
#include <chrono>
#include <memory>

namespace nevermore::sensors {

using namespace std::literals::chrono_literals;

// AHT10 spec says 20ms for power on.
// AHT21 spec says 100ms to 500ms for power on.
// IDK what the hell AHT21 is doing.
constexpr auto AHTxx_POWER_ON_DELAY = 100ms;

std::unique_ptr<SensorPeriodic> ahtxx(i2c_inst_t&, EnvironmentalFilter);

}  // namespace nevermore::sensors
