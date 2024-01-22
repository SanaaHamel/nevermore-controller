#pragma once

#include "async_sensor.hpp"
#include "environmental.hpp"
#include <memory>

namespace nevermore {

struct I2C_Bus;

namespace sensors {

using namespace std::literals::chrono_literals;

// AHT10 spec says 20ms for power on.
// AHT21 spec says 100ms to 500ms for power on.
// IDK what the hell AHT21 is doing.
constexpr auto AHTxx_POWER_ON_DELAY = 100ms;

std::unique_ptr<SensorPeriodic> ahtxx(I2C_Bus&, EnvironmentalFilter);

}  // namespace sensors
}  // namespace nevermore
