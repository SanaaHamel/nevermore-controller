#pragma once

#include "sensors/base.hpp"
#include <chrono>
#include <memory>

namespace nevermore::sensorium::sensors {

using namespace std::literals::chrono_literals;

// UNKNOWN, no documentation
constexpr auto ZMOD4410_POWER_ON_DELAY = 100ms;
// UNKNOWN, depends on chosen config + jitter, would have to measure.
constexpr auto ZMOD4410_READ_DELAY = 500ms;

std::unique_ptr<Sensor> zmod4410(Pins::BusI2C const&);

}  // namespace nevermore::sensorium::sensors
