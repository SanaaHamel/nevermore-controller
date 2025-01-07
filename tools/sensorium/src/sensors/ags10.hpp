#pragma once

#include "sensors/base.hpp"
#include <chrono>
#include <memory>

namespace nevermore::sensorium::sensors {

using namespace std::literals::chrono_literals;

// no documented power on delay; assume 100ms
constexpr auto AGS10_POWER_ON_DELAY = 100ms;
// this thing is SLOOOOOW to read
constexpr auto AGS10_READ_DELAY = 1500ms;

std::unique_ptr<Sensor> ags10(Pins::BusI2C const&);

}  // namespace nevermore::sensorium::sensors
