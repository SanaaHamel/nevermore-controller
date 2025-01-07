#pragma once

#include "sensors/base.hpp"
#include <chrono>
#include <memory>

namespace nevermore::sensorium::sensors {

using namespace std::literals::chrono_literals;

constexpr auto SGP30_POWER_ON_DELAY = 100ms;
constexpr auto SGP30_READ_DELAY = 25ms;

std::unique_ptr<Sensor> sgp30(Pins::BusI2C const&);

}  // namespace nevermore::sensorium::sensors
