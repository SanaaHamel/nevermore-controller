#pragma once

#include "sensors/base.hpp"
#include <chrono>
#include <memory>

namespace nevermore::sensorium::sensors {

using namespace std::literals::chrono_literals;

// spec is slightly better: 0.6ms. Round up to keep `ms` units.
constexpr auto SGP40_POWER_ON_DELAY = 1ms;
constexpr auto SGP40_READ_DELAY = 320ms;

std::unique_ptr<Sensor> sgp40(Pins::BusI2C const&);

}  // namespace nevermore::sensorium::sensors
