#pragma once

#include "sensors/base.hpp"
#include <chrono>
#include <memory>

namespace nevermore::sensorium::sensors {

using namespace std::literals::chrono_literals;

constexpr auto SHT4x_POWER_ON_DELAY = 1ms;
constexpr auto SHT4x_READ_DELAY = 10ms;

std::unique_ptr<Sensor> sht4x(Pins::BusI2C const&);

}  // namespace nevermore::sensorium::sensors
