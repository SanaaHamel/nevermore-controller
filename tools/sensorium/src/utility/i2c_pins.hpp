#pragma once

#include "config/pins.hpp"
#include <vector>

namespace nevermore::sensorium {

std::vector<Pins::BusI2C> i2c_pins();

}
