#pragma once

#include "config/pins.hpp"

namespace nevermore {

// Manually bit-bangs out a reset sequence.
// Pico's I2C hardware doesn't support doing this for us.
// see: https://github.com/raspberrypi/pico-feedback/issues/199
//
// returns `true` IFF no one is holding the bus.
//
// PRE:  Both pins are on the same bus.
// PRE:  `sda` is an SDA pin, `scl` is an SCL pin.
// PRE:  No other pins are assigned to the same I2C bus.
// POST: I2C bus is in an undefined state. User must call `i2c_init`.
// POST: Pins are set to `GPIO_FUNC_SIO`, pull up, out-dir
bool i2c_bitbang_reset(GPIO sda, GPIO scl, unsigned clock_cycles_timeout = 10);

}  // namespace nevermore
