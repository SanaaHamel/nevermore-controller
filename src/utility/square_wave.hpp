#pragma once

// Generate a square wave of a given freq on some arbitrary pin.
// Used for debugging purposes, particularly when testing the tachometer implementation.

#include "config/pins.hpp"
#include <cstdint>

namespace nevermore {

// sets up a PWM to emit a square wave with the given frequency
void square_wave_pwm_init(GPIO, uint32_t hz, float duty = .5f);
GPIO square_wave_pwm_first_available_pin();

}  // namespace nevermore
