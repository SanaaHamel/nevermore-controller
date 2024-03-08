// https://www.waveshare.com/product/rp2040-zero.htm

#pragma once

#include "config/pins.hpp"
#include "pico.h"  // IWYU pragma: keep for `WAVESHARE_RP2040_ZERO`
#include <initializer_list>

#ifndef WAVESHARE_RP2040_ZERO
#error "`PICO_BOARD` is likely incorrect, `WAVESHARE_RP2040_ZERO` is not defined"
#endif

// Available as tiny impossible to solder pads on the back:
//   17, 18, 19, 20, 21, 22, 23, 24, 25

namespace nevermore {

constexpr Pins PINS_DEFAULT{
        .i2c{
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::intake, .clock = 27, .data = 26},
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::exhaust, .clock = 29, .data = 28},
        },

        .spi{
                Pins::BusSPI{.kind = Pins::BusSPI::Kind::display, .clock = 2, .send = 3, .recv = 4},
        },

        .fan_pwm = {14},
        .fan_tachometer = {13},
        .neopixel_data = {12},
        .photocatalytic_pwm = {10},

        .display_command = 5,
        .display_reset = 6,
        .display_brightness_pwm = 7,
        .touch_interrupt = 8,
        .touch_reset = 9,
};

constexpr std::initializer_list<GPIO> PINS_RESERVED_BOARD{PICO_DEFAULT_WS2812_PIN};

}  // namespace nevermore
