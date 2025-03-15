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
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::intake, .clock = 3, .data = 2},
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::exhaust, .clock = 5, .data = 4},
        },

        .spi{
                Pins::BusSPI{.kind = Pins::BusSPI::Kind::display, .clock = 22, .send = 19, .recv = 20},
        },

        .fan_pwm = {6},
        .fan_tachometer = {7},
        .neopixel_data = {12},
        .photocatalytic_pwm = 14,
        .vent_servo_pwm = 13,
        .cooler_pwm = 15,

        .display_command = 18,  // could put this on 21 (SPI CSn)
        .display_reset = 23,
        .display_brightness_pwm = 17,
        .touch_interrupt = 24,
        .touch_reset = 25,
};

constexpr std::initializer_list<GPIO> PINS_RESERVED_BOARD{PICO_DEFAULT_WS2812_PIN};

}  // namespace nevermore
