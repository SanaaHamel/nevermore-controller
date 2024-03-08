// Classic Pico W.

#pragma once

#include "config/pins.hpp"
#include "pico.h"  // IWYU pragma: keep for `RASPBERRYPI_PICO_W`
#include <initializer_list>

#ifndef RASPBERRYPI_PICO_W
#error "`PICO_BOARD` is likely incorrect, `RASPBERRYPI_PICO_W` is not defined"
#endif

// Exposed GPIOs: [0, 22], [26, 28]

namespace nevermore {

constexpr Pins PINS_DEFAULT{
        .i2c{
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::intake, .clock = 21, .data = 20},
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::exhaust, .clock = 19, .data = 18},
        },

        .spi{
                Pins::BusSPI{.kind = Pins::BusSPI::Kind::display, .clock = 2, .send = 3, .recv = 4},
        },

        .fan_pwm = {13},
        .fan_tachometer = {15},
        .neopixel_data = {12},
        .photocatalytic_pwm = {10},

        .display_command = 5,
        .display_reset = 6,
        .display_brightness_pwm = 7,
        .touch_interrupt = 8,
        .touch_reset = 9,
};

// wireless SPI CS on 25
constexpr std::initializer_list<GPIO> PINS_RESERVED_BOARD{
        CYW43_PIN_WL_REG_ON, CYW43_PIN_WL_HOST_WAKE, 25, PICO_VSYS_PIN};

}  // namespace nevermore
