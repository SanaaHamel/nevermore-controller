// https://www.waveshare.com/product/rp2040-lcd-1.28.htm

#pragma once

#include "config/pins.hpp"
#include "pico.h"  // IWYU pragma: keep for `WAVESHARE_RP2040_LCD_1_28`
#include <initializer_list>

#ifndef WAVESHARE_RP2040_LCD_1_28
#error "`PICO_BOARD` is likely incorrect, `WAVESHARE_RP2040_LCD_1_28` is not defined"
#endif

// Exposed GPIOs: [0, 29]
// NB: Some exposed GPIOs are bound to internal hardware.
// IMU interrupt 1, 2 are on 23, 24
// ADC_BAT on 29

namespace nevermore {

constexpr Pins PINS_DEFAULT{
        .i2c{
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::intake, .clock = 5, .data = 4},
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::exhaust, .clock = 7, .data = 6},
        },

        .spi{
                Pins::BusSPI{
                        .kind = Pins::BusSPI::Kind::display,
                        .clock = WAVESHARE_RP2040_LCD_SCLK_PIN,
                        .send = WAVESHARE_RP2040_LCD_TX_PIN,
                        .recv = {},  // doesn't bind 12 to RX, unlike touch variant
                        .select = WAVESHARE_RP2040_LCD_CS_PIN,
                },
        },

        .fan_pwm = {14},
        .fan_tachometer = {13},
        .neopixel_data = {15},
        .photocatalytic_pwm = {2},

        .display_command = WAVESHARE_RP2040_LCD_DC_PIN,
        .display_reset = WAVESHARE_RP2040_LCD_RST_PIN,
        .display_brightness_pwm = WAVESHARE_RP2040_LCD_BL_PIN,
        .touch_interrupt = {},
        .touch_reset = {},
};

constexpr std::initializer_list<GPIO> PINS_RESERVED_BOARD{};

}  // namespace nevermore
