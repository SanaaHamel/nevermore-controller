// https://www.waveshare.com/product/rp2040-touch-lcd-1.28.htm

#pragma once

#include "config/pins.hpp"
#include "pico.h"  // IWYU pragma: keep for `WAVESHARE_RP2040_LCD_1_28`
#include <initializer_list>

// Don't have an official board def for the touch variant, but it's
// almost identical to the non-touch waveshare LCD board.
#ifndef WAVESHARE_RP2040_LCD_1_28
#error "`PICO_BOARD` is likely incorrect, `WAVESHARE_RP2040_LCD_1_28` is not defined"
#endif

// Exposed GPIOs: [16, 18], [26, 28]
// IMU interrupt 1, 2 are on 23, 24
// ADC_BAT on 29

namespace nevermore {

constexpr Pins PINS_DEFAULT{
        .i2c{
                // hardwired to touch/IMU
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::generic, .clock = 7, .data = 6},
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::exhaust, .clock = 17, .data = 16},
        },

        .spi{
                Pins::BusSPI{
                        .kind = Pins::BusSPI::Kind::display,
                        .clock = WAVESHARE_RP2040_LCD_SCLK_PIN,
                        .send = WAVESHARE_RP2040_LCD_TX_PIN,
                        // FUTURE WORK: schematic has line going to a SDO on the LCD board, but GC9A01
                        //              doesn't talk back? investigate.
                        .recv = 12,
                        .select = WAVESHARE_RP2040_LCD_CS_PIN,
                },
        },

        .fan_pwm = {28},
        .fan_tachometer = {27},
        .neopixel_data = {26},
        .photocatalytic_pwm = {},

        .display_command = WAVESHARE_RP2040_LCD_DC_PIN,
        .display_reset = 13,  // unlike non-touch which has it on 12
        .display_brightness_pwm = WAVESHARE_RP2040_LCD_BL_PIN,
        .touch_interrupt = 21,
        .touch_reset = 22,
};

constexpr std::initializer_list<GPIO> PINS_RESERVED_BOARD{};

}  // namespace nevermore
