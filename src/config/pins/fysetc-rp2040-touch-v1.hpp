// Fysetc's RP2040 Touch board for Mini

#pragma once

#include "config/pins.hpp"
#include "pico.h"  // IWYU pragma: keep for `FYSETC_RP2040_TOUCH_V1`
#include <initializer_list>

#ifndef FYSETC_RP2040_TOUCH_V1
#error "`PICO_BOARD` is likely incorrect, `FYSETC_RP2040_TOUCH_V1` is not defined"
#endif

// Disable persistence until the pins are finalised.
#undef NEVERMORE_SETTINGS_PERSISTENCE
#define NEVERMORE_SETTINGS_PERSISTENCE 0

namespace nevermore {

constexpr Pins PINS_DEFAULT{
        .i2c{
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::intake, .clock = 17, .data = 16},
                Pins::BusI2C{.kind = Pins::BusI2C::Kind::exhaust, .clock = 7, .data = 6},
        },

        .spi{
                Pins::BusSPI{.kind = Pins::BusSPI::Kind::display,
                        .clock = FYSETC_RP2040_TOUCH_V1_LCD_SCLK_PIN,
                        .send = FYSETC_RP2040_TOUCH_V1_LCD_TX_PIN,
                        .select = FYSETC_RP2040_TOUCH_V1_LCD_CS_PIN},
        },

        .fan_pwm = {14, 28},
        .fan_tachometer = {15, 27},
        .neopixel_data = {26},
        .photocatalytic_pwm = {},

        .display_command = FYSETC_RP2040_TOUCH_V1_LCD_DC_PIN,
        .display_reset = FYSETC_RP2040_TOUCH_V1_LCD_RST_PIN,
        .display_brightness_pwm = FYSETC_RP2040_TOUCH_V1_LCD_BL_PIN,
        .touch_interrupt = FYSETC_RP2040_TOUCH_V1_TOUCH_INTERRUPT_PIN,
        .touch_reset = FYSETC_RP2040_TOUCH_V1_TOUCH_RST_PIN,
};

constexpr std::initializer_list<GPIO> PINS_RESERVED_BOARD{};

}  // namespace nevermore
