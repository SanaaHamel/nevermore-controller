// https://www.waveshare.com/product/rp2040-zero.htm

#pragma once

#include "boards/waveshare_rp2040_zero.h"
#include "config/pins.hpp"
#include "pico.h"  // IWYU pragma: keep for `WAVESHARE_RP2040_ZERO`
#include <optional>

#ifndef WAVESHARE_RP2040_ZERO
#error "`PICO_BOARD` is likely incorrect, `WAVESHARE_RP2040_ZERO` is not defined"
#endif

// Available as tiny impossible to solder pads on the back:
//   17, 18, 19, 20, 21, 22, 23, 24, 25

constexpr GPIO_Pin PIN_FAN_PWM = 13;
constexpr GPIO_Pin PIN_FAN_TACHOMETER = 15;
constexpr GPIO_Pin PIN_NEOPIXEL_DATA_IN = 12;
constexpr std::optional<GPIO_Pin> PIN_PHOTOCATALYTIC_PWM;

// This layout sucks, but we're rather constrained.
constexpr GPIO_Pin PINS_I2C[] = {
        8, 9,    // I2C 0
        26, 27,  // I2C 1
};

constexpr GPIO_Pin PINS_DISPLAY_SPI[] = {2, 3, 4};
constexpr GPIO_Pin PIN_DISPLAY_COMMAND = 5;
constexpr GPIO_Pin PIN_DISPLAY_RESET = 6;
constexpr GPIO_Pin PIN_DISPLAY_BRIGHTNESS = 7;
constexpr GPIO_Pin PIN_TOUCH_INTERRUPT = 28;
constexpr GPIO_Pin PIN_TOUCH_RESET = 29;

constexpr GPIO_Pin PINS_RESERVED_BOARD[]{PICO_DEFAULT_WS2812_PIN};
