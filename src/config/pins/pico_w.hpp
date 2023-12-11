// Classic Pico W.

#pragma once

#include "config/pins.hpp"

constexpr GPIO_Pin PIN_FAN_PWM = 13;
constexpr GPIO_Pin PIN_FAN_TACHOMETER = 15;
constexpr GPIO_Pin PIN_NEOPIXEL_DATA_IN = 12;

constexpr GPIO_Pin PINS_I2C[] = {
        20, 21,  // I2C 0
        18, 19,  // I2C 1
};

constexpr GPIO_Pin PINS_DISPLAY_SPI[] = {2, 3, 4};
constexpr GPIO_Pin PIN_DISPLAY_COMMAND = 5;
constexpr GPIO_Pin PIN_DISPLAY_RESET = 6;
constexpr GPIO_Pin PIN_DISPLAY_BRIGHTNESS = 7;
constexpr GPIO_Pin PIN_TOUCH_INTERRUPT = 8;
constexpr GPIO_Pin PIN_TOUCH_RESET = 9;

constexpr GPIO_Pin PINS_RESERVED_BOARD[]{23, 24, 25, 29};
