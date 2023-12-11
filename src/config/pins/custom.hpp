#pragma once

#include "config/pins.hpp"

// All of the following pins must be defined.
constexpr GPIO_Pin PIN_FAN_PWM = 0;
constexpr GPIO_Pin PIN_FAN_TACHOMETER = 0;
constexpr GPIO_Pin PIN_NEOPIXEL_DATA_IN = 0;

// A pin has at most 1 I2C function & bus, so no need to specify if the pin is SDA/SCL.
// You can enable multiple pins for the same bus if you wish, but that isn't recommended.
// PSA: Have a care soldering your headers. I apparently damaged my GP17 and GP18 pins.
constexpr GPIO_Pin PINS_I2C[] = {0, 0, 0, 0};

// must all be on the same SPI, only need clock & TX (for now)
constexpr GPIO_Pin PINS_DISPLAY_SPI[] = {0, 0, 0};
// cmd & rst can be any GPIO
constexpr GPIO_Pin PIN_DISPLAY_COMMAND = 0;
constexpr GPIO_Pin PIN_DISPLAY_RESET = 0;
constexpr GPIO_Pin PIN_DISPLAY_BRIGHTNESS = 0;
constexpr GPIO_Pin PIN_TOUCH_INTERRUPT = 0;
constexpr GPIO_Pin PIN_TOUCH_RESET = 0;

// These pins are reserved on the Pico W (e.g. the wireless chip).
// Feel free to customise for your board in question.
constexpr GPIO_Pin PINS_RESERVED_BOARD[]{23, 24, 25, 29};
