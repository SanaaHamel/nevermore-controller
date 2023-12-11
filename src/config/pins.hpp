#pragma once

#include <algorithm>
#include <array>  // for `begin`/`end`
#include <cstdint>

using GPIO_Pin = uint8_t;

constexpr GPIO_Pin PIN_MAX = 30;  // DO NOT ALTER.

#ifdef NEVERMORE_CONFIG
#include NEVERMORE_CONFIG  // IWYU pragma: keep
#else
#include "config/pins/custom.hpp"  // IWYU pragma: keep
#endif

template <typename F>
constexpr bool pins_forall(F&& go) {
    if (!std::all_of(std::begin(PINS_I2C), std::end(PINS_I2C), go)) return false;
    if (!(go(PIN_FAN_PWM) && go(PIN_FAN_TACHOMETER))) return false;
    if (!go(PIN_NEOPIXEL_DATA_IN)) return false;
    if (!std::all_of(std::begin(PINS_DISPLAY_SPI), std::end(PINS_DISPLAY_SPI), go)) return false;
    if (!go(PIN_DISPLAY_COMMAND)) return false;
    if (!go(PIN_DISPLAY_RESET)) return false;
    if (!go(PIN_DISPLAY_BRIGHTNESS)) return false;
    if (!go(PIN_TOUCH_INTERRUPT)) return false;
    if (!go(PIN_TOUCH_RESET)) return false;

    return true;
}

template <typename F>
constexpr bool pin_exists(F&& go) {
    return !pins_forall([&](auto p) { return !go(p); });
}
