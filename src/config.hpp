#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>

using namespace std::literals::chrono_literals;

////////////////////////////
// PIN ASSIGNMENTS
////////////////////////////

using GPIO_Pin = uint8_t;

constexpr GPIO_Pin PIN_FAN_PWM = 13;
constexpr GPIO_Pin PIN_FAN_TACHOMETER = 15;
constexpr GPIO_Pin PIN_NEOPIXEL_DATA_IN = 12;

// A pin has at most 1 I2C function & bus, so no need to specify if the pin is SDA/SCL.
// You can enable multiple pins for the same bus if you wish, but that isn't recommended.
// PSA: Have a care soldering your headers. I apparently damaged my GP17 and GP18 pins.
constexpr GPIO_Pin PINS_I2C[] = {
        20, 21,  // I2C 0
        18, 19,  // I2C 1
};

// must all be on the same SPI, only need clock & TX
constexpr GPIO_Pin PINS_DISPLAY_SPI[] = {2, 3, 4};
// cmd & rst can be any GPIO
constexpr GPIO_Pin PIN_DISPLAY_COMMAND = 5;
constexpr GPIO_Pin PIN_DISPLAY_RESET = 6;
constexpr GPIO_Pin PIN_DISPLAY_BRIGHTNESS = 7;
constexpr GPIO_Pin PIN_TOUCH_INTERRUPT = 8;
constexpr GPIO_Pin PIN_TOUCH_RESET = 9;

////////////////////////////
// Other configurable stuff.
////////////////////////////

// period between sensor polls (in sec)
// SGP40's VOC Index algo is calibrated for 1s and 10s update rates,
// and has a minimum update rate of 0.5s.
constexpr auto SENSOR_UPDATE_PERIOD = 1s;
static_assert(0.5s <= SENSOR_UPDATE_PERIOD,
        "SENSOR_UPDATE_PERIOD too low, SGP40 needs at least 0.5s between measures.");

constexpr auto ADVERTISE_INTERVAL_MIN = 300ms;
constexpr auto ADVERTISE_INTERVAL_MAX = 500ms;

// Set to desired baud rate. Most sensors support 400 kbit/s.
// Compile time error checks will trigger if set too high for included sensors.
constexpr uint32_t I2C_BAUD_RATE = 400 * 1000;
// TODO:  Find what's the actual max baud rate for a GC9A01.
//        So far I've ran all the way to max (125M).
constexpr uint32_t SPI_BAUD_RATE_DISPLAY = 125'000'000 / 2;

////////////////////////////////////////////////////
//         End of Configurable Settings.
// Everything below is not of interest to end users.
////////////////////////////////////////////////////

constexpr GPIO_Pin PIN_MAX = 30;  // DO NOT ALTER.

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
