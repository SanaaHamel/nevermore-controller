#pragma once

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

////////////////////////////
// Other configurable stuff.
////////////////////////////

// period between sensor polls (in sec)
// SGP40's VOC Index algo is calibrated for 1s and 10s update rates,
// and has a minimum update rate of 0.5s.
constexpr auto SENSOR_UPDATE_PERIOD = 1s;
static_assert(0.5s <= SENSOR_UPDATE_PERIOD,
        "SENSOR_UPDATE_PERIOD too low, SGP40 needs at least 0.5s between measures.");

constexpr auto ADVERTISE_INTERVAL_MIN = 1000ms;
constexpr auto ADVERTISE_INTERVAL_MAX = 1000ms;
static_assert(ADVERTISE_INTERVAL_MIN <= ADVERTISE_INTERVAL_MAX);

// SGP40 supports std-mode 100 kHz and fast mode 400 kHz
// HTU21D supports "up to 400 kHz"
constexpr uint32_t I2C_BAUD_RATE_HZ = 400 * 1000;
static_assert(100 * 1000 <= I2C_BAUD_RATE_HZ, "I2C_BAUD_RATE_HZ minimum is 100 kHz");
static_assert(I2C_BAUD_RATE_HZ <= 400 * 1000, "I2C_BAUD_RATE_HZ too high for some sensors");

constexpr GPIO_Pin PIN_MAX = 30;  // DO NOT ALTER.
