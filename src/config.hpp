#pragma once

#include "config/pins.hpp"  // IWYU pragma: keep
#include <chrono>

using namespace std::literals::chrono_literals;

// period between setting changes getting persisted
// special value: 0 to disable
constexpr auto SETTINGS_PERSIST_PERIOD = 10s;

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
