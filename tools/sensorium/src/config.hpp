#pragma once

#include <chrono>

namespace nevermore {

using namespace std::literals::chrono_literals;

// period between sensor polls (in sec)
// SGP40's VOC Index algo is calibrated for 1s and 10s update rates,
// and has a minimum update rate of 0.5s.
constexpr auto SENSOR_UPDATE_PERIOD = 1s;
static_assert(0.5s <= SENSOR_UPDATE_PERIOD,
        "SENSOR_UPDATE_PERIOD too low, SGP40 needs at least 0.5s between measures.");

constexpr auto ADVERTISE_INTERVAL_MIN = 300ms;
constexpr auto ADVERTISE_INTERVAL_MAX = 500ms;

// AGS10 is slooooooooow and only does 15k
constexpr uint32_t I2C_BAUD_RATE_SENSOR_MAX = 15 * 1000;

}  // namespace nevermore
