#pragma once

#include "sdk/ble_data_types.hpp"

namespace nevermore {

struct GPIO;

struct [[gnu::packed]] ServoRange {
    BLE::Percentage16_10 start = 0;
    BLE::Percentage16_10 end = 100;  // `end` may be <= `start`

    [[nodiscard]] constexpr bool validate() const {
        return start != BLE::NOT_KNOWN && end != BLE::NOT_KNOWN;
    }
};

void servo_set(GPIO pin, ServoRange const&, BLE::Percentage16_10 perc);

}  // namespace nevermore
