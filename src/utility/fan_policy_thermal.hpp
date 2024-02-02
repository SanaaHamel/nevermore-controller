#pragma once

#include "sdk/ble_data_types.hpp"
#include <cassert>
#include <cmath>
#include <optional>

namespace nevermore {

struct [[gnu::packed]] FanPolicyThermal {
    BLE::Temperature min = 50;             // not-known -> disallowed
    BLE::Temperature max = 60;             // not-known -> disallowed
    BLE::Percentage16_10 coefficient = 0;  // not-known -> disabled (equiv to 100%)

    [[nodiscard]] bool validate() const {
        if (min == BLE::NOT_KNOWN) return false;
        if (max == BLE::NOT_KNOWN) return false;
        if (max < min) return false;
        if (coefficient < 0 || 100 < coefficient) return false;
        return true;
    }

    //  [0, 1] or None
    [[nodiscard]] std::optional<double> percent(BLE::Temperature current) const {
        if (current == BLE::NOT_KNOWN) return {};
        if (max <= current) return 1;  // trivially 0, also handles case where max <= min
        if (current < min) return 0;   // trivially 1, below limiter kicks in

        auto range = max.value_or(0) - min.value_or(0);
        auto x = current.value_or(0) - min.value_or(0);
        assert(0 < max && "`0 < range` due to above checks");
        return x / range;
    }

    [[nodiscard]] double operator()(BLE::Temperature current) const {
        auto coeff = coefficient.value_or(100);
        if (coeff == 100) return 1;

        auto perc = percent(current);
        if (!perc) return 1;

        return std::lerp(1, coeff / 100, *perc);
    }
};

}  // namespace nevermore
