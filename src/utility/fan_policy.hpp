#pragma once

#include "sensors.hpp"
#include <chrono>

namespace nevermore {

using namespace std::literals::chrono_literals;

struct FanPolicyEnvironmental {
    using VOCIndex = nevermore::sensors::VOCIndex;

    // How long to keep spinning after `should_filter` returns `false`
    BLE::TimeSecond16 cooldown = 60 * 15;
    VOCIndex voc_passive_max = 125;  // <= max(intake, exhaust)  -> filthy in here; get scrubbin'
    VOCIndex voc_improve_min = 5;    // <= (intake - exhaust)    -> things are improving, keep filtering

    struct Instance {
        FanPolicyEnvironmental const& params;
        std::chrono::system_clock::time_point last_filter = std::chrono::system_clock::time_point::min();

        // Stateful.
        // Returns fan power [0, 1] based on env state and policy parameters.
        [[nodiscard]] float operator()(nevermore::sensors::Sensors const& state,
                std::chrono::system_clock::time_point now = std::chrono::system_clock::now());
    };

    // NB: DANGER - `this` must outlive `instance`
    [[nodiscard]] constexpr Instance instance() const {
        return {*this};
    }
};

}  // namespace nevermore
