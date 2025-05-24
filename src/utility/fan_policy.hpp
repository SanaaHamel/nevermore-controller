#pragma once

#include "sdk/ble_data_types.hpp"
#include "sensors.hpp"
#include "utility/pid.hpp"
#include <chrono>

namespace nevermore {

using namespace std::literals::chrono_literals;

struct [[gnu::packed]] FanPolicyEnvironmental {
    using VOCIndex = nevermore::sensors::VOCIndex;

    // How long to keep spinning after `should_filter` returns `false`
    BLE::TimeSecond16 cooldown = 60 * 15;
    VOCIndex voc_passive_max = 250;  // <= max(intake, exhaust)  -> filthy in here; get scrubbin'
    VOCIndex voc_improve_min;        // <= (intake - exhaust)    -> things are improving, keep filtering

    // for now there's nothing to do; no additional constraints on any fields
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    [[nodiscard]] bool validate() const {
        return true;
    }

    struct Instance {
        using Clock = std::chrono::steady_clock;
        using PID = nevermore::PID<float>;

        FanPolicyEnvironmental const& params;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
        Clock::time_point last_update = Clock::time_point::min();
        Clock::time_point last_filter = Clock::time_point::min();
        PID::State<-1.f, 0.f> controller;

        // Stateful.
        // Returns fan power [0, 1] based on env state and policy parameters.
        [[nodiscard]] float operator()(
                nevermore::sensors::Sensors const& state, Clock::time_point now = Clock::now());
    };

    // NB: DANGER - `this` must outlive `instance`
    [[nodiscard]] constexpr Instance instance() const {
        return {*this};
    }
};

}  // namespace nevermore
