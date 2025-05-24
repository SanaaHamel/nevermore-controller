#include "fan_policy.hpp"
#include "sdk/ble_data_types.hpp"
#include <chrono>
#include <cmath>
#include <utility>

using namespace std;
using namespace std::literals::chrono_literals;
using namespace BLE;
using nevermore::sensors::VOCIndex;

namespace nevermore {

namespace {

using PID = FanPolicyEnvironmental::Instance::PID;

// configurable
constexpr float FILTER_POWER_MINIMUM = 0.2f;
constexpr auto FAN_PID = PID::mk(0.01f, PID::Sec(60), PID::Sec(0.1));

// non-configurable
constexpr float VOC_NOMINAL_MAX = 200;

constexpr bool policy_voc_too_high(VOCIndex voc_passive_max, VOCIndex intake, VOCIndex exhaust) {
    return voc_passive_max <= max(intake.value_or(0), exhaust.value_or(0));
}

constexpr bool policy_voc_improving(VOCIndex voc_improve_min, VOCIndex intake, VOCIndex exhaust) {
    if (intake == NOT_KNOWN || exhaust == NOT_KNOWN) return false;  // Need a reading for both sensors.

    auto const voc_improvement = intake.value_or(0) - exhaust.value_or(0);
    return 0 < voc_improvement && voc_improve_min <= voc_improvement;
}

constexpr bool should_filter(FanPolicyEnvironmental const& params, VOCIndex intake, VOCIndex exhaust) {
    return policy_voc_too_high(params.voc_passive_max, intake, exhaust) ||
           policy_voc_improving(params.voc_improve_min, intake, exhaust);
}

enum class PolicyState { Idle, Filtering, Cooldown };
using enum PolicyState;

constexpr PolicyState evaluate(FanPolicyEnvironmental::Instance const& instance,
        nevermore::sensors::Sensors const& state, chrono::steady_clock::time_point now) {
    if (should_filter(instance.params, state.voc_index_intake, state.voc_index_exhaust)) return Filtering;

    auto cooldown_end =
            instance.last_filter + chrono::seconds(uint32_t(instance.params.cooldown.value_or(0)));
    if (now < cooldown_end) return Cooldown;

    return Idle;
}

}  // namespace

float FanPolicyEnvironmental::Instance::operator()(
        nevermore::sensors::Sensors const& state, Clock::time_point now) {
    auto const dt = now - last_update;
    last_update = now;

    // HACK: [100, 440] VOC index range is almost linear, no need to sigmoid^-1.
    //       We'll cheat and assume the rest is linear too.
    auto const voc_curr = float(max(state.voc_index_intake.value_or(0), state.voc_index_exhaust.value_or(0)));
    auto const voc_limit = max(VOC_NOMINAL_MAX, float(params.voc_passive_max.value_or(0)));
    auto const fan_power = -controller.update(FAN_PID, dt, voc_curr, voc_limit);

    switch (evaluate(*this, state, now)) {
    case Idle: return 0;
    case Cooldown: return max(FILTER_POWER_MINIMUM, fan_power);
    case Filtering: {
        last_filter = now;
        return max(FILTER_POWER_MINIMUM, fan_power);
    }
    }

    // Annoying. GCC's case analysis fails to detect that switch is total.
    std::unreachable();
}

// Policy Tests

// Initial state should be off if no sensors.
static_assert(evaluate(FanPolicyEnvironmental{}.instance(), {}, {}) == Idle);

// VOC-exceeds-limits case
static_assert(policy_voc_too_high(1, 1, 1));                   // barely
static_assert(policy_voc_too_high(1, 1, NOT_KNOWN));           // barely
static_assert(policy_voc_too_high(1, NOT_KNOWN, 1));           // barely
static_assert(!policy_voc_too_high(2, 1, 1));                  // not enough
static_assert(!policy_voc_too_high(NOT_KNOWN, 500, 500));      // disabled
static_assert(!policy_voc_too_high(1, NOT_KNOWN, NOT_KNOWN));  // sensors not connected

// VOC-improved-by-filtering case
static_assert(policy_voc_improving(2, 3, 1));                   // barely
static_assert(!policy_voc_improving(2, 3, 2));                  // not enough
static_assert(!policy_voc_improving(2, 3, NOT_KNOWN));          // basic case - need both sensors
static_assert(!policy_voc_improving(2, NOT_KNOWN, 1));          // basic case - need both sensors
static_assert(!policy_voc_improving(NOT_KNOWN, 0, 0));          // disabled
static_assert(!policy_voc_improving(NOT_KNOWN, 1, 1));          // disabled
static_assert(!policy_voc_improving(NOT_KNOWN, 2, 1));          // disabled
static_assert(!policy_voc_improving(NOT_KNOWN, 1, 2));          // disabled
static_assert(!policy_voc_improving(2, NOT_KNOWN, NOT_KNOWN));  // sensors not connected
static_assert(!policy_voc_improving(NOT_KNOWN, 68, 76));        // disabled
static_assert(!policy_voc_improving(1, 68, 76));                // disabled

namespace {

using State = decltype(FanPolicyEnvironmental::Instance::controller);
[[maybe_unused]] constexpr float pid_sim(float voc, float voc_dt = 0) {
    constexpr float dt = 0.1f;

    State state;
    float v;
    for (float i = 0; i < 60; i += dt) {
        v = state.update(FAN_PID, PID::Sec(dt), voc, VOC_NOMINAL_MAX);
        voc += voc_dt * dt;
    }
    return v;
}

// PI tests
static_assert(pid_sim(0) == State::max);                // < target
static_assert(pid_sim(VOC_NOMINAL_MAX) == State::max);  // == target
static_assert(pid_sim(500) == State::min);              // >> target

// D tests
static_assert(pid_sim(VOC_NOMINAL_MAX + 1, 0.f) < State::max);      // just above target, active
static_assert(State::min < pid_sim(VOC_NOMINAL_MAX + 1, 0.f));      // just above target, but not saturated
static_assert(pid_sim(VOC_NOMINAL_MAX + 1, -0.05f) == State::max);  // just above target, but going down
static_assert(pid_sim(500, -10.f) == State::max);  // very high and quickly decreasing, but going down

}  // namespace

}  // namespace nevermore
