#pragma once

#include "FreeRTOS.h"  // IWYU pragma: keep
#include "sdk/timer.hpp"
#include "task.h"  // IWYU pragma: keep [vTaskDelay, and others]
#include <cassert>
#include <chrono>
#include <limits>
#include <ratio>

namespace nevermore {

constexpr auto TASK_DELAY_MAX = std::chrono::milliseconds((portMAX_DELAY / configTICK_RATE_HZ) * 1000);

struct TaskDelayArg {
    int64_t us = 0;

    consteval TaskDelayArg() = default;

    template <typename A, typename P>
    constexpr TaskDelayArg(std::chrono::duration<A, P> const& delay) {
        if (delay < std::chrono::duration<A, P>(0)) throw "delay negative";
        if (TASK_DELAY_MAX <= delay) throw "delay too long";

        us = std::chrono::duration_cast<std::chrono::duration<int64_t, std::micro>>(delay).count();
        if (delay.count() != 0 && us == 0) throw "delay too short";
    }
};

struct NonZeroTickDuration {
    TickType_t ticks;

    template <typename A, typename P>
    constexpr NonZeroTickDuration(std::chrono::duration<A, P> const& delay) {
        if (delay <= std::chrono::duration<A, P>(0)) throw "delay not positive";
        if (TASK_DELAY_MAX <= delay) throw "delay too long";

        auto ms = std::chrono::duration<double, std::milli>(delay);
        ticks = pdMS_TO_TICKS(ms.count());
        if (ticks == 0) throw "delay too short for tick hz";
    }

    constexpr operator TickType_t() const {
        return ticks;
    }
};

template <typename A, typename Ratio>
consteval TickType_t to_ticks_safe(std::chrono::duration<A, Ratio> delay, bool allow_underflow = false) {
    using namespace std::literals::chrono_literals;

    auto delay_ms = std::chrono::duration_cast<std::chrono::duration<int64_t, std::milli>>(delay);
    if (delay_ms.count() <= 0) throw "invalid delay value";
    if (std::numeric_limits<TickType_t>::max() < delay_ms.count()) throw "invalid delay value";

    auto delay_ticks = pdMS_TO_TICKS(delay_ms / 1ms);
    if (!allow_underflow && delay_ticks == 0) throw "delay too small for tick rate";

    return delay_ticks;
}

template <TaskDelayArg delay>
void task_delay() {
    if constexpr (delay.us == 0) return;

    constexpr auto ticks = pdMS_TO_TICKS(delay.us / 1000);
    if constexpr (ticks <= 0)  // too small for task-delay -> busy wait instead
        busy_wait(std::chrono::microseconds(delay.us));
    else
        vTaskDelay(ticks);
}

inline void task_delay(TaskDelayArg delay) {
    if (delay.us == 0) return;

    auto ticks = pdMS_TO_TICKS(delay.us / 1000);
    if (ticks <= 0)  // too small for task-delay -> busy wait instead
        busy_wait(std::chrono::microseconds(delay.us));
    else
        vTaskDelay(ticks);
}

}  // namespace nevermore
