#pragma once

#include "FreeRTOS.h"  // IWYU pragma: keep
#include "sdk/timer.hpp"
#include "task.h"  // IWYU pragma: keep [vTaskDelay, and others]
#include <cassert>
#include <chrono>
#include <limits>
#include <ratio>

namespace nevermore {

template <typename A, typename Ratio>
consteval TickType_t to_ticks_safe(std::chrono::duration<A, Ratio> delay, bool allow_underflow = true) {
    using namespace std::literals::chrono_literals;

    auto delay_ms = std::chrono::duration_cast<std::chrono::duration<int64_t, std::milli>>(delay);
    if (delay_ms.count() <= 0) throw "invalid delay value";
    if (std::numeric_limits<TickType_t>::max() < delay_ms.count()) throw "invalid delay value";

    auto delay_ticks = pdMS_TO_TICKS(delay_ms / 1ms);
    if (!allow_underflow && delay_ticks == 0) throw "tick rate too large for delay";

    return delay_ticks;
}

template <typename A, typename Period>
void task_delay(std::chrono::duration<A, Period> delay) {
    if (delay <= std::chrono::duration<A, Period>(0)) return;

    auto delay_ms = std::chrono::duration_cast<std::chrono::duration<int64_t, std::milli>>(delay);
    if (delay_ms.count() <= 0) {  // too small for task-delay -> busy wait instead
        busy_wait(delay);
        return;
    }

    assert(delay_ms.count() <= std::numeric_limits<TickType_t>::max());
    vTaskDelay(pdMS_TO_TICKS(delay_ms.count()));
}

}  // namespace nevermore
