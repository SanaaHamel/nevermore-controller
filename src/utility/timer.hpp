#pragma once

#include "FreeRTOS.h"  // IWYU pragma: keep
#include "sdk/task.hpp"
#include "timers.h"  // IWYU pragma: keep
#include <chrono>

namespace nevermore {

template <typename A, typename Period>
consteval auto mk_timer(char const* name, std::chrono::duration<A, Period> period, bool one_shot = false) {
    auto ticks = to_ticks_safe(period);
    return [=](TimerCallbackFunction_t go) {
        auto x = xTimerCreate(name, ticks, one_shot ? pdFALSE : pdTRUE, reinterpret_cast<void*>(go), go);
        if (x) xTimerStart(x, 0);
        return x;
    };
}

}  // namespace nevermore
