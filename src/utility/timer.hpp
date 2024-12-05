#pragma once

#include "FreeRTOS.h"  // IWYU pragma: keep
#include "sdk/task.hpp"
#include "timers.h"  // IWYU pragma: keep

namespace nevermore {

consteval auto mk_timer(char const* name, NonZeroTickDuration ticks, bool one_shot = false) {
    return [=](TimerCallbackFunction_t go) {
        auto x = xTimerCreate(name, ticks, one_shot ? pdFALSE : pdTRUE, reinterpret_cast<void*>(go), go);
        if (x) xTimerStart(x, 0);
        return x;
    };
}

}  // namespace nevermore
