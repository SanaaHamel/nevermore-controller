#pragma once

#include "FreeRTOS.h"  // IWYU pragma: keep
#include "sdk/task.hpp"
#include <algorithm>

#if NEVERMORE_PICO_W_BT
#include "btstack_run_loop.h"
#else
#include "timers.h"  // IWYU pragma: keep
#endif

namespace nevermore {

// Sometimes you need to do stuff that might interact with the CYW43.
// You'll want that running in the bluetooth comm context, but that doesn't exist
// for all build variants. This'll execute in that context if it exists, or
// as a standard FreeRTOS timer if it doesn't.
struct Cyw43TimerInstance {
#if NEVERMORE_PICO_W_BT
    btstack_timer_source_t bt;
#else
    TimerHandle_t handle;
#endif

    void start() {
#if NEVERMORE_PICO_W_BT
        bt.process(&bt);
#else
        xTimerStart(handle, 0);
#endif
    }
};

template <NonZeroTickDuration period>
consteval auto mk_cyw43_timer(char const* name) {
#if NEVERMORE_PICO_W_BT
    return [](void (*go)(btstack_timer_source_t*)) -> Cyw43TimerInstance {
        return {{
                .process =
                        [](btstack_timer_source_t* timer) {
                            reinterpret_cast<void (*)(btstack_timer_source_t*)>(timer->context)(timer);
                            constexpr auto delay = uint32_t(
                                    std::max(1.f, (period / float(configTICK_RATE_HZ)) * 1000));  // in ms
                            btstack_run_loop_set_timer(timer, delay);
                            btstack_run_loop_add_timer(timer);
                        },
                .context = reinterpret_cast<void*>(go),
        }};
    };
#else
    return [=](TimerCallbackFunction_t go) {
        return Cyw43TimerInstance{xTimerCreate(name, period, pdTRUE, reinterpret_cast<void*>(go), go)};
    };
#endif
}

}  // namespace nevermore
