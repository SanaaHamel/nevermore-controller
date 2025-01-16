#pragma once

#include "FreeRTOS.h"  // IWYU pragma: keep
#include "sdk/task.hpp"
#include "task.h"  // IWYU pragma: keep
#include <cassert>

namespace nevermore {

// NOLINTNEXTLINE(performance-enum-size)
enum class Priority : UBaseType_t {
    Idle = 0,
    Display,
    Sensors,
    Communication = CYW43_TASK_PRIORITY,  // higher priority
    Startup,
    USBD,
    WatchdogUpdate,  // highest
};
static_assert(UBaseType_t(Priority::WatchdogUpdate) < configMAX_PRIORITIES);
static_assert(UBaseType_t(Priority::WatchdogUpdate) == configTIMER_TASK_PRIORITY);

struct Task {
    Task() = default;
    Task(Task const&) = delete;
    Task& operator=(Task const&) = delete;

    Task(Task&& rhs) noexcept : task(rhs.release()) {}
    Task& operator=(Task&& rhs) noexcept {
        if (task != rhs.task) {
            reset(rhs.release());
        }

        return *this;
    }

    explicit Task(TaskHandle_t task) : task(task) {}

    Task(void (*go)(void*), void* param, char const* name, Priority priority, uint32_t stack_depth,
            UBaseType_t affinity_mask = tskNO_AFFINITY) {
        xTaskCreateAffinitySet(go, name, stack_depth, param, UBaseType_t(priority), affinity_mask, &task);
    }

    Task(void (*go)(), char const* name, Priority priority, uint32_t stack_depth,
            UBaseType_t affinity_mask = tskNO_AFFINITY) {
        xTaskCreateAffinitySet([](void* go) { reinterpret_cast<void (*)()>(go)(); }, name, stack_depth,
                reinterpret_cast<void*>(go), UBaseType_t(priority), affinity_mask, &task);
    }

    template <typename A>
    Task(A (*go)(), char const* name, Priority priority, uint32_t stack_depth,
            UBaseType_t affinity_mask = tskNO_AFFINITY) {
        xTaskCreateAffinitySet([](void* go) { reinterpret_cast<A (*)()>(go)(); }, name, stack_depth,
                reinterpret_cast<void*>(go), UBaseType_t(priority), affinity_mask, &task);
    }

    ~Task() {
        reset();
    }

    explicit operator bool() const {
        return !!task;
    }

    void suspend() {
        if (task) vTaskSuspend(task);
    }

    void resume() {
        if (task) vTaskResume(task);
    }

    bool reset(TaskHandle_t new_task = {}) {
        assert(new_task != task || !task);
        if (task) vTaskDelete(task);

        task = new_task;
        return true;
    }

    TaskHandle_t release() {
        TaskHandle_t task = this->task;
        this->task = nullptr;
        return task;
    }

    [[nodiscard]] operator TaskHandle_t() {
        return task;
    }

private:
    TaskHandle_t task{};
};

constexpr auto mk_task(char const* name, Priority priority, uint32_t stack_depth,
        UBaseType_t affinity_set = tskNO_AFFINITY) {
    return [=](auto go) { return Task(go, name, priority, stack_depth, affinity_set); };
}

template <NonZeroTickDuration period>
[[noreturn]] auto periodic(auto&& go) {
    auto last_wake = xTaskGetTickCount();
    for (;;) {
        go();
        xTaskDelayUntil(&last_wake, period);
    }
}

}  // namespace nevermore
