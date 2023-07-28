#pragma once

#include "FreeRTOS.h"  // IWYU pragma: keep
#include "portmacro.h"
#include "sdk/task.hpp"
#include "task.h"  // IWYU pragma: keep
#include <cassert>

namespace nevermore {

enum class Priority : UBaseType_t {
    Idle = 0,
    Display,
    Sensors,
    Communication = CYW43_TASK_PRIORITY,  // higher priority
    Startup,
};

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

    Task(void (*go)(void*), char const* name, uint32_t stack_depth, void* param, Priority priority) {
        xTaskCreate(go, "", stack_depth, param, UBaseType_t(priority), &task);
    }

    Task(void (*go)(), char const* name, Priority priority, uint32_t stack_depth) {
        xTaskCreate([](void* go) { reinterpret_cast<void (*)()>(go)(); }, "", stack_depth,
                reinterpret_cast<void*>(go), UBaseType_t(priority), &task);
    }

    template <typename A>
    Task(A (*go)(), char const* name, Priority priority, uint32_t stack_depth) {
        xTaskCreate([](void* go) { reinterpret_cast<A (*)()>(go)(); }, "", stack_depth,
                reinterpret_cast<void*>(go), UBaseType_t(priority), &task);
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

private:
    TaskHandle_t task{};
};

constexpr auto mk_task(char const* name, Priority priority, uint32_t stack_depth) {
    return [=](auto go) { return Task(go, name, priority, stack_depth); };
}

template <typename A, typename Period>
consteval auto periodic(std::chrono::duration<A, Period> delay) {
    auto delay_ticks = to_ticks_safe(delay);

    return [=](auto&& go) {
        auto last_wake = xTaskGetTickCount();
        for (;;) {
            go();
            xTaskDelayUntil(&last_wake, delay_ticks);
        }
    };
}

}  // namespace nevermore
