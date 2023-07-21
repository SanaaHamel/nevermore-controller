#include "async_sensor.hpp"
#include "utility/task.hpp"
#include <chrono>
#include <cstdio>

using namespace std;

namespace nevermore::sensors {

namespace {

constexpr uint32_t SENSOR_STACK_DEPTH = 1024;

}

void SensorPeriodic::start() {
    if (task) return;  // already started

    // honestly coroutines would probably be nicer than allocating big stacks...
    auto go = [](void* self_) {
        auto* self = reinterpret_cast<SensorPeriodic*>(self_);
        auto last_wake = xTaskGetTickCount();
        for (;;) {
            self->read();

            auto delay_ticks = pdMS_TO_TICKS(self->update_period() / 1ms);
            xTaskDelayUntil(&last_wake, delay_ticks);
        }
    };

    task = Task(go, name(), SENSOR_STACK_DEPTH, this, Priority::Sensors);
}

void SensorPeriodic::stop() {
    task = {};
}

}  // namespace nevermore::sensors
