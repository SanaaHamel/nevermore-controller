#include "async_sensor.hpp"
#include "sdk/async.hpp"
#include <chrono>
#include <cstdio>

using namespace std;

void SensorPeriodic::update_dispatcher(async_context_t* context, async_work_on_timeout* work) {
    reinterpret_cast<SensorPeriodic*>(work->user_data)->update(*context);
}

void SensorPeriodic::update(async_context_t& context) {
    auto const bgn = chrono::steady_clock::now();
    read();
    auto const end = chrono::steady_clock::now();
    update_enqueue(context, end - bgn);
}

void SensorPeriodic::update_enqueue(
        async_context_t& context, chrono::steady_clock::duration update_duration) {
    // printf("sensor update enqueue - %s - duration %d us\n", name(), int(update_duration / 1us));
    auto const delay_ms = max<int64_t>((update_period() - update_duration) / 1ms, 0);
    async_using(context, [&]() { async_context_add_at_time_worker_in_ms(&context, &update_task, delay_ms); });
}

void SensorDelayedResponse::update(async_context_t& context) {
    update_bgn = chrono::steady_clock::now();

    if (!issue()) {
        // Something went wrong during sensor cmd issue -> skip this update cycle and hope it is temporary.
        // TODO: Should we re-issue ASAP instead of trying to follow the normal update period?
        printf("SensorDelayedResponse::update - %s - issue failed\n", name());
        update_enqueue(context, chrono::steady_clock::now() - update_bgn);
        return;
    }

    async_using(context,
            [&]() { async_context_add_at_time_worker_in_ms(&context, &read_task, read_delay() / 1ms); });
}

void SensorDelayedResponse::read_dispatcher(async_context_t* context, async_work_on_timeout* work) {
    auto& self = *reinterpret_cast<SensorDelayedResponse*>(work->user_data);
    self.read();
    self.update_enqueue(*context, chrono::steady_clock::now() - self.update_bgn);
}
