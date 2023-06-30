#pragma once

#include "config.hpp"
#include "pico/async_context.h"

struct Sensor {
    virtual ~Sensor() = default;

    [[nodiscard]] virtual char const* name() const = 0;
};

// A sensor that schedules itself for periodic updates via an async context.
// Useful for sensors that take a long time (10ms+) to measure/respond.
// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
struct SensorPeriodic : Sensor {
    SensorPeriodic() = default;
    SensorPeriodic(SensorPeriodic const&) = delete;  // copying is almost certainly a mistake
    SensorPeriodic(SensorPeriodic&&) = delete;       // not safe to move b/c we're pinned once registered

    [[nodiscard]] virtual std::chrono::milliseconds update_period() const {
        return SENSOR_UPDATE_PERIOD;
    }

    void register_(async_context_t& context) {
        update_enqueue(context, 0ms);
    }

protected:
    virtual void update(async_context_t&);  // default impl is to await then enqueue a new update
    virtual void read() = 0;

    void update_enqueue(async_context_t&, std::chrono::steady_clock::duration update_duration);

private:
    static void update_dispatcher(async_context_t*, async_work_on_timeout*);

    async_at_time_worker_t update_task{.do_work = update_dispatcher, .user_data = this};
};

struct SensorDelayedResponse : SensorPeriodic {
    using SensorPeriodic::SensorPeriodic;

protected:
    // calls `issue()`, and if successful enqueues a `read`
    void update(async_context_t&) final;

    [[nodiscard]] virtual bool issue() = 0;
    [[nodiscard]] virtual std::chrono::milliseconds read_delay() const = 0;

private:
    static void read_dispatcher(async_context_t*, async_work_on_timeout*);

    std::chrono::steady_clock::time_point update_bgn{};
    async_at_time_worker_t read_task{.do_work = read_dispatcher, .user_data = this};
};
