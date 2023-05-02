#pragma once

#include "config.hpp"
#include "gatt/environmental.hpp"
#include "pico/async_context.h"
#include "sdk/ble_data_types.hpp"
#include <tuple>

struct Sensor {
    using Data = std::tuple<BLE::Temperature&, BLE::Humidity&, BLE::Pressure&, EnvironmentService::VOCIndex&>;

    virtual ~Sensor() = default;

    [[nodiscard]] virtual char const* name() const = 0;
};

// A sensor that schedules itself for periodic updates via an async context.
// Useful for sensors that take a long time (10ms+) to measure/respond.
struct SensorPeriodic : Sensor {
    [[nodiscard]] virtual std::chrono::milliseconds update_period() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(SENSOR_UPDATE_PERIOD);
    }

    void register_(async_context_t& context) {
        update_enqueue(context, 0ms);
    }

protected:
    virtual void update(async_context_t&);  // default impl is to await then enqueue a new update
    virtual void read() = 0;

    void update_enqueue(async_context_t&, std::chrono::steady_clock::duration update_duration);

private:
    static void dispatcher(async_context_t*, async_work_on_timeout*);

    async_at_time_worker_t update_task{.do_work = dispatcher, .user_data = this};
};

struct SensorDelayedResponse : SensorPeriodic {
    using SensorPeriodic::SensorPeriodic;

protected:
    // calls `issue()`, and if successfull enqueues a `read`
    void update(async_context_t&) final;

    [[nodiscard]] virtual bool issue() = 0;
    [[nodiscard]] virtual std::chrono::milliseconds read_delay() const = 0;

private:
    static void read_dispatcher(async_context_t*, async_work_on_timeout*);

    std::chrono::steady_clock::time_point update_bgn{};
    async_at_time_worker_t read_task{.do_work = read_dispatcher, .user_data = this};
};
