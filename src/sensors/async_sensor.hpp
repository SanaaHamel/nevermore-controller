#pragma once

#include "config.hpp"
#include "utility/task.hpp"

namespace nevermore::sensors {

struct Sensor {
    virtual ~Sensor() = default;

    [[nodiscard]] virtual char const* name() const = 0;

    virtual void reset_calibration() {};
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

    virtual void start();
    virtual void stop();

protected:
    virtual void read() = 0;

    Task task;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes)
};

}  // namespace nevermore::sensors
