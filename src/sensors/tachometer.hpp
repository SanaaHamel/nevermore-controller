#pragma once

#include "async_sensor.hpp"
#include "sdk/pwm.hpp"
#include <chrono>
#include <cstdint>

namespace nevermore::sensors {

using namespace std::literals::chrono_literals;

struct Tachometer final : SensorPeriodic {
    // DELTA BFB0712H spec sheet says an RPM of 2900 -> ~49 rev/s
    // Honestly at this low a rate it might be worth just using
    // an interrupt instead of screwing with a PWM slice...
    constexpr static auto TACHOMETER_READ_PERIOD = SENSOR_UPDATE_PERIOD;
    static_assert(100ms <= TACHOMETER_READ_PERIOD && "need at least 100ms to get a good sampling");

    Tachometer(GPIO_Pin pin, uint32_t pulses_per_revolution = 1)
            : slice_num(pwm_gpio_to_slice_num_(pin)), pulses_per_revolution(pulses_per_revolution) {
        assert(pwm_gpio_to_channel(pin) == PWM_CHAN_B && "tachometers must run on a B channel");
        assert(0 < pulses_per_revolution);
    }

    [[nodiscard]] double revolutions_per_second() const {
        return revolutions_per_second_;
    }

    [[nodiscard]] char const* name() const override {
        return "Tachometer";
    }

protected:
    void read() override {
        pwm_set_counter(slice_num, 0);
        auto begin = std::chrono::steady_clock::now();
        pwm_set_enabled(slice_num, true);

        task_delay(TACHOMETER_READ_PERIOD);

        pwm_set_enabled(slice_num, false);
        auto end = std::chrono::steady_clock::now();

        auto duration_sec =
                std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(end - begin);
        auto count = pwm_get_counter(slice_num);
        revolutions_per_second_ = count / duration_sec.count() / pulses_per_revolution;

        // printf("tachometer_measure dur=%f s cnt=%d rev-per-sec=%f rpm=%f\n",
        //    duration_sec.count(), int(count), revolutions_per_second_, revolutions_per_second_ * 60);
    }

private:
    uint slice_num;
    uint pulses_per_revolution;
    double revolutions_per_second_ = 0;
};

}  // namespace nevermore::sensors
