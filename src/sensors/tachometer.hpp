#pragma once

#include "async_sensor.hpp"
#include "sdk/pwm.hpp"
#include <chrono>
#include <cstdint>

namespace nevermore::sensors {

using namespace std::literals::chrono_literals;

struct Tachometer final : SensorDelayedResponse {
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

    [[nodiscard]] std::chrono::milliseconds read_delay() const override {
        return TACHOMETER_READ_PERIOD;
    }

    [[nodiscard]] bool issue() override {
        pwm_set_counter(slice_num, 0);
        begin = std::chrono::steady_clock::now();
        pwm_set_enabled(slice_num, true);
        return true;
    }

    void read() override {
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
    std::chrono::steady_clock::time_point begin{};
};

}  // namespace nevermore::sensors
