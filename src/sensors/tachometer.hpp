#pragma once

#include "async_sensor.hpp"
#include "config/pins.hpp"
#include "hardware/gpio.h"
#include "sdk/pwm.hpp"
#include <algorithm>
#include <bitset>
#include <chrono>
#include <cstdint>

namespace nevermore::sensors {

using namespace std::literals::chrono_literals;

// 'Low' speed tachometer, intended for < 1000 pulses/sec.
// DELTA BFB0712H spec sheet says an RPM of 2900 -> ~49 rev/s
// PWM counter wraps at 2^16-1 -> if a fan is spinning that fast you've a problem.
struct Tachometer final : SensorPeriodic {
    // need at least 100ms for a reasonable read and no point sampling longer than 1s
    constexpr static auto TACHOMETER_READ_PERIOD =
            std::clamp<std::chrono::milliseconds>(SENSOR_UPDATE_PERIOD, 100ms, 1s);

    // hz_max_pulse = hz_sample / 2
    // sample at 10 kHz, that'll support up to 150'000 RPM w/ 2 pulses per rev
    constexpr static auto PIN_SAMPLING_PERIOD = 0.1ms;

    Tachometer() = default;

    void setup(Pins::GPIOs const& pins, uint32_t pulses_per_revolution = 1) {
        assert(0 < pulses_per_revolution);

        for (auto&& pin : pins) {
            if (!pin) continue;

            switch (gpio_get_function(pin)) {
            default: assert(false); break;
            case GPIO_FUNC_SIO: break;
            case GPIO_FUNC_PWM: {
                auto cfg = pwm_get_default_config();
                pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_FALLING);
                pwm_init(pwm_gpio_to_slice_num_(pin), &cfg, false);
            } break;
            }
        }

        std::copy(std::begin(pins), std::end(pins), this->pins);
        this->pulses_per_revolution = pulses_per_revolution;
    }

    [[nodiscard]] auto revolutions_per_second() const {
        return revolutions_per_second_;
    }

    [[nodiscard]] char const* name() const override {
        return "Tachometer";
    }

protected:
    void read() override {
        // Since we're targetting relatively low pulse hz don't bother about
        // keeping code in RAM to avoid flash penalty or function overhead;
        // we're running at 125 MHz & reading <= 1 kHz pulse signals,
        // we've plenty of room for sloppiness.
        auto begin = std::chrono::steady_clock::now();
        uint32_t pulses = pulse_count(begin);
        auto end = std::chrono::steady_clock::now();

        auto duration_sec =
                std::chrono::duration_cast<std::chrono::duration<float, std::ratio<1>>>(end - begin);
        // NOLINTNEXTLINE(bugprone-narrowing-conversions, cppcoreguidelines-narrowing-conversions)
        revolutions_per_second_ = pulses / duration_sec.count() / pulses_per_revolution;

        // printf("tachometer_measure dur=%f s cnt=%u rev-per-sec=%f rpm=%f\n", duration_sec.count(),
        //         unsigned(pulses), revolutions_per_second_, revolutions_per_second_ * 60);
    }

private:
    // we're nowhere near high precision stuff
    uint32_t pulse_count(std::chrono::steady_clock::time_point const begin) {
        uint32_t pulses = 0;
        if (pulse_start()) {  // polling required, we've non-PWM tacho pins
            for (auto now = begin; (now - begin) < TACHOMETER_READ_PERIOD;
                    now = std::chrono::steady_clock::now()) {
                pulses += pulse_poll();
                task_delay(PIN_SAMPLING_PERIOD);
            }
        } else  // everything is handled by PWM slices, just nap for a bit
            task_delay(TACHOMETER_READ_PERIOD);

        pulses += pulse_end();  // add pulses from PWM counters
        return pulses;
    }

    bool pulse_start() {
        bool do_polling = false;

        for (auto&& pin : pins) {
            if (!pin) continue;

            switch (gpio_get_function(pin)) {
            default: break;
            case GPIO_FUNC_SIO: {
                do_polling = true;
                state.set(&pin - pins, gpio_get(pin));
            } break;
            case GPIO_FUNC_PWM: {
                auto slice = pwm_gpio_to_slice_num_(pin);
                pwm_set_counter(slice, 0);
                pwm_set_enabled(slice, true);
            } break;
            }
        }

        return do_polling;
    }

    uint32_t pulse_poll() {
        uint32_t pulses = 0;

        for (auto&& pin : pins) {
            if (pin && gpio_get_function(pin) == GPIO_FUNC_SIO) {
                auto curr = gpio_get(pin);
                auto prev = state.test(&pin - pins);
                pulses += (prev != curr) && curr;  // count rising edges
                state.set(&pin - pins, curr);
            }
        }

        return pulses;
    }

    uint32_t pulse_end() {
        uint32_t pulses = 0;

        for (auto&& pin : pins) {
            if (pin && gpio_get_function(pin) == GPIO_FUNC_PWM) {
                auto slice = pwm_gpio_to_slice_num_(pin);
                pwm_set_enabled(slice, false);
                pulses += pwm_get_counter(slice);
            }
        }

        return pulses;
    }

    Pins::GPIOs pins;
    std::bitset<Pins::ALTERNATIVES_MAX> state;
    uint32_t pulses_per_revolution = 1;
    float revolutions_per_second_ = 0;
};

}  // namespace nevermore::sensors
