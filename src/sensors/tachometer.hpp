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
struct Tachometer final : SensorPeriodic {
    // need at least 100ms for a reasonable read and no point sampling longer than 1s
    constexpr static auto TACHOMETER_READ_PERIOD =
            std::clamp<std::chrono::milliseconds>(SENSOR_UPDATE_PERIOD, 100ms, 1s);

    // WORKAROUND:  There's EMI from the PWM wire (runs adjacent to tacho).
    // Proper fix:  Add a 2.2k pull-up & 0.1uF capacitor-to-0v to tachometer.
    //    Our fix:  Denoise the signal in software. Just wait for consensus
    //              over multiple samples before considering the state changed.
    //              Downside is that we need to do this in software instead of
    //              using the PWM hardware.
    // Credit to @Mario1up on Discord for confirming the EMI issue and proposing
    // and testing the hardware fix.
    using ConsensusSet = uint8_t;  // bigger type -> longer consensus period

    // hz_max_pulse = hz_sample / 2
    // sample at 1 kHz, that'll support up to 15'000 RPM w/ 2 pulses per rev
    constexpr static auto PIN_SAMPLING_PERIOD = 1.0ms / (sizeof(ConsensusSet) * CHAR_BIT);

    Tachometer() = default;

    void setup(Pins::GPIOs const& pins, uint32_t pulses_per_revolution = 1) {
        assert(0 < pulses_per_revolution);

        for (auto&& pin : pins)
            assert(!pin || gpio_get_function(pin) == GPIO_FUNC_SIO);

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

        pulse_start();

        for (auto now = begin; (now - begin) < TACHOMETER_READ_PERIOD;
                now = std::chrono::steady_clock::now()) {
            pulses += pulse_poll();
            task_delay(PIN_SAMPLING_PERIOD);
        }

        return pulses;
    }

    void pulse_start() {
        for (auto&& pin : pins) {
            if (!pin) continue;

            bool value = gpio_get(pin);
            state.set(&pin - pins, value);
            // assume noisy sample, init w/ conflicting to force wait for consensus
            denoise.at(&pin - pins) = 0b10;
        }
    }

    uint32_t pulse_poll() {
        uint32_t pulses = 0;

        for (auto&& pin : pins) {
            if (!pin) continue;

            auto curr = gpio_get(pin);
            auto prev = state.test(&pin - pins);
            auto accum = ((denoise.at(&pin - pins) << 1) | curr) & DENOISE_ALL;
            auto consensus = curr ? DENOISE_ALL : 0;
            denoise.at(&pin - pins) = accum;

            if (accum == consensus && prev != curr) {
                pulses += curr;  // count rising edges
                state.set(&pin - pins, curr);
            }
        }

        return pulses;
    }

    Pins::GPIOs pins;
    std::bitset<Pins::ALTERNATIVES_MAX> state;
    std::array<ConsensusSet, Pins::ALTERNATIVES_MAX> denoise{};
    uint32_t pulses_per_revolution = 1;
    float revolutions_per_second_ = 0;

    static constexpr auto DENOISE_ALL = std::numeric_limits<ConsensusSet>::max();
};

}  // namespace nevermore::sensors
