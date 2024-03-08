#pragma once

// Generate a square wave of a given freq on some arbitrary pin.
// Used for debugging purposes, particularly when testing the tachometer implementation.

#include "config/pins.hpp"
#include "sdk/pwm.hpp"
#include <cstdint>

namespace nevermore {

// sets up a PWM to emit a square wave with the given frequency
void square_wave_pwm_init(GPIO, uint32_t hz, float duty = .5f);

constexpr GPIO square_wave_pwm_first_available_pin(Pins const& config) {
    uint32_t slices = 0;
    config.foreach_pwm_function([&](std::span<GPIO const> xs, bool allow_sharing) {
        for (auto x : xs)
            if (x) slices |= 1u << pwm_gpio_to_slice_num_(x);
    });

    // skip pins {0, 1}, they're reserved for UART
    for (GPIO pin = 2; pin.gpio < PIN_MAX; pin.gpio += 1) {
        // O(n^2) inefficient, but who cares, it's compile time.
        if (slices & (1u << pwm_gpio_to_slice_num_(pin))) continue;
        if (config.pins_exist([&](GPIO p) { return p == pin; })) continue;

        return pin;
    }

    return {};
}

// constexpr auto PIN_DBG_SQUARE_WAVE = square_wave_pwm_first_available_pin();

}  // namespace nevermore
