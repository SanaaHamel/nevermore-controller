#pragma once

// Generate a square wave of a given freq on some arbitrary pin.
// Used for debugging purposes, particularly when testing the tachometer implementation.

#include "config.hpp"
#include "sdk/pwm.hpp"
#include <cstdint>
#include <optional>

namespace nevermore {

// sets up a PWM to emit a square wave with the given frequency
void square_wave_pwm_init(GPIO_Pin, uint32_t hz, float duty = .5f);

constexpr std::optional<GPIO_Pin> square_wave_pwm_first_available_pin() {
    // skip pins {0, 1}, they're reserved for UART
    for (GPIO_Pin pin = 2; pin < PIN_MAX; ++pin) {
        // O(n^2) inefficient, but who cares, it's compile time.
        if (pwm_gpio_to_slice_num_(pin) == pwm_gpio_to_slice_num_(PIN_FAN_PWM)) continue;
        if (pwm_gpio_to_slice_num_(pin) == pwm_gpio_to_slice_num_(PIN_FAN_TACHOMETER)) continue;
        if (pin_exists([&](GPIO_Pin p) { return p == pin; })) continue;

        return pin;
    }

    return {};
}

constexpr auto PIN_DBG_SQUARE_WAVE = square_wave_pwm_first_available_pin();

}  // namespace nevermore
