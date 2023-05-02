#pragma once

#include "hardware/pwm.h"
#include <cstdint>

constexpr uint8_t pwm_gpio_to_slice_num_(uint8_t gpio) {
    return (gpio >> 1u) & 0b111;
}

constexpr pwm_chan pwm_gpio_to_channel_(uint8_t gpio) {
    return gpio & 1u ? PWM_CHAN_B : PWM_CHAN_A;
}

// Return the level while accounting for the slice's current top.
uint16_t pwm_gpio_duty(uint8_t gpio, uint16_t duty);

// Sets the level while accounting for the slice's current top.
void pwm_set_gpio_duty(uint8_t gpio, uint16_t duty);

void pwm_config_set_freq_hz(pwm_config& c, uint32_t freq_hz);
