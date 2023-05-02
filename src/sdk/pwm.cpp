#include "pwm.hpp"
#include "hardware/pwm.h"

namespace {

constexpr auto TOP_MAX = UINT16_MAX - 1;  // - 1 to allow 100% duty w/ UINT16_MAX level

// calculate clock div w/ proper rounding
uint32_t clock_div(uint32_t source_hz, uint32_t target_hz) {
    return (source_hz + target_hz / 2) / target_hz;
}

}  // namespace

// stolen/derived from: https://github.com/micropython/micropython/blob/master/ports/rp2/machine_pwm.c
uint16_t pwm_gpio_duty(uint8_t gpio, uint16_t duty) {
    uint const slice_num = pwm_gpio_to_slice_num(gpio);
    uint const chan = pwm_gpio_to_channel(gpio);
    check_slice_num_param(slice_num);

    // Use rounding here to set it as accurately as possible
    auto top = pwm_hw->slice[slice_num].top;  // NOLINT
    return clock_div(duty * (top + 1), TOP_MAX + 1);
}

void pwm_set_gpio_duty(uint8_t gpio, uint16_t duty) {
    pwm_set_gpio_level(gpio, pwm_gpio_duty(gpio, duty));
}

// stolen/derived from: https://github.com/micropython/micropython/blob/master/ports/rp2/machine_pwm.c
void pwm_config_set_freq_hz(pwm_config& c, uint32_t const freq_hz) {
    const auto source_hz = clock_get_hz(clk_sys);

    // Returns: floor((16*F + offset) / div16)
    auto const slice_hz = [&](uint32_t offset, uint32_t div16) -> uint32_t {
        // Avoid overflow in the numerator that would occur if
        //   16*F + offset > 2**32
        //   F + offset/16 > 2**28 = 268435456 (approximately, due to flooring)
        return (16 * uint64_t(source_hz) + offset) / div16;
    };

    auto const slice_hz_ceil = [&](uint32_t div16) { return slice_hz(div16 - 1, div16); };
    auto const slice_hz_round = [&](uint32_t div16) { return slice_hz(div16 / 2, div16); };

    uint32_t div16 = 16;  // 8.4 fixed point format
    uint32_t top = clock_div(source_hz, freq_hz);

    if (TOP_MAX <= top) {
        // Choose the smallest possible DIV for maximum duty cycle resolution.
        // Constraint: 16*F/(div16*freq) < TOP_MAX
        // So:
        div16 = slice_hz_ceil(TOP_MAX * freq_hz);

        // Set TOP as accurately as possible using rounding.
        top = slice_hz_round(div16 * freq_hz);
    }

    pwm_config_set_clkdiv_int_frac(&c, div16 / 16, div16 & 0b1111);
    pwm_config_set_wrap(&c, top - 1);  // - 1 to enable 100% duty using `top`
}
