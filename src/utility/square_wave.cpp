
#include "square_wave.hpp"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <cassert>
#include <cstdio>

namespace nevermore {

void square_wave_pwm_init(GPIO pin, uint32_t hz, float duty_f) {
    assert(0 < hz);
    assert(0 <= duty_f && duty_f <= 1);

    auto const slice_num = pwm_gpio_to_slice_num(pin);
    auto const& slice_hw = pwm_hw->slice[slice_num];  // NOLINT

    auto cfg = pwm_get_default_config();
    pwm_config_set_freq_hz(cfg, hz);
    pwm_init(slice_num, &cfg, true);

    auto level = pwm_gpio_duty(pin, uint16_t(UINT16_MAX * duty_f));
    pwm_set_gpio_level(pin, level);

    gpio_set_function(pin, GPIO_FUNC_PWM);

    // same spurious warning from GCC as in `main()`.
    static_assert(sizeof(hz) == sizeof(unsigned));
    printf("DEBUG - SQUARE WAVE pin=%d w/ %u hz @ %.2f%% duty\n"
           "\tdiv=%d.%d top=%d level=%d\n",
            int(pin), unsigned(hz), duty_f * 100, int(slice_hw.div / 16), int(slice_hw.div & 0b1111),
            int(slice_hw.top), int(level));
}

GPIO square_wave_pwm_first_available_pin() {
    uint32_t slices = 0;
    for (uint i = 0; i < PIN_MAX; ++i)
        if (gpio_get_function(i) == GPIO_FUNC_PWM) {
            slices |= 1u << pwm_gpio_to_slice_num(i);
        }

    // skip pins {0, 1}, they're reserved for UART
    for (uint i = 2; i < PIN_MAX; i += 1) {
        if (gpio_get_function(i) != GPIO_FUNC_NULL) continue;
        if (slices & (1u << pwm_gpio_to_slice_num(i))) continue;

        return i;
    }

    return {};
}

}  // namespace nevermore
