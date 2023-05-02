
#include "square_wave.hpp"
#include "hardware/gpio.h"
#include "sdk/pwm.hpp"
#include <cassert>
#include <climits>
#include <cstdio>

void square_wave_pwm_init(GPIO_Pin pin, uint32_t hz, float duty_f) {
    assert(0 < hz);
    assert(0 <= duty_f && duty_f <= 1);

    auto const slice_num = pwm_gpio_to_slice_num_(pin);
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
