#include "btstack_run_loop.h"
#include "config.hpp"
#include "gatt.hpp"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio.h"
#include "sensors.hpp"
#include "utility/square_wave.hpp"
#include <cstdint>
#include <cstdio>

namespace {

// so far PIN config can be statically checked, so no risk of runtime error
void pins_setup() {
    // Leave pins {0, 1} set to UART TX/RX.
    // Clear everything else.
    for (GPIO_Pin pin = 2; pin < PIN_MAX; ++pin) {
        gpio_set_function(pin, GPIO_FUNC_NULL);
        gpio_set_pulls(pin, false, false);
    }

    for (auto pin : PINS_I2C) {
        gpio_set_function(pin, GPIO_FUNC_I2C);
        gpio_pull_up(pin);
    }

    gpio_set_function(PIN_FAN_PWM, GPIO_FUNC_PWM);
    gpio_set_function(PIN_FAN_TACHOMETER, GPIO_FUNC_PWM);
    gpio_pull_up(PIN_FAN_TACHOMETER);

    // we're setting up the WS2812 controller on PIO0
    gpio_set_function(PIN_NEOPIXEL_DATA_IN, GPIO_FUNC_PIO0);

#ifndef NDEBUG
    if (PIN_DBG_SQUARE_WAVE) {
        // setup a debug
        square_wave_pwm_init(*PIN_DBG_SQUARE_WAVE, 30);
    } else
        printf("!! No available PWM slice for square wave generator.\n");
#endif
}

}  // namespace

int main() {
    stdio_init_all();
    adc_init();

    if (auto err = cyw43_arch_init()) {
        printf("cyw43_arch_init failed = 0x%08x\n", err);
        return -1;
    }

    // GCC 12.2.1 bug: -Werror=format reports that `I2C_BAUD_RATE` is a `long unsigned int`.
    // This is technically true on this platform, see static-assert below, but it is benign since
    // `unsigned == long unsigned int` is also true on this platform. Pedant.
    // Fix by casting to unsigned instead of changing format specifier, this keeps clangd happy
    // since `clangd`, incorrectly, thinks that `unsigned != long unsigned int` on this platform.
    static_assert(sizeof(I2C_BAUD_RATE) == sizeof(unsigned));
    printf("I2C bus 0 running at %u baud/s (requested %u baud/s)\n", i2c_init(i2c0, I2C_BAUD_RATE),
            unsigned(I2C_BAUD_RATE));
    printf("I2C bus 1 running at %u baud/s (requested %u baud/s)\n", i2c_init(i2c1, I2C_BAUD_RATE),
            unsigned(I2C_BAUD_RATE));

    auto& ctx_async = *cyw43_arch_async_context();

    pins_setup();
    if (!sensors_init(ctx_async, EnvironmentService::g_service_data)) return -1;
    if (!gatt_init(ctx_async)) return -1;

    btstack_run_loop_execute();  // !! NO-RETURN
    return 0;
}
