#include "FreeRTOS.h"  // IWYU pragma: keep
#include "btstack_run_loop.h"
#include "config.hpp"
#include "display.hpp"
#include "gatt.hpp"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/platform_defs.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio.h"
#include "sdk/i2c.hpp"
#include "sdk/spi.hpp"
#include "sensors.hpp"
#include "task.h"  // IWYU pragma: keep
#include "utility/i2c.hpp"
#include "utility/task.hpp"
#include "utility/timer.hpp"
#include "ws2812.hpp"
#include <cstdint>
#include <cstdio>
#include <utility>

#ifndef NDEBUG
#include "utility/square_wave.hpp"
#endif

using namespace std;
using namespace nevermore;

extern "C" {
void vApplicationTickHook() {}

void vApplicationStackOverflowHook(TaskHandle_t Task, char* pcTaskName) {
    panic("PANIC - stack overflow in task %s\n", pcTaskName);
}

void vApplicationMallocFailedHook() {
    panic("PANIC - heap alloc failed\n");
}
}

namespace {

// Leave pins {0, 1} set to UART TX/RX.
// Clear everything else.
void pins_clear_except_uart() {
    for (GPIO_Pin pin = 2; pin < PIN_MAX; ++pin) {
        gpio_set_function(pin, GPIO_FUNC_NULL);
        gpio_set_dir(pin, false);
        gpio_pull_down(pin);
    }
}

// so far PIN config can be statically checked, so no risk of runtime error
void pins_setup() {
    for (auto pin : PINS_I2C) {
        gpio_set_function(pin, GPIO_FUNC_I2C);
        gpio_pull_up(pin);
    }

    gpio_set_function(PIN_FAN_PWM, GPIO_FUNC_PWM);
    gpio_set_function(PIN_FAN_TACHOMETER, GPIO_FUNC_PWM);
    gpio_pull_up(PIN_FAN_TACHOMETER);

    // we're setting up the WS2812 controller on PIO0
    gpio_set_function(PIN_NEOPIXEL_DATA_IN, GPIO_FUNC_PIO0);

    for (auto pin : PINS_DISPLAY_SPI)
        gpio_set_function(pin, GPIO_FUNC_SPI);

    gpio_set_function(PIN_DISPLAY_COMMAND, GPIO_FUNC_SIO);
    gpio_set_function(PIN_DISPLAY_RESET, GPIO_FUNC_SIO);
    gpio_set_function(PIN_DISPLAY_BRIGHTNESS, GPIO_FUNC_PWM);
    gpio_set_function(PIN_TOUCH_INTERRUPT, GPIO_FUNC_SIO);
    gpio_set_function(PIN_TOUCH_RESET, GPIO_FUNC_SIO);

    gpio_set_dir(PIN_DISPLAY_COMMAND, true);
    gpio_set_dir(PIN_DISPLAY_RESET, true);
    gpio_set_dir(PIN_TOUCH_INTERRUPT, false);
    gpio_set_dir(PIN_TOUCH_RESET, true);

#ifndef NDEBUG
    if (PIN_DBG_SQUARE_WAVE) {
        // setup a debug
        square_wave_pwm_init(*PIN_DBG_SQUARE_WAVE, 30);
    } else
        printf("!! No available PWM slice for square wave generator.\n");
#endif
}

void pins_i2c_reset() {
    auto get = [](uint8_t bus, I2C_Pin kind) {
        for (auto pin : PINS_I2C)
            if (i2c_gpio_bus_num(pin) == bus && i2c_gpio_kind(pin) == kind) return pin;

        unreachable();
    };

    static_assert(size(PINS_I2C) == 4, "too many pins - not impl");
    for (uint8_t i = 0; i < NUM_I2CS; ++i) {
        if (!i2c_bitbang_reset(get(i, I2C_Pin::SDA), get(i, I2C_Pin::SCL)))
            printf("WARN - I2C%d - failed to reset bus\n", i);
    }
}

}  // namespace

int main() {
    for (auto& lock : g_i2c_locks)
        lock = xSemaphoreCreateMutex();

    stdio_init_all();
    adc_init();

    pins_clear_except_uart();
    pins_i2c_reset();          // bit-bang out a reset for the I2C buses
    pins_clear_except_uart();  // clear pins again, `pins_i2c_reset` leaves things dirty
    pins_setup();              // setup everything (except UART, which should be set to default 0/1)

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

    // I2C devices aren't required to understand/respond to general-call (addr 0) reset (0x06).
    // But for those that do, hey, might as well tell them to reset in case we just rebooted.
    // TODO:  Can't do this with the Pico SDK because it asserts/param checks that the address isn't
    //        reserved, which is a problem since the general-call address *is* a reserved address.
#if 0
    i2c_write_blocking(*i2c0, 0x00, 0x06_u8);
    i2c_write_blocking(*i2c1, 0x00, 0x06_u8);
#endif

    auto* spi = spi_gpio_bus(PINS_DISPLAY_SPI[0]);
    printf("SPI bus %d running at %u baud/s (requested %u baud/s)\n", spi_gpio_bus_num(PINS_DISPLAY_SPI[0]),
            spi_init(spi, SPI_BAUD_RATE_DISPLAY), unsigned(SPI_BAUD_RATE_DISPLAY));

    mk_task("startup", Priority::Startup, 1024)([]() {
        if (auto err = cyw43_arch_init()) {
            panic("ERR - cyw43_arch_init failed = 0x%08x\n", err);
        }

        ws2812::init();
        // display must be init before sensors b/c some sensors are display input devices
        if (!display::init_with_ui()) return;
        if (!sensors::init()) return;
        if (!gatt::init()) return;

        mk_timer("led-blink", SENSOR_UPDATE_PERIOD)([](TimerHandle_t) {
            static bool led_on = false;
            led_on = !led_on;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
        });

        mk_task("bluetooth", Priority::Communication, 1024)(btstack_run_loop_execute).release();

        vTaskDelete(nullptr);  // we're done, delete ourselves
    }).release();

    vTaskStartScheduler();  // !! NO-RETURN
    return 0;
}
