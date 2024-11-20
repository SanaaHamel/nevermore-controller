#include "FreeRTOS.h"  // IWYU pragma: keep
#include "btstack_run_loop.h"
#include "config.hpp"
#include "display.hpp"
#include "gatt.hpp"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico.h"  // IWYU pragma: keep for transitive includes (e.g. board)
#include "pico/stdio.h"
#include "pico/time.h"
#include "picowota/reboot.h"
#include "sdk/i2c.hpp"
#include "sdk/task.hpp"
#include "sensors.hpp"
#include "settings.hpp"
#include "task.h"  // IWYU pragma: keep
#include "utility/i2c.hpp"
#include "utility/task.hpp"
#include "utility/timer.hpp"
#include "ws2812.hpp"
#include <cassert>
#include <cstdio>

#if NEVERMORE_PICO_W_BT || defined(CYW43_WL_GPIO_LED_PIN) || CYW43_USES_VSYS_PIN
#define CYW43_IN_USE 1
#else
#define CYW43_IN_USE 0
#endif

#if CYW43_IN_USE
#include "pico/cyw43_arch.h"
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

// declared/defined by `pico_stdio_usb`
bool stdio_usb_connected();
}

namespace {

constexpr auto WATCHDOG_TIMEOUT = 3000ms;

// Leave pins {0, 1} set to UART TX/RX.
// Clear everything else.
void pins_clear_user_defined() {
    for (GPIO pin = 2; pin.gpio < PIN_MAX; pin.gpio += 1) {
        if (find(begin(PINS_RESERVED_BOARD), end(PINS_RESERVED_BOARD), pin) != end(PINS_RESERVED_BOARD))
            continue;

        gpio_set_function(pin, GPIO_FUNC_NULL);
        gpio_set_dir(pin, false);
        gpio_pull_down(pin);
    }
}

// NB: changes pin function assignments
void pins_i2c_reset(Pins const& pins = settings::g_active.pins) {
    for (auto const& bus : pins.i2c) {
        // `i2c_bitbang_reset` is responsible for changing the pin functions
        if (bus && !i2c_bitbang_reset(bus.data, bus.clock)) {
            printf("WARN - I2C (CLK %d, DAT %d) - failed to reset bus\n", (int)bus.clock, (int)bus.data);
        }
    }
}

optional<bool> vbus_powered() {
#if defined(CYW43_WL_GPIO_VBUS_PIN)
    return cyw43_arch_gpio_get(CYW43_WL_GPIO_VBUS_PIN);
#elif defined(PICO_VBUS_PIN)
    gpio_set_function(PICO_VBUS_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(pin, false);
    return gpio_get(PICO_VBUS_PIN);
#else
    return {};  // don't know
#endif
}

void setup_watchdog() {
    if (watchdog_enable_caused_reboot()) {
        printf("WARN - last reboot triggered by watchdog timeout\n");
    }

    picowota_watchdog_enable_bootloader(WATCHDOG_TIMEOUT / 1ms, true);

    // Kinda wasteful to allocate a whole task for this.
    // Can't use a timer, they might starve/miss their deadline.
    mk_task("watchdog-update", Priority::WatchdogUpdate, 128)([]() {
        for (;;) {
            watchdog_update();
            task_delay(WATCHDOG_TIMEOUT / 4);
        }
    }).release();
}

}  // namespace

void startup() {
    stdio_init_all();
    adc_init();

#if CYW43_IN_USE
    // need the CYW43 up to access the LED, even if we don't have BT enabled
    if (auto err = cyw43_arch_init()) {
        panic("ERR - cyw43_arch_init failed = 0x%08x\n", err);
    }
#endif

    // tri-logic: some boards can't tell if VBUS is powered
    if (vbus_powered() != false) {
        auto const deadline = make_timeout_time_ms(STDIO_USB_CONNECT_TIMEOUT / 1ms);
        while (!stdio_usb_connected() && !time_reached(deadline))
            sleep(100ms);
    }

    setup_watchdog();

    settings::init();

    pins_clear_user_defined();
    pins_i2c_reset();           // bit-bang out a reset for the I2C buses
    pins_clear_user_defined();  // clear pins again, `pins_i2c_reset` leaves things dirty
    // setup everything (except UART, which should be set to default 0/1)
    [[maybe_unused]] auto pin_setup_ok = Pins::setup(settings::g_active.pins);
    assert(pin_setup_ok);

    ws2812::init();
    if (!gatt::init()) return;
    // display must be init before sensors b/c some sensors are display input devices
    if (!display::init_with_ui()) return;
    if (!sensors::init()) return;

    // HACK: Why the hell is this a BT timer instead of a FreeRTOS timer?
    // Because apparently you can't safely call `cyw43_arch_gpio_put` from a
    // FreeRTOS timer without risking a deadlock. Why? IDK, that API calls
    // `CYW43_THREAD_{ENTER/EXIT}` but apparently that's not enough to keep the
    // peace.
    // Calling it from a BT timer seems to solve the problem. IDK why, but I'm not
    // paid to find out and I've already wasted an afternoon on this.
    static btstack_timer_source_t led_timer{.process = [](btstack_timer_source_t* timer) {
        static bool led_on = false;
        led_on = !led_on;

#if defined(PICO_DEFAULT_LED_PIN)
        gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(PICO_DEFAULT_WS2812_PIN)
        // FUTURE WORK: implement WS2812 LED
#elif defined(CYW43_WL_GPIO_LED_PIN)
        // HACK:  `cyw43_arch_gpio_put` w/o having the HCI powered on
        //        kills the timer task when it enters `cyw43_ensure_up`.
        //        Root cause unknown. This hack should be benign since
        //        Pico W is typically built w/ BT enabled.
        if constexpr (NEVERMORE_PICO_W_BT) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
        }
#endif

        btstack_run_loop_set_timer(timer, SENSOR_UPDATE_PERIOD / 1ms);
        btstack_run_loop_add_timer(timer);
    }};
    led_timer.process(&led_timer);

    if constexpr (NEVERMORE_PICO_W_BT) {
        mk_task("bluetooth", Priority::Communication, 1024)(btstack_run_loop_execute).release();
    }
}

int main() {
    // has to be done on core 0 for `stdio_init_all`.
    mk_task("startup", Priority::Startup, 1024, 1 << 0)([]() {
        startup();
        vTaskDelete(nullptr);  // we're done, delete ourselves
    }).release();

    vTaskStartScheduler();  // !! NO-RETURN
    return 0;
}
