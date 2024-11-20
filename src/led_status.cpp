#include "led_status.hpp"
#include "config.hpp"
#include "pico.h"  // IWYU pragma: keep for transitive includes (e.g. board)
#include "settings.hpp"
#include "utility/periodic_waves.hpp"

#if defined(CYW43_WL_GPIO_LED_PIN)
#include "pico/cyw43_arch.h"
#endif

using namespace nevermore;
using namespace std::chrono_literals;

namespace nevermore::led_status {

namespace {

constexpr auto PERIOD_VOC_CALIBRATE_ON = 1s;
constexpr auto PERIOD_VOC_CALIBRATE_OFF = 0.5s;

bool g_voc_calibration_saved = false;

void update_voc_calibration() {
    auto pin = Pins::active().led_status_voc_calibration;
    if (!pin) return;

    auto period =
            settings::g_active.voc_calibration_enabled ? PERIOD_VOC_CALIBRATE_ON : PERIOD_VOC_CALIBRATE_OFF;
    bool led_on = g_voc_calibration_saved || 0.5 < periodic_waves::square(period);
    gpio_put(pin, led_on);
}

[[maybe_unused]] void update_board() {
    [[maybe_unused]] bool led_on = 0.5 < periodic_waves::square(SENSOR_UPDATE_PERIOD * 2);

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
}

}  // namespace

void voc_calibration_mark_saved() {
    // ARMv6 byte transactions are single-copy atomic, concurrent read/write is okay
    g_voc_calibration_saved = true;
}

void update() {
    update_board();
    update_voc_calibration();
}

}  // namespace nevermore::led_status
