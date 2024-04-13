#include "display.hpp"
#include "config/pins.hpp"
#include "display/gc9a01.hpp"
#include "display/lv_driver_interface.hpp"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "lvgl.h"  // IWYU pragma: keep
#include "sdk/pwm.hpp"
#include "sdk/task.hpp"
#include "settings.hpp"
#include "ui.hpp"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>

using namespace std;
using namespace nevermore;

namespace nevermore::display {

namespace {

constexpr auto DISPLAY_BACKLIGHT_FREQ = 1'000;

lv_color_t g_draw_scratch_buffers[2][RESOLUTION.width * RESOLUTION.height / 2];
lv_disp_draw_buf_t g_draw_buffer;
lv_disp_drv_t g_driver;
lv_disp_t* g_display;

}  // namespace

void brightness(float power) {
    settings::g_active.display_brightness = clamp(power, 0.f, 1.f);
    if (auto pin = Pins::active().display_brightness_pwm) {
        pwm_set_gpio_duty(pin, UINT16_MAX * settings::g_active.display_brightness);
    }
}

float brightness() {
    return settings::g_active.display_brightness;
}

// Initialises the UI. Everything else should be hands off after that.
// FUTURE WORK: Move to second core if we ever run into perf issues.
//              Have a care regarding potential issues w/ interrupts/timers w/ core 0.
bool init_with_ui() {
    if (auto pin = Pins::active().display_brightness_pwm) {
        auto cfg = pwm_get_default_config();
        pwm_config_set_freq_hz(cfg, DISPLAY_BACKLIGHT_FREQ);
        pwm_init(pwm_gpio_to_slice_num_(pin), &cfg, true);
    }
    brightness(settings::g_active.display_brightness);

    lv_init();
    lv_disp_draw_buf_init(&g_draw_buffer, g_draw_scratch_buffers[0], g_draw_scratch_buffers[1],
            size(g_draw_scratch_buffers[0]));

    switch (settings::g_active.display_hw) {
    case settings::DisplayHW::GC9A01_240_240: {
        auto driver = gc9a01();
        if (!driver) return false;

        g_driver = *driver;
        g_driver.hor_res = RESOLUTION.width;
        g_driver.ver_res = RESOLUTION.height;
        g_driver.draw_buf = &g_draw_buffer;
    } break;

    default: {
        printf("ERR - init_with_ui - unsupported display HW %u\n", (unsigned)settings::g_active.display_hw);
        return false;
    } break;
    }

    if (g_display = lv_disp_drv_register(&g_driver); !g_display) {
        printf("ERR - init_with_ui - lv_disp_drv_register returned null\n");
        return false;
    }

    return ui::init();
}

spi_inst_t* active_spi() {
    for (auto&& bus : Pins::active().spi) {
        if (!bus || bus.kind != Pins::BusSPI::Kind::display) continue;

        // FUTURE WORK: won't work correctly when PIO buses happen b/c this
        // might be a PIO even if it can be a HW
        if (auto hw = bus.hardware_bus_num()) {
            assert(hw <= 1);
            return hw == 0 ? spi0 : spi1;
        }

        assert(false && "PIO `LV_DRV_DISP_SPI_WR_ARRAY` not impl");
    }

    return nullptr;
}

}  // namespace nevermore::display

////////////////////////////////////
// LVGL DRIVER INTERFACE FUNCTIONS
////////////////////////////////////

void LV_DRV_DISP_CMD_DATA(bool value) {
    if (auto pin = Pins::active().display_command) {
        gpio_put(pin, value);
    }
}

void LV_DRV_DISP_RST(bool value) {
    if (auto pin = Pins::active().display_reset) {
        gpio_put(pin, value);
    }
}

void LV_DRV_DISP_SPI_CS(bool) {
    // NOP - got a single device on the SPI bus for now, can always assume active
}

// Slow path fallback. It is expected that the driver use DMA to feed the SPI engine.
void LV_DRV_DISP_SPI_WR_ARRAY(uint8_t const* src, unsigned len) {
    assert(src);
    if (len <= 0) return;

    if (auto* spi = display::active_spi()) {
        [[maybe_unused]] auto n = spi_write_blocking(spi, src, len);
        assert(n == int(len) && "failed to write octet");
    }
}

// The LVGL driver API seems to use `char` instead of `uint8_t`. Whatever. Keep warning free.
void LV_DRV_DISP_SPI_WR_ARRAY(char const* src, unsigned len) {
    LV_DRV_DISP_SPI_WR_ARRAY(reinterpret_cast<uint8_t const*>(src), len);
}

void LV_DRV_DISP_SPI_WR_BYTE(uint8_t x) {
    LV_DRV_DISP_SPI_WR_ARRAY(&x, 1);
}

void LV_DRV_DELAY_MS(uint32_t ms) {
    task_delay(chrono::milliseconds(ms));
}
