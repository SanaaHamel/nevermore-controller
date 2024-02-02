#include "display.hpp"
#include "config.hpp"
#include "config/lib/lv_drv_conf.h"  // need the `extern "C"` decls for LVGL driver interface
#include "display/gc9a01.hpp"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "lvgl.h"  // IWYU pragma: keep
#include "sdk/pwm.hpp"
#include "sdk/spi.hpp"
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
    pwm_set_gpio_duty(PIN_DISPLAY_BRIGHTNESS, UINT16_MAX * settings::g_active.display_brightness);
}

float brightness() {
    return settings::g_active.display_brightness;
}

// Initialises the UI. Everything else should be hands off after that.
// FUTURE WORK: Move to second core if we ever run into perf issues.
//              Have a care regarding potential issues w/ interrupts/timers w/ core 0.
bool init_with_ui() {
    auto cfg_display_brightness = pwm_get_default_config();
    pwm_config_set_freq_hz(cfg_display_brightness, DISPLAY_BACKLIGHT_FREQ);
    pwm_init(pwm_gpio_to_slice_num_(PIN_DISPLAY_BRIGHTNESS), &cfg_display_brightness, true);
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

}  // namespace nevermore::display

////////////////////////////////////
// LVGL DRIVER INTERFACE FUNCTIONS
////////////////////////////////////

void LV_DRV_DISP_CMD_DATA(bool value) {
    gpio_put(PIN_DISPLAY_COMMAND, value);
}

void LV_DRV_DISP_RST(bool value) {
    gpio_put(PIN_DISPLAY_RESET, value);
}

void LV_DRV_DISP_SPI_CS(bool) {
    // NOP - got a single device on the SPI bus for now, can always assume active
}

void LV_DRV_DISP_SPI_WR_ARRAY(uint8_t const* src, unsigned len) {
    assert(src);
    [[maybe_unused]] auto n = spi_write_blocking(spi_gpio_bus(PINS_DISPLAY_SPI[0]), src, len);
    assert(n == int(len) && "failed to write octet");
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
