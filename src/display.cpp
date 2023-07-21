#include "display.hpp"
#include "config.hpp"
#include "display/GC9A01.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "lvgl.h"
#include "sdk/pwm.hpp"
#include "sdk/timer.hpp"
#include "ui.hpp"
#include "utility/task.hpp"
#include <chrono>
#include <cstdint>
#include <cstdio>

using namespace std;
using namespace std::literals::chrono_literals;
using namespace nevermore;

namespace {
spi_inst_t* g_display_spi;
}

namespace nevermore::display {

namespace {

constexpr auto DISPLAY_BACKLIGHT_FREQ = 1'000;

float g_display_brightness = 1;

lv_color_t g_draw_scratch_buffer[RESOLUTION.width * RESOLUTION.height];
lv_disp_draw_buf_t g_draw_buffer;
lv_disp_drv_t g_driver;
lv_disp_t* g_display;

}  // namespace

void brightness(float power) {
    g_display_brightness = clamp(power, 0.f, 1.f);
    pwm_set_gpio_duty(PIN_DISPLAY_BRIGHTNESS, UINT16_MAX * g_display_brightness);
}

float brightness() {
    return g_display_brightness;
}

// Initialises the UI. Everything else should be hands off after that.
// FUTURE WORK: Move to second core if we ever run into perf issues.
//              Have a care regarding potential issues w/ interrupts/timers w/ core 0.
bool init_with_ui(spi_inst_t& spi) {
    assert(!g_display_spi && "already initialised?");  // likely an error to attempt repeat init
    g_display_spi = &spi;

    auto cfg_display_brightness = pwm_get_default_config();
    pwm_config_set_freq_hz(cfg_display_brightness, DISPLAY_BACKLIGHT_FREQ);
    pwm_init(pwm_gpio_to_slice_num_(PIN_DISPLAY_BRIGHTNESS), &cfg_display_brightness, true);
    brightness(1);

    if (auto e = GC9A01_init(); e != 0) {
        printf("ERR - ui_init - GC9A01_init failed = %d\n", e);
        return false;
    }

    lv_init();
    lv_disp_draw_buf_init(&g_draw_buffer, g_draw_scratch_buffer, nullptr, size(g_draw_scratch_buffer));
    lv_disp_drv_init(&g_driver);
    g_driver.hor_res = RESOLUTION.width;
    g_driver.ver_res = RESOLUTION.height;
    g_driver.draw_buf = &g_draw_buffer;
    // FUTURE WORK: use non-blocking SPI (DMA + interrupt) for faster flush (and add 2-buffer mode?)
    // FUTURE WORK: use 2-data mode + PIO to double tx bandwidth
    // Both of these will require a custom flush impl'.
    g_driver.flush_cb = GC9A01_flush;

    if (g_display = lv_disp_drv_register(&g_driver); !g_display) {
        printf("ERR - ui_init - lv_disp_drv_register returned null\n");
        return false;
    }

    // must finish init-ing the UI *before* we setup the display timer (which could otherwise interrupt)
    if (!ui::init()) return false;

    return true;
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
    assert(g_display_spi && "SPI not yet ready");
}

void LV_DRV_DISP_SPI_WR_ARRAY(uint8_t const* src, unsigned len) {
    assert(g_display_spi && "SPI not yet ready");
    assert(src);
    [[maybe_unused]] auto n = spi_write_blocking(g_display_spi, src, len);
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
    busy_wait(chrono::milliseconds(ms));
}
