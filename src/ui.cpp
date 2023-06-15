#include "ui.hpp"
#include "config.hpp"
#include "display/GC9A01.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/cyw43_arch.h"
#include "sdk/async.hpp"
#include "sdk/timer.hpp"
#include "ui/ui.h"
#include <algorithm>
#include <chrono>
#include <cstdio>

using namespace std;
using namespace std::literals::chrono_literals;

namespace {

constexpr auto GC9A01_HOR_RES = 240;
constexpr auto GC9A01_VER_RES = 240;

constexpr auto DISPLAY_TIMER_INTERVAL = 5ms;

lv_color_t g_draw_scratch_buffer[GC9A01_HOR_RES * GC9A01_VER_RES];
lv_disp_draw_buf_t g_draw_buffer;
lv_disp_drv_t g_driver;
lv_disp_t* g_display;
spi_inst_t* g_display_spi;

async_at_time_worker_t g_display_timer{
        .do_work = [](async_context_t* context, async_at_time_worker_t* worker) {
            lv_timer_handler();
            async_using(*context, [&]() {
                async_context_add_at_time_worker_in_ms(context, worker, DISPLAY_TIMER_INTERVAL / 1ms);
            });
        }};

}  // namespace

// Initialises the UI. Everything else should be hands off after that.
bool ui_init(spi_inst_t& spi, async_context_t& ctx_async) {
    assert(!g_display_spi && "already initialised?");  // likely an error to attempt repeat init
    g_display_spi = &spi;

    lv_init();

    if (auto e = GC9A01_init(); e != 0) {
        printf("ERR - ui_init - GC9A01_init failed = %d\n", e);
        return false;
    }

    lv_disp_draw_buf_init(&g_draw_buffer, g_draw_scratch_buffer, nullptr, size(g_draw_scratch_buffer));
    lv_disp_drv_init(&g_driver);
    g_driver.hor_res = GC9A01_HOR_RES;
    g_driver.ver_res = GC9A01_VER_RES;
    g_driver.draw_buf = &g_draw_buffer;
    // FUTURE WORK: use non-blocking SPI (DMA + interrupt) for faster flush (and add 2-buffer mode?)
    // FUTURE WORK: use 2-data mode + PIO to double tx bandwidth
    // Both of these will require a custom flush impl'.
    g_driver.flush_cb = GC9A01_flush;

    if (g_display = lv_disp_drv_register(&g_driver); !g_display) {
        printf("ERR - ui_init - lv_disp_drv_register returned null\n");
        return false;
    }

    ui_init();

    g_display_timer.do_work(&ctx_async, &g_display_timer);
    return true;
}

bool ui_init_on_second_cpu(spi_inst_t& spi) {
    // HACK: for dbg only
    auto& ctx_async = *cyw43_arch_async_context();
    return ui_init(spi, ctx_async);
}

////////////////////////////////////
// LVGL DRIVER INTERFACE FUNCTIONS
////////////////////////////////////

void LV_DRV_DISP_CMD_DATA(bool value) {
    gpio_put(PINS_DISPLAY_CMD, value);
}

void LV_DRV_DISP_RST(bool value) {
    gpio_put(PINS_DISPLAY_RST, value);
}

void LV_DRV_DISP_SPI_CS(bool) {
    // NOP - got a single device on the SPI bus for now, can always assume active
    assert(g_display_spi && "SPI not yet ready");
}

void LV_DRV_DISP_SPI_WR_ARRAY(uint8_t const* src, unsigned len) {
    assert(g_display_spi && "SPI not yet ready");
    assert(src);
    auto n = spi_write_blocking(g_display_spi, src, len);
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
