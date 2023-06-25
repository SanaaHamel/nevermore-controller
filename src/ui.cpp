#include "ui.hpp"
#include "config.hpp"
#include "display/GC9A01.h"
#include "gatt/environmental.hpp"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "lvgl.h"
#include "pico/cyw43_arch.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/timer.hpp"
#include "ui/ui.h"
#include "utility/async_worker.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <initializer_list>

using namespace std;
using namespace std::literals::chrono_literals;

namespace {

constexpr auto GC9A01_HOR_RES = 240;
constexpr auto GC9A01_VER_RES = 240;

constexpr uint8_t CHART_SERIES_ENTIRES_MAX = GC9A01_HOR_RES / 3;
constexpr auto CHART_X_AXIS_LENGTH = 1.h;

constexpr auto DISPLAY_TIMER_INTERVAL = 5ms;
constexpr auto DISPLAY_TIMER_LABELS_INTERVAL = 1s;
constexpr auto DISPLAY_TIMER_CHART_INTERVAL = CHART_X_AXIS_LENGTH / CHART_SERIES_ENTIRES_MAX;

lv_color_t g_draw_scratch_buffer[GC9A01_HOR_RES * GC9A01_VER_RES];
lv_disp_draw_buf_t g_draw_buffer;
lv_disp_drv_t g_driver;
lv_disp_t* g_display;
spi_inst_t* g_display_spi;

struct Series {
    lv_chart_series_t* ui = {};
    array<lv_coord_t, CHART_SERIES_ENTIRES_MAX> values{};

    void setup(lv_obj_t* chart, lv_chart_axis_t axis, uint32_t clr) {
        assert(!ui);
        ui = lv_chart_add_series(chart, lv_color_hex(clr), axis);
        lv_chart_set_ext_y_array(chart, ui, values.data());
    }
};

Series ui_chart_voc_intake;
Series ui_chart_voc_exhaust;
Series ui_chart_temp_intake;
Series ui_chart_temp_exhaust;

auto g_display_timer = mk_sync_worker<DISPLAY_TIMER_INTERVAL / 1ms>(lv_timer_handler);

auto g_display_content_update_timer = mk_async_worker<DISPLAY_TIMER_LABELS_INTERVAL / 1ms>([]() {
    auto& state = EnvironmentService::g_service_data;

    auto label_set = [&](lv_obj_t* obj, char const* unk, char const* fmt, auto&& value, double scale = 1) {
        if (value == BLE::NOT_KNOWN) {
            lv_label_set_text(obj, unk);
            return;
        }

        char buffer[256];  // b/c we apparently don't have `<format>` yet in GCC 12.2.1
        sprintf(buffer, fmt, double(value) / scale);
        lv_label_set_text(obj, buffer);
    };

    label_set(ui_PressureIn, "??? kPa", "%0.f kPa", state.pressure_intake, 1e3);
    label_set(ui_PressureOut, "??? kPa", "%0.f kPa", state.pressure_exhaust, 1e3);
    label_set(ui_HumidityIn, "??%", "%02.1f%%", state.humidity_intake);
    label_set(ui_HumidityOut, "??%", "%02.1f%%", state.humidity_exhaust);
    label_set(ui_VocValueIn, "???/500", "%03.f/500", state.voc_index_intake);
    label_set(ui_VocValueOut, "???/500", "%03.f/500", state.voc_index_exhaust);
    lv_bar_set_value(ui_VocBarIn, state.voc_index_intake.value_or(0), LV_ANIM_OFF);
    lv_bar_set_value(ui_VocBarOut, state.voc_index_exhaust.value_or(0), LV_ANIM_OFF);
});

auto g_display_chart_update_timer = mk_async_worker<uint32_t(DISPLAY_TIMER_CHART_INTERVAL / 1ms)>([]() {
    auto& state = EnvironmentService::g_service_data;
    lv_chart_set_next_value(ui_Chart, ui_chart_voc_intake.ui, state.voc_index_intake.value_or(0));
    lv_chart_set_next_value(ui_Chart, ui_chart_voc_exhaust.ui, state.voc_index_exhaust.value_or(0));
    lv_chart_set_next_value(ui_Chart, ui_chart_temp_intake.ui, state.temperature_intake.value_or(0));
    lv_chart_set_next_value(ui_Chart, ui_chart_temp_exhaust.ui, state.temperature_exhaust.value_or(0));

    auto scale_axis = [](lv_chart_axis_t axis, lv_coord_t top_min, lv_coord_t step,
                              initializer_list<Series> const& xs) {
        lv_coord_t top = 0;
        for (auto&& x : xs)
            top = max(top, *std::max_element(x.values.begin(), x.values.end()));

        auto top_stepped = ((max(top_min, top) + step - 1) / step) * step;
        lv_chart_set_range(ui_Chart, axis, 0, lv_coord_t(top_stepped));
    };

    scale_axis(LV_CHART_AXIS_PRIMARY_Y, 100, 25, {ui_chart_voc_intake, ui_chart_voc_exhaust});
    scale_axis(LV_CHART_AXIS_SECONDARY_Y, 50, 25, {ui_chart_temp_intake, ui_chart_temp_exhaust});
});

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

    lv_chart_set_point_count(ui_Chart, CHART_SERIES_ENTIRES_MAX);
    ui_chart_voc_intake.setup(ui_Chart, LV_CHART_AXIS_PRIMARY_Y, 0xFFFF00);
    ui_chart_voc_exhaust.setup(ui_Chart, LV_CHART_AXIS_PRIMARY_Y, 0x00FFFF);
    ui_chart_temp_intake.setup(ui_Chart, LV_CHART_AXIS_SECONDARY_Y, 0x808000);
    ui_chart_temp_exhaust.setup(ui_Chart, LV_CHART_AXIS_SECONDARY_Y, 0x008080);

    g_display_timer.register_(ctx_async);
    g_display_content_update_timer.register_(ctx_async);
    g_display_chart_update_timer.register_(ctx_async, 1s);
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
