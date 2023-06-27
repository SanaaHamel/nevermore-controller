#include "ui.hpp"
#include "display.hpp"
#include "gatt/environmental.hpp"
#include "lvgl.h"
#include "sdk/ble_data_types.hpp"
#include "ui/ui.h"
#include "utility/async_worker.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <initializer_list>

using namespace std;
using namespace std::literals::chrono_literals;

namespace {

constexpr uint8_t CHART_SERIES_ENTIRES_MAX = DISPLAY_RESOLUTION.width / 3;
constexpr auto CHART_X_AXIS_LENGTH = 1.h;

constexpr auto DISPLAY_TIMER_LABELS_INTERVAL = 1s;
constexpr auto DISPLAY_TIMER_CHART_INTERVAL = CHART_X_AXIS_LENGTH / CHART_SERIES_ENTIRES_MAX;

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

bool ui_init(async_context_t& ctx_async) {
    ui_init();  // invoke generated code setup

    lv_chart_set_point_count(ui_Chart, CHART_SERIES_ENTIRES_MAX);
    ui_chart_voc_intake.setup(ui_Chart, LV_CHART_AXIS_PRIMARY_Y, 0xFFFF00);
    ui_chart_voc_exhaust.setup(ui_Chart, LV_CHART_AXIS_PRIMARY_Y, 0x00FFFF);
    ui_chart_temp_intake.setup(ui_Chart, LV_CHART_AXIS_SECONDARY_Y, 0x808000);
    ui_chart_temp_exhaust.setup(ui_Chart, LV_CHART_AXIS_SECONDARY_Y, 0x008080);

    g_display_content_update_timer.register_(ctx_async);
    g_display_chart_update_timer.register_(ctx_async, 1s);
    return true;
}
