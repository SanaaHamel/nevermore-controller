#include "ui.hpp"
#include "display.hpp"
#include "gatt/environmental.hpp"
#include "gatt/fan.hpp"
#include "lvgl.h"
#include "sdk/ble_data_types.hpp"
#include "ui/ui.h"
#include "utility/async_worker.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <initializer_list>
#include <string>

using namespace std;
using namespace std::literals::chrono_literals;

namespace {

constexpr auto CHART_X_AXIS_LENGTH = 1.h;  // 1.h;
constexpr lv_opa_t CHART_RED_ZONE_HI = LV_OPA_30;
constexpr uint8_t CHART_SERIES_ENTIRES_MAX = DISPLAY_RESOLUTION.width / 3;

constexpr auto DISPLAY_TIMER_CHART_INTERVAL = CHART_X_AXIS_LENGTH / CHART_SERIES_ENTIRES_MAX;
constexpr auto DISPLAY_TIMER_LABELS_INTERVAL = 1s;

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

lv_point_t top_left(lv_area_t const& coord) {
    return {.x = coord.x1, .y = coord.y1};
}

[[maybe_unused]] lv_point_t operator+(lv_point_t const& l, lv_point_t const& r) {
    return {.x = lv_coord_t(l.x + r.x), .y = lv_coord_t(l.y + r.y)};
}

template <typename A, typename Ratio>
auto pretty_print_time(std::chrono::duration<A, Ratio> const& dur, char const* spec = "%.f") {
    auto format = [&](char const* unit, double value) -> std::string {
        char buffer[256];  // b/c we apparently don't have `<format>` yet in GCC 12.2.1
        auto n = sprintf(buffer, spec, double(value));
        assert(0 <= n);
        strcpy(buffer + n, unit);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        return buffer;
    };

    if (dur < 1ms) return format("us", dur / 1.us);
    if (dur < 1s) return format("ms", dur / 1.ms);
    if (dur < 1min) return format("s", dur / 1.s);
    if (dur < 1h) return format("min", dur / 1.min);
    return format("h", dur / 1.h);
}

template <typename A>
auto label_set(lv_obj_t* obj, char const* unk, char const* fmt, A&& value, double scale = 1) {
    if (value == BLE::NOT_KNOWN) {
        lv_label_set_text(obj, unk);
        return;
    }

    char buffer[256];  // b/c we apparently don't have `<format>` yet in GCC 12.2.1
    sprintf(buffer, fmt, double(value) / scale);
    lv_label_set_text(obj, buffer);
};

// returns point relative to `obj`
auto chart_pos_for_value(
        lv_obj_t const* obj, lv_chart_series_t const* series, lv_coord_t value, bool last = false) {
    // HACK: there is no API to do this, so we have to mug an existing value (temporarily)
    auto id = last ? lv_chart_get_point_count(obj) - 1 : 0;
    auto* y_value = series->y_points + id;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    auto old = *y_value;
    *y_value = value;
    lv_point_t pos;
    lv_chart_get_point_pos_by_id(  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
            const_cast<lv_obj_t*>(obj), const_cast<lv_chart_series_t*>(series), id, &pos);
    *y_value = old;
    return pos;
};

auto g_display_content_update_timer = mk_async_worker<DISPLAY_TIMER_LABELS_INTERVAL / 1ms>([]() {
    auto& state = EnvironmentService::g_service_data;

    label_set(ui_PressureIn, "??? kPa", "%.f kPa", state.pressure_intake, 1e3);
    label_set(ui_PressureOut, "??? kPa", "%.f kPa", state.pressure_exhaust, 1e3);
    label_set(ui_HumidityIn, "??%", "%2.1f%%", state.humidity_intake);
    label_set(ui_HumidityOut, "??%", "%2.1f%%", state.humidity_exhaust);

    label_set(ui_VocIn, "??? VOC", "%.0f VOC", state.voc_index_intake);
    label_set(ui_VocOut, "??? VOC", "%.0f VOC", state.voc_index_exhaust);
    label_set(ui_TempIn, "?.?c", "%.1fc", state.temperature_intake);
    label_set(ui_TempOut, "?.?c", "%.1fc", state.temperature_exhaust);

    label_set(ui_FanPower, "", "%.0f%%", BLE::Percentage8(ceil(FanService::fan_power())));
});

auto g_display_chart_update_timer = mk_async_worker<uint32_t(DISPLAY_TIMER_CHART_INTERVAL / 1ms)>([]() {
    auto& state = EnvironmentService::g_service_data;

    if (lv_chart_get_point_count(ui_Chart) < CHART_SERIES_ENTIRES_MAX) {
        // extend # of points until maximum
        auto n = lv_chart_get_point_count(ui_Chart) + 1;
        // HACK:  Directly set the point count w/o using `lv_chart_set_point_count`
        //        because that function resets the next-point for each series to 0.
        //        (Sane if the # of points goes down, but not so much for our case.)
        reinterpret_cast<lv_chart_t*>(ui_Chart)->point_cnt = n;

        lv_label_set_text(ui_XAxisScale, pretty_print_time(n * DISPLAY_TIMER_CHART_INTERVAL).c_str());
    }

    lv_chart_set_next_value(ui_Chart, ui_chart_voc_intake.ui, state.voc_index_intake.value_or(0));
    lv_chart_set_next_value(ui_Chart, ui_chart_voc_exhaust.ui, state.voc_index_exhaust.value_or(0));
    lv_chart_set_next_value(ui_Chart, ui_chart_temp_intake.ui, state.temperature_intake.value_or(0));
    lv_chart_set_next_value(ui_Chart, ui_chart_temp_exhaust.ui, state.temperature_exhaust.value_or(0));

    auto scale_axis = [](lv_chart_axis_t axis, lv_coord_t top_min, lv_coord_t step,
                              initializer_list<Series> const& xs) {
        lv_coord_t top = 0;
        for (auto&& x : xs)
            top = max(top, *std::max_element(x.values.begin(), x.values.end()));

        auto ticks = 1 + (max(top_min, top) + step - 1) / step;
        lv_chart_set_range(ui_Chart, axis, 0, lv_coord_t(ticks * step));
        return ticks;
    };

    auto ticks_y = scale_axis(LV_CHART_AXIS_PRIMARY_Y, 100, 25, {ui_chart_voc_intake, ui_chart_voc_exhaust});
    scale_axis(LV_CHART_AXIS_SECONDARY_Y, 50, 25, {ui_chart_temp_intake, ui_chart_temp_exhaust});

    lv_chart_set_div_line_count(ui_Chart, ticks_y + 1, 10);
});

void on_chart_draw(bool begin, lv_event_t* e) {
    auto* obj = lv_event_get_target(e);
    auto const& desc = *lv_event_get_draw_part_dsc(e);

    auto abs_pos_for_value = [&](lv_chart_series_t const* series, lv_coord_t value, bool last = false) {
        return chart_pos_for_value(obj, series, value, last) + top_left(obj->coords);
    };

    switch (desc.part) {
    default: break;

    case LV_PART_MAIN: {
        // draw the VOC clean-line after all other lines
        if (begin || desc.p1 || desc.p2 || !desc.line_dsc) break;

        constexpr lv_coord_t VOC_LINE_PAD = 0;
        desc.line_dsc->color = lv_color_make(0, 255, 0);
        desc.line_dsc->opa = LV_OPA_50;
        desc.line_dsc->width = 2;
        desc.line_dsc->dash_gap = 6;
        desc.line_dsc->dash_width = 6;
        auto p1 = abs_pos_for_value(ui_chart_voc_intake.ui, 100, false);
        auto p2 = abs_pos_for_value(ui_chart_voc_intake.ui, 100, true);
        p1.x -= VOC_LINE_PAD;
        p2.x += VOC_LINE_PAD;
        lv_draw_line(desc.draw_ctx, desc.line_dsc, &p1, &p2);
    } break;

    case LV_PART_ITEMS: {
        // draw before a line segment
        if (!begin || !desc.p1 || !desc.p2) break;

        auto const* series = reinterpret_cast<lv_chart_series_t const*>(desc.sub_part_ptr);
        if (series->y_axis_sec) break;  // no BG fade for temperature series

        // HACK: Want to shade only the area under either VOC curve.
        //       Nominally `intake` should be >= exhaust, so cheat for now and
        //       only shade 'neath intake curve.
        //       Bonus: Since intake is drawn before exhaust (registered first),
        //              we do not draw over the exhaust line.
        if (series != ui_chart_voc_intake.ui) break;

        auto y_fade_top = abs_pos_for_value(series, 200).y;
        auto y_fade_bot = abs_pos_for_value(series, 100).y;
        // everything below the line has 0 opacity -> no-op
        if (y_fade_bot < min(desc.p1->y, desc.p2->y)) return;

        // mask everything above line
        lv_draw_mask_line_param_t line_mask_param{};
        lv_draw_mask_line_points_init(&line_mask_param, desc.p1->x, desc.p1->y, desc.p2->x, desc.p2->y,
                LV_DRAW_MASK_LINE_SIDE_BOTTOM);
        auto line_mask_id = lv_draw_mask_add(&line_mask_param, {});

        // fade from line being drawn to 100-voc-index line
        lv_draw_mask_fade_param_t fade_mask_param{};
        lv_draw_mask_fade_init(
                &fade_mask_param, &obj->coords, LV_OPA_30, y_fade_top, LV_OPA_TRANSP, y_fade_bot);
        auto fade_mask_id = lv_draw_mask_add(&fade_mask_param, {});

        lv_draw_rect_dsc_t draw_rect_dsc{};
        lv_draw_rect_dsc_init(&draw_rect_dsc);
        draw_rect_dsc.bg_opa = LV_OPA_COVER;
        draw_rect_dsc.bg_color = lv_color_make(255, 0, 0);
        lv_area_t a{
                .x1 = desc.p1->x,
                .y1 = min(desc.p1->y, desc.p2->y),
                .x2 = lv_coord_t(desc.p2->x - 1),
                .y2 = y_fade_bot,
        };
        lv_draw_rect(desc.draw_ctx, &draw_rect_dsc, &a);

        lv_draw_mask_free_param(&line_mask_param);
        lv_draw_mask_free_param(&fade_mask_param);
        lv_draw_mask_remove_id(line_mask_id);
        lv_draw_mask_remove_id(fade_mask_id);
    } break;
    }
}

}  // namespace

bool ui_init(async_context_t& ctx_async) {
    ui_init();  // invoke generated code setup

    lv_chart_set_point_count(ui_Chart, 1);
    ui_chart_voc_intake.setup(ui_Chart, LV_CHART_AXIS_PRIMARY_Y, 0xFFFF00);
    ui_chart_voc_exhaust.setup(ui_Chart, LV_CHART_AXIS_PRIMARY_Y, 0x00FFFF);
    ui_chart_temp_intake.setup(ui_Chart, LV_CHART_AXIS_SECONDARY_Y, 0x808000);
    ui_chart_temp_exhaust.setup(ui_Chart, LV_CHART_AXIS_SECONDARY_Y, 0x008080);

    lv_obj_add_event_cb(ui_Chart, [](auto* e) { on_chart_draw(true, e); }, LV_EVENT_DRAW_PART_BEGIN, {});
    lv_obj_add_event_cb(ui_Chart, [](auto* e) { on_chart_draw(false, e); }, LV_EVENT_DRAW_PART_END, {});

#if 0  // DEBUG HELPER - pre-populate chart with some data to test rendering
    lv_chart_set_point_count(ui_Chart, CHART_SERIES_ENTIRES_MAX);
    lv_label_set_text(ui_XAxisScale,
            pretty_print_time(CHART_SERIES_ENTIRES_MAX * DISPLAY_TIMER_CHART_INTERVAL).c_str());
    for (uint i = 0; i < CHART_SERIES_ENTIRES_MAX; ++i) {
        auto p = double(i) / (CHART_SERIES_ENTIRES_MAX - 1);
        lv_chart_set_next_value(ui_Chart, ui_chart_voc_intake.ui, p * 250);
    }
#endif

    g_display_content_update_timer.register_(ctx_async);
    g_display_chart_update_timer.register_(ctx_async, 1s);
    return true;
}
