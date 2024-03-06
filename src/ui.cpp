#include "ui.hpp"
#include "FreeRTOS.h"
#include "display.hpp"
#include "gatt/fan.hpp"
#include "lvgl.h"
#include "sdk/ble_data_types.hpp"
#include "semphr.h"
#include "sensors.hpp"
#include "settings.hpp"
#include "ui/circle_240/ui.hpp"
#include "utility/task.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <tuple>

using namespace std;
using namespace std::literals::chrono_literals;

namespace nevermore::ui {

namespace {

constexpr bool CHAR_DRAW_TEMPERATURE_HDIV = false;

constexpr auto CHART_X_AXIS_LENGTH = 1.h;
constexpr uint8_t CHART_SERIES_ENTIRES_MAX = display::RESOLUTION.width / 3;

constexpr auto DISPLAY_TIMER_PLOT_INTERVAL = CHART_X_AXIS_LENGTH / CHART_SERIES_ENTIRES_MAX;
constexpr auto DISPLAY_TIMER_LABELS_INTERVAL = 1s;
constexpr auto DISPLAY_REFRESH_INTERVAL = 5ms;

struct ChartDivY {
    uint8_t min;
    lv_coord_t value_per;
};

constexpr ChartDivY CHART_DIV_VOC{.min = 5, .value_per = 50};
constexpr ChartDivY CHART_DIV_TEMP{.min = 6, .value_per = 10};
constexpr lv_opa_t CHART_RED_ZONE_HI = LV_OPA_30;

bool chart_point_less_than(lv_coord_t x, lv_coord_t y) {
    if (y == LV_CHART_POINT_NONE) return false;
    if (x == LV_CHART_POINT_NONE) return true;
    return x < y;
}

struct Series {
    lv_chart_series_t* ui = {};
    array<lv_coord_t, CHART_SERIES_ENTIRES_MAX> values{};

    Series() {
        values.fill(LV_CHART_POINT_NONE);
    }

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
NevermoreDisplayUI ui;

lv_point_t top_left(lv_area_t const& coord) {
    return {.x = coord.x1, .y = coord.y1};
}

[[maybe_unused]] lv_point_t operator+(lv_point_t const& l, lv_point_t const& r) {
    return {.x = lv_coord_t(l.x + r.x), .y = lv_coord_t(l.y + r.y)};
}

template <typename A, typename Ratio>
auto pretty_print_time(std::chrono::duration<A, Ratio> const& dur, char const* spec = "%.f") {
    auto format = [&](char const* unit, double value) -> std::string {
        auto n = snprintf(nullptr, 0, spec, double(value));
        assert(0 <= n);
        char buffer[n + strlen(unit) + 1];  // b/c we apparently don't have `<format>` yet in GCC 12.2.1
        sprintf(buffer, spec, double(value));
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
    if (!obj) return;

    if constexpr (BLE::has_not_known<std::decay_t<A>>) {
        if (value == BLE::NOT_KNOWN) {
            lv_label_set_text(obj, unk);
            return;
        }
    }

    // b/c we apparently don't have `<format>` yet in GCC 12.2.1
    char buffer[32];  // labels should be short, minimise stack usage
    snprintf(buffer, size(buffer), fmt, double(value) / scale);
    lv_label_set_text(obj, buffer);
};

double lv_arc_get_percent(lv_obj_t const* obj) {
    auto range = lv_arc_get_max_value(obj) - lv_arc_get_min_value(obj);
    if (range == 0) return 0;

    return lv_arc_get_value(obj) / double(range);
}

void lv_arc_set_percent(lv_obj_t* obj, double perc) {
    auto range = lv_arc_get_max_value(obj) - lv_arc_get_min_value(obj);
    lv_arc_set_value(obj, int16_t(lv_arc_get_min_value(obj) + perc * range));
}

// returns point relative to `obj`
auto chart_pos_for_value(
        lv_obj_t const* obj, lv_chart_series_t const* series, lv_coord_t value, bool last = false) {
    // HACK: there is no API to do this, so we have to mug an existing value (temporarily)
    auto id = last ? lv_chart_get_point_count(obj) - 1 : 0;
    auto id_start = lv_chart_get_x_start_point(  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
            obj, const_cast<lv_chart_series_t*>(series));
    auto id_storage = ((int32_t)id_start + id) % lv_chart_get_point_count(obj);

    auto* y_value = series->y_points + id_storage;  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    auto old = *y_value;
    *y_value = value;
    lv_point_t pos;
    lv_chart_get_point_pos_by_id(  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
            const_cast<lv_obj_t*>(obj), const_cast<lv_chart_series_t*>(series), id, &pos);
    *y_value = old;
    return pos;
};

void fan_power_arc_colour_update() {
    if (!ui.fan_power_arc) return;

    lv_obj_set_style_arc_color(ui.fan_power_arc,
            lv_color_hex(gatt::fan::fan_power_override() == BLE::NOT_KNOWN ? 0x00FFFF : 0xFFFF00),
            LV_PART_INDICATOR | int(LV_STATE_DEFAULT));
}

void display_update_labels() {
    auto const& state = nevermore::sensors::g_sensors.with_fallbacks();

    label_set(ui.pressure_in, "--- hPa", "%.1f hPa", state.pressure_intake, 1e2);
    label_set(ui.pressure_out, "--- hPa", "%.1f hPa", state.pressure_exhaust, 1e2);
    label_set(ui.humidity_in, "--- %", "%2.1f%%", state.humidity_intake);
    label_set(ui.humidity_out, "--- %", "%2.1f%%", state.humidity_exhaust);

    label_set(ui.voc_in, "---", "%.0f", state.voc_index_intake);
    label_set(ui.voc_out, "---", "%.0f", state.voc_index_exhaust);
    label_set(ui.temp_in, "--- c", "%.1fc", state.temperature_intake);
    label_set(ui.temp_out, "--- c", "%.1fc", state.temperature_exhaust);

    label_set(ui.fan_power, "", "%.0f%%", BLE::Percentage8(ceil(gatt::fan::fan_power())));
    label_set(ui.fan_rpm, "", "%.0f", gatt::fan::fan_rpm());

    if (ui.fan_power_arc) {
        lv_arc_set_percent(ui.fan_power_arc, gatt::fan::fan_power() / 100);
        fan_power_arc_colour_update();
    }
}

void display_update_plot() {
    if (!ui.chart) return;

    auto const& state = nevermore::sensors::g_sensors.with_fallbacks();

    if (lv_chart_get_point_count(ui.chart) < CHART_SERIES_ENTIRES_MAX) {
        // extend # of points until maximum
        auto n = lv_chart_get_point_count(ui.chart) + 1;
        // HACK:  Directly set the point count w/o using `lv_chart_set_point_count`
        //        because that function resets the next-point for each series to 0.
        //        (Sane if the # of points goes down, but not so much for our case.)
        reinterpret_cast<lv_chart_t*>(ui.chart)->point_cnt = n;

        if (ui.chart_x_axis_scale) {
            lv_label_set_text(
                    ui.chart_x_axis_scale, pretty_print_time(n * DISPLAY_TIMER_PLOT_INTERVAL).c_str());
        }
    }

    auto set_next_value = [&](auto* series, auto&& value) {
        lv_chart_set_next_value(ui.chart, series, value.value_or(LV_CHART_POINT_NONE));
    };

    set_next_value(ui_chart_voc_intake.ui, state.voc_index_intake);
    set_next_value(ui_chart_voc_exhaust.ui, state.voc_index_exhaust);
    set_next_value(ui_chart_temp_intake.ui, state.temperature_intake);
    set_next_value(ui_chart_temp_exhaust.ui, state.temperature_exhaust);

    auto scale_axis = [](lv_chart_axis_t axis, ChartDivY const& div, initializer_list<Series> const& xs) {
        // TODO: handle case where plot coords are < 0 (why are you running your printer in a freezer?)
        lv_coord_t top = 0;
        for (auto&& x : xs) {
            auto val = *std::max_element(x.values.begin(), x.values.end(), chart_point_less_than);
            if (val != LV_CHART_POINT_NONE) {
                top = max(top, val);
            }
        }

        auto lines = max<uint>(div.min, 1 + (top + div.value_per - 1) / div.value_per);
        auto coord = lv_coord_t(lines * div.value_per);
        lv_chart_set_range(ui.chart, axis, 0, coord);
        return tuple{lines, coord};
    };

    auto [lines_voc, max_voc] =
            scale_axis(LV_CHART_AXIS_PRIMARY_Y, CHART_DIV_VOC, {ui_chart_voc_intake, ui_chart_voc_exhaust});
    auto [_, max_temp] = scale_axis(
            LV_CHART_AXIS_SECONDARY_Y, CHART_DIV_TEMP, {ui_chart_temp_intake, ui_chart_temp_exhaust});

    lv_chart_set_div_line_count(ui.chart, lines_voc + 1, 10);

    if (ui.chart_max) {
        char buffer[256];
        sprintf(buffer, "%u VOC\n%uc", max_voc, max_temp);
        lv_label_set_text(ui.chart_max, buffer);
    }
}

// Code more or less ripped from LVGL's `lv_chart.c`.
[[maybe_unused]] void chart_draw_hdivs(lv_draw_ctx_t& draw_ctx, lv_draw_line_dsc_t const& line_desc,
        lv_chart_t const& chart, uint16_t hdiv_cnt) {
    if (hdiv_cnt <= 0) return;

    auto const border_opa = lv_obj_get_style_border_opa(&chart.obj, LV_PART_MAIN);
    auto const border_side = lv_obj_get_style_border_side(&chart.obj, LV_PART_MAIN);
    auto const border_width = lv_obj_get_style_border_width(&chart.obj, LV_PART_MAIN);
    auto const pad_top = lv_obj_get_style_pad_top(&chart.obj, LV_PART_MAIN) + border_width;
    // WORKAROUND: `lv_obj_get_scroll_top` mistakenly lacks a `const` qualifier
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    auto const scroll_top = lv_obj_get_scroll_top(const_cast<lv_obj_t*>(&chart.obj));
    auto const y_ofs = chart.obj.coords.y1 + pad_top - scroll_top;
    auto const h = (lv_obj_get_content_height(&chart.obj) * chart.zoom_y) >> 8;
    static_assert(sizeof(int32_t) <= sizeof(int), "some of this code assumes <= 32 bits");

    lv_point_t p1{.x = chart.obj.coords.x1};
    lv_point_t p2{.x = chart.obj.coords.x2};

    auto i_start = 0;
    auto i_end = hdiv_cnt;
    if (LV_OPA_MIN < border_opa && 0 < border_width) {
        if ((border_side & LV_BORDER_SIDE_TOP) && lv_obj_get_style_pad_top(&chart.obj, LV_PART_MAIN) == 0)
            i_start++;

        if ((border_side & LV_BORDER_SIDE_BOTTOM) &&
                lv_obj_get_style_pad_bottom(&chart.obj, LV_PART_MAIN) == 0)
            i_end--;
    }

    for (auto i = i_start; i < i_end; i++) {
        p1.y = y_ofs + (h * i) / (hdiv_cnt - 1);
        p2.y = p1.y;

        lv_draw_line(&draw_ctx, &line_desc, &p1, &p2);
    }
}

void on_chart_draw(bool begin, lv_event_t* e) {
    auto* obj = lv_event_get_target(e);
    auto const& chart = *reinterpret_cast<lv_chart_t const*>(obj);
    auto const& desc = *lv_event_get_draw_part_dsc(e);

    auto abs_pos_for_value = [&](lv_chart_series_t const* series, lv_coord_t value, bool last = false) {
        return chart_pos_for_value(obj, series, value, last) + top_left(obj->coords);
    };

    switch (desc.part) {
    default: break;

    case LV_PART_MAIN: {
        if (desc.p1 || desc.p2 || !desc.line_dsc) break;  // drawing main lines, or no line-info

        if (begin) {
            if constexpr (CHAR_DRAW_TEMPERATURE_HDIV) {
                // draw the secondary axis division lines before the primary axis lines
                auto y_secondary_range = max(0, chart.ymax[1] - chart.ymin[1]);
                auto hdiv_cnt = 1 + (y_secondary_range / CHART_DIV_TEMP.value_per);

                auto line_desc = *desc.line_dsc;
                line_desc.dash_gap = 6;
                line_desc.dash_width = 6;
                chart_draw_hdivs(*desc.draw_ctx, line_desc, chart, hdiv_cnt);
            }
        } else {
            // draw the VOC clean-line after all other lines
            lv_draw_line_dsc_t line_desc{
                    .color = lv_color_make(0, 255, 0),
                    .width = 2,
                    .dash_width = 6,
                    .dash_gap = 6,
                    .opa = LV_OPA_50,
            };
            auto p1 = abs_pos_for_value(ui_chart_voc_intake.ui, 100, false);
            auto p2 = abs_pos_for_value(ui_chart_voc_intake.ui, 100, true);
            lv_draw_line(desc.draw_ctx, &line_desc, &p1, &p2);
        }
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

        auto threshold =
                settings::g_active.voc_gating_threshold_override.or_(settings::g_active.voc_gating_threshold);
        if (threshold == BLE::NOT_KNOWN) break;  // no gating threshold? odd.

        // not quite precise for visualising the gating region, but this is for
        // Entertainment Purposes(TM) so who cares...
        auto threshold_mid = int(threshold.value_or(0));
        auto threshold_hi = threshold_mid + threshold_mid / 10;
        auto threshold_lo = threshold_mid - threshold_mid / 10;
        if (threshold_lo == threshold_hi) break;  // too small to visualise, don't bother

        auto y_fade_top = abs_pos_for_value(series, lv_coord_t(threshold_hi)).y;
        auto y_fade_bot = abs_pos_for_value(series, lv_coord_t(threshold_lo)).y;
        // everything below the line has 0 opacity -> no-op
        if (y_fade_bot < min(desc.p1->y, desc.p2->y)) break;

        // mask everything above line
        lv_draw_mask_line_param_t line_mask_param{};
        lv_draw_mask_line_points_init(&line_mask_param, desc.p1->x, desc.p1->y, desc.p2->x, desc.p2->y,
                LV_DRAW_MASK_LINE_SIDE_BOTTOM);
        auto line_mask_id = lv_draw_mask_add(&line_mask_param, {});

        // fade from line being drawn to 100-voc-index line
        lv_draw_mask_fade_param_t fade_mask_param{};
        lv_draw_mask_fade_init(
                &fade_mask_param, &obj->coords, CHART_RED_ZONE_HI, y_fade_top, LV_OPA_TRANSP, y_fade_bot);
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

SemaphoreHandle_t g_ui_lock;
auto using_semaphore(SemaphoreHandle_t& lock, TickType_t ticks_to_wait = portMAX_DELAY) {
    return [=](auto&& go) {
        if (!xSemaphoreTake(lock, ticks_to_wait)) return false;

        go();
        xSemaphoreGive(lock);
        return true;
    };
}

}  // namespace

bool init() {
    g_ui_lock = xSemaphoreCreateMutex();  // we panic on alloc failures, no need to handle null

    using enum settings::DisplayUI;
    switch (settings::g_active.display_ui) {
    case CIRCLE_240_CLASSIC: {
        ui = circle_240::classic();
    } break;
    case CIRCLE_240_SMALL_PLOT: {
        ui = circle_240::small_plot();
    } break;
    case CIRCLE_240_NO_PLOT: {
        ui = circle_240::no_plot();
    } break;

    default: {
        printf("ERR - ui::init - unsupported display UI %u\n", (unsigned)settings::g_active.display_ui);
        return false;
    } break;
    }

    // HACK: Need at least 2 points to draw the 100-VOC line.
    if (ui.chart) {
        lv_chart_set_point_count(ui.chart, 2);
        ui_chart_voc_intake.setup(ui.chart, LV_CHART_AXIS_PRIMARY_Y, 0xFFFF00);
        ui_chart_voc_exhaust.setup(ui.chart, LV_CHART_AXIS_PRIMARY_Y, 0x00FFFF);
        ui_chart_temp_intake.setup(ui.chart, LV_CHART_AXIS_SECONDARY_Y, 0x808000);
        ui_chart_temp_exhaust.setup(ui.chart, LV_CHART_AXIS_SECONDARY_Y, 0x008080);

        lv_obj_add_event_cb(ui.chart, [](auto* e) { on_chart_draw(true, e); }, LV_EVENT_DRAW_PART_BEGIN, {});
        lv_obj_add_event_cb(ui.chart, [](auto* e) { on_chart_draw(false, e); }, LV_EVENT_DRAW_PART_END, {});
    }

    if (ui.touch_overlay) {
        lv_obj_add_event_cb(ui.touch_overlay,
                [](auto*) {
                    gatt::fan::fan_power_override(gatt::fan::fan_power_override() == BLE::NOT_KNOWN
                                                          ? BLE::Percentage8(100)
                                                          : BLE::NOT_KNOWN);
                },
                LV_EVENT_LONG_PRESSED, {});
    }

    if (ui.fan_power_arc) {
        lv_obj_add_event_cb(ui.fan_power_arc,
                [](lv_event_t* e) {
                    auto state = lv_obj_get_state(ui.fan_power_arc);
                    if (!(state & LV_STATE_PRESSED)) return;

                    BLE::Percentage8 power = lv_arc_get_percent(ui.fan_power_arc) * 100;
                    if (power == 0) {
                        power = BLE::NOT_KNOWN;  // clear override if dragged to zero
                    }

                    gatt::fan::fan_power_override(power);
                    fan_power_arc_colour_update();
                },
                LV_EVENT_VALUE_CHANGED, {});
    }

#if 0  // DEBUG HELPER - pre-populate chart with some data to test rendering
    if (ui.chart) {
        lv_chart_set_point_count(ui.chart, CHART_SERIES_ENTIRES_MAX);
        if (ui.chart_x_axis_scale) {
            lv_label_set_text(ui.chart_x_axis_scale,
                    pretty_print_time(CHART_SERIES_ENTIRES_MAX * DISPLAY_TIMER_PLOT_INTERVAL).c_str());
        }
        for (uint i = 0; i < CHART_SERIES_ENTIRES_MAX; ++i) {
            auto p = double(i) / (CHART_SERIES_ENTIRES_MAX - 1);
            lv_chart_set_next_value(ui.chart, ui_chart_voc_intake.ui, p * 500);
        }
    }
#endif

#define DISPLAY_TASK(name, period, stack_size, go)                \
    mk_task(name, Priority::Display, stack_size)([]() {           \
        periodic(period)([] { using_semaphore(g_ui_lock)(go); }); \
    }).release()

    // must finish init-ing the UI *before* we start `lv_timer_handler` (which could otherwise interrupt)
    DISPLAY_TASK("display", DISPLAY_REFRESH_INTERVAL, 1024, lv_timer_handler);
    DISPLAY_TASK("display-label", DISPLAY_TIMER_LABELS_INTERVAL, 1024, display_update_labels);
    DISPLAY_TASK("display-plot", DISPLAY_TIMER_PLOT_INTERVAL, 1024, display_update_plot);
    return true;
}

}  // namespace nevermore::ui
