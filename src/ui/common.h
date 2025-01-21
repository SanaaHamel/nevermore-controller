#pragma once

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// NOLINTNEXTLINE(modernize-use-using)
typedef struct NevermoreDisplayUI {
    lv_obj_t* screen;  // required
    /* lv_label_t */ lv_obj_t* temp_in;
    /* lv_label_t */ lv_obj_t* temp_out;
    /* lv_label_t */ lv_obj_t* pressure_in;
    /* lv_label_t */ lv_obj_t* pressure_out;
    /* lv_label_t */ lv_obj_t* humidity_in;
    /* lv_label_t */ lv_obj_t* humidity_out;
    /* lv_label_t */ lv_obj_t* voc_in;
    /* lv_label_t */ lv_obj_t* voc_out;
    /* lv_chart_t */ lv_obj_t* chart;
    /* lv_label_t */ lv_obj_t* chart_max;
    /* lv_label_t */ lv_obj_t* chart_x_axis_scale;
    /* lv_label_t */ lv_obj_t* fan_power;
    /* lv_label_t */ lv_obj_t* fan_rpm;
    /* lv_arc_t */ lv_obj_t* fan_power_arc;
    /* lv_obj_t */ lv_obj_t* touch_overlay;
    /* lv_obj_t */ lv_obj_t* startup_overlay;
} NevermoreDisplayUI;

#ifdef __cplusplus
}
#endif
