#pragma once

#include "hardware/spi.h"
#include "pico/async_context.h"

// Initialises the display and the UI.
bool display_and_ui_init(spi_inst_t&, async_context_t&);

// `display_and_ui_init`, but on core 1.
bool display_and_ui_init_on_second_cpu(spi_inst_t&);

struct DisplayResolution {
    uint16_t width;
    uint16_t height;
};

constexpr DisplayResolution DISPLAY_RESOLUTION{
        .width = 240,
        .height = 240,
};
