#pragma once

#include "hardware/spi.h"
#include <cstdint>

namespace nevermore::display {

// Initialises the display and the UI.
bool init_with_ui();

void brightness(float power);  // range: [0, 1]
float brightness();            // range: [0, 1]

struct Resolution {
    uint16_t width;
    uint16_t height;
};

constexpr Resolution RESOLUTION{
        .width = 240,
        .height = 240,
};

// HACK: internal API, do not use
spi_inst_t* active_spi();

};  // namespace nevermore::display
