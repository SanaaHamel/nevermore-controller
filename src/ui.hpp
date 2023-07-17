#pragma once

#include "pico/async_context.h"

namespace nevermore::ui {

// Initialises the UI. Must be done using the same async context as the display.
bool init(async_context_t&);

}  // namespace nevermore::ui
