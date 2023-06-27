#pragma once

#include "pico/async_context.h"

// Initialises the UI. Must be done using the same async context as the display.
bool ui_init(async_context_t&);
