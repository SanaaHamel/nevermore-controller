#pragma once

#include "pico/async_context.h"

// Setup bluetooth and GATT services.
// Caller is responsible for subsequently calling `btstack_run_loop_execute`.
bool gatt_init(async_context_t&);
