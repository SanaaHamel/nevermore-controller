#pragma once

#include "pico/async_context.h"

namespace nevermore::gatt {

// Setup bluetooth and GATT services.
// Caller is responsible for subsequently calling `btstack_run_loop_execute`.
bool init(async_context_t&);

}  // namespace nevermore::gatt
