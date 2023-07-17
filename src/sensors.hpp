#pragma once

#include "gatt/environmental.hpp"
#include "pico/async_context.h"

// Sensors are registered as periodic workers for the context.
// `EnvironmentService::Sensors` must remain valid for the lifetime of the program.
bool sensors_init(async_context_t&, EnvironmentService::Sensors&);
