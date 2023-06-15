#pragma once

#include "hardware/spi.h"
#include "pico/async_context.h"

// Initialises the UI. Everything else should be hands off after that.
bool ui_init(spi_inst_t&, async_context_t&);

// `ui_init`, but on core 1.
bool ui_init_on_second_cpu(spi_inst_t&);
