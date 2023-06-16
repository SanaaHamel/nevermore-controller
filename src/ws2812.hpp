#pragma once

#include "pico/async_context.h"
#include <cstdint>
#include <span>

void ws2812_init(async_context_t&);

// returns false if the update couldn't be applied for whatever reason
bool ws2812_update(size_t offset, std::span<uint8_t const> pixel_data);
// returns false if unable to setup (e.g. insufficent memory, etc)
// NB: We deal in total number of pixel components b/c a user could have a heterogenous pixel chain.
bool ws2812_setup(size_t num_components_total);
size_t ws2812_components_total();
