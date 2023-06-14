#pragma once

#include "pico/async_context.h"
#include <cstdint>
#include <span>

void ws2812_init(async_context_t&);

// returns false if the update couldn't be applied for whatever reason
bool ws2812_update(size_t offset, std::span<uint8_t const> pixel_data);
// returns false if unable to setup (e.g. insufficent memory, etc)
bool ws2812_setup(uint8_t num_pixels, uint8_t components_per_pixel);
