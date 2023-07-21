#pragma once

#include <cstdint>
#include <span>

namespace nevermore::ws2812 {

void init();

// returns false if the update couldn't be applied for whatever reason
bool update(size_t offset, std::span<uint8_t const> pixel_data);
// returns false if unable to setup (e.g. insufficent memory, etc)
// NB: We deal in total number of pixel components b/c a user could have a heterogenous pixel chain.
bool setup(size_t num_components_total);
size_t components_total();

}  // namespace nevermore::ws2812
