#pragma once

#include <cstdint>
#include <span>

namespace ws2812 {

// Must be packed b/c we're directly reading it via BLE GATTs.
struct [[gnu::packed]] UpdateSpanHeader {
    uint8_t components = 3;  // usually 3 (GRB) or 4 (GRBW)
    uint8_t offset = 0;
    uint8_t length = 0;

    [[nodiscard]] constexpr uint16_t data_length() const {
        return components * length;
    }
};

// returns false if the update couldn't be applied for whatever reason
bool update_span(UpdateSpanHeader const&, std::span<uint8_t const> pixel_data);

}  // namespace ws2812
