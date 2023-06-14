#include "neopixel.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/btstack.hpp"
#include "ws2812.hpp"
#include <cstdint>
#include <span>
#include <utility>

#define WS2812_UPDATE_SPAN_UUID 5d91b6ce_7db1_4e06_b8cb_d75e7dd49aae
#define WS2812_UPDATE_SPAN_01 5d91b6ce_7db1_4e06_b8cb_d75e7dd49aae_01

namespace {

// Must be packed b/c we're directly reading it via BLE GATTs.
struct [[gnu::packed]] UpdateSpanHeader {
    uint8_t offset;
    uint8_t length;
};

}  // namespace

std::optional<uint16_t> NeoPixelService::attr_read(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    auto readBlob = [&](auto&& datum) -> uint16_t {
        return att_read_callback_handle_blob(
                std::forward<decltype(datum)>(datum), offset, buffer, buffer_size);
    };

    switch (att_handle) {
        USER_DESCRIBE(WS2812_UPDATE_SPAN_01, "Updates a span of the WS2812 (aka NeoPixel) chain.")

        default: return {};
    }
}

std::optional<int> NeoPixelService::attr_write(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t const* buffer, uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;
    WriteConsumer consume{offset, buffer, buffer_size};

    switch (att_handle) {
        case HANDLE_ATTR(WS2812_UPDATE_SPAN_01, VALUE): {
            UpdateSpanHeader const* header = consume;
            if (!header) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
            if (header->length == 0) return 0;  // report trivial success

            auto const payload = consume.span<uint8_t>(header->length);
            // empty && 0 < length -> bad payload
            if (payload.empty()) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;

            ws2812_update(header->offset, payload);
            return 0;
        }

        default: return {};
    }
}
