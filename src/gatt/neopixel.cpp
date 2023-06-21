#include "neopixel.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "ws2812.hpp"
#include <cstdint>
#include <span>
#include <utility>

using namespace std;

#define DEBUG_NEOPIXEL_UPDATE_RATE_LOG 0
#if DEBUG_NEOPIXEL_UPDATE_RATE_LOG
#include <chrono>
#include <cstdio>
#include <deque>
#include <limits>
#include <ratio>

using namespace std::literals::chrono_literals;
#endif

#define WS2812_UPDATE_SPAN_UUID 5d91b6ce_7db1_4e06_b8cb_d75e7dd49aae

#define WS2812_UPDATE_SPAN_01 5d91b6ce_7db1_4e06_b8cb_d75e7dd49aae_01
#define WS2812_TOTAL_COMPONENTS_01 2AEA_01

namespace {

struct [[gnu::packed]] UpdateSpanHeader {
    uint8_t offset;
    uint8_t length;
};

void DBG_update_rate_log() {
#if DEBUG_NEOPIXEL_UPDATE_RATE_LOG
    constexpr auto LOG_DELAY = 1s;
    constexpr size_t MAX_ENTRIES = 100;
    static_assert(0 < MAX_ENTRIES);  // sancheck. logic below depends on this.

    static deque<chrono::system_clock::time_point> g_timestamps;
    static chrono::system_clock::time_point g_log_next;

    if (g_timestamps.size() == MAX_ENTRIES) {
        g_timestamps.pop_front();
    }
    g_timestamps.push_back(chrono::system_clock::now());

    if (g_timestamps.size() < 2) return;           // not enough data
    if (g_timestamps.back() < g_log_next) return;  // too soon
    g_log_next = g_timestamps.back() + LOG_DELAY;

    chrono::duration<double, milli> dur = g_timestamps.back() - g_timestamps.front();
    auto mean = dur / (g_timestamps.size() - 1);
    printf("DBG - NeoPixel Update Rate - Mean FPS: %f\n", 1s / mean);
#endif
}

}  // namespace

void NeoPixelService::disconnected(hci_con_handle_t) {}

optional<uint16_t> NeoPixelService::attr_read(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    auto readBlob = [&](auto&& datum) -> uint16_t {
        return att_read_callback_handle_blob(
                std::forward<decltype(datum)>(datum), offset, buffer, buffer_size);
    };

    switch (att_handle) {
        USER_DESCRIBE(WS2812_TOTAL_COMPONENTS_01, "Total # of components (i.e. octets) in the WS2812 chain.")
        USER_DESCRIBE(WS2812_UPDATE_SPAN_01, "Update a span of the WS2812 chain.")

        READ_VALUE(WS2812_TOTAL_COMPONENTS_01, ([]() -> uint16_t {
            // -1 because 0xFFFF is reserved as not-known for a BLE::Count16
            return min<size_t>(ws2812_components_total(), UINT16_MAX - 1);
        })())

        default: return {};
    }
}

optional<int> NeoPixelService::attr_write(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t const* buffer, uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;
    WriteConsumer consume{offset, buffer, buffer_size};

    switch (att_handle) {
        case HANDLE_ATTR(WS2812_TOTAL_COMPONENTS_01, VALUE): {
            BLE::Count16 count = consume;
            if (count == BLE::NOT_KNOWN) return ATT_ERROR_VALUE_NOT_ALLOWED;
            if (!ws2812_setup(size_t(double(count)))) return ATT_ERROR_VALUE_NOT_ALLOWED;

            return 0;
        }

        case HANDLE_ATTR(WS2812_UPDATE_SPAN_01, VALUE): {
            DBG_update_rate_log();

            UpdateSpanHeader header = consume;
            // be extra picky, reject any pending extra data
            // FUTURE WORK: change attr to just assume any tailing data is the span?
            if (header.length != consume.remaining()) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
            if (!ws2812_update(header.offset, consume.span(header.length)))
                return ATT_ERROR_VALUE_NOT_ALLOWED;

            return 0;
        }

        default: return {};
    }
}
