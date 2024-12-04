#include "ws2812.hpp"
#include "../ws2812.hpp"
#include "handler_helpers.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"

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

namespace nevermore::gatt::ws2812 {

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

    static deque<chrono::steady_clock::time_point> g_timestamps;
    static chrono::steady_clock::time_point g_log_next;

    if (g_timestamps.size() == MAX_ENTRIES) {
        g_timestamps.pop_front();
    }
    g_timestamps.push_back(chrono::steady_clock::now());

    if (g_timestamps.size() < 2) return;           // not enough data
    if (g_timestamps.back() < g_log_next) return;  // too soon
    g_log_next = g_timestamps.back() + LOG_DELAY;

    chrono::duration<double, milli> dur = g_timestamps.back() - g_timestamps.front();
    auto mean = dur / (g_timestamps.size() - 1);
    printf("DBG - NeoPixel Update Rate - Mean FPS: %f\n", 1s / mean);
#endif
}

}  // namespace

bool init() {
    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(hci_con_handle_t conn, uint16_t attr, span<uint8_t> buffer) {
    switch (attr) {
        USER_DESCRIBE(WS2812_TOTAL_COMPONENTS, "Total # of components (i.e. octets) in the WS2812 chain.")
        USER_DESCRIBE(WS2812_UPDATE_SPAN, "Update a span of the WS2812 chain.")

        READ_VALUE(WS2812_TOTAL_COMPONENTS, ([]() -> uint16_t {
            // -1 because 0xFFFF is reserved as not-known for a BLE::Count16
            return min<size_t>(nevermore::ws2812::components_total(), UINT16_MAX - 1);
        })())

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t attr, span<uint8_t const> buffer) {
    WriteConsumer consume{buffer};

    switch (attr) {
    case HANDLE_ATTR(WS2812_TOTAL_COMPONENTS, VALUE): {
        BLE::Count16 count = consume;
        if (count == BLE::NOT_KNOWN) return ATT_ERROR_VALUE_NOT_ALLOWED;
        if (!nevermore::ws2812::setup(size_t(double(count)))) return ATT_ERROR_VALUE_NOT_ALLOWED;

        return 0;
    }

    case HANDLE_ATTR(WS2812_UPDATE_SPAN, VALUE): {
        DBG_update_rate_log();

        UpdateSpanHeader header = consume;
        // be extra picky, reject any pending extra data
        // FUTURE WORK: change attr to just assume any tailing data is the span?
        if (header.length != consume.remaining()) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
        if (!nevermore::ws2812::update(header.offset, consume.span(header.length)))
            return ATT_ERROR_VALUE_NOT_ALLOWED;

        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::ws2812
