#include "configuration.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/btstack.hpp"
#include "sensors.hpp"
#include <array>
#include <cstdint>

using namespace std;

#define WS2812_UPDATE_SPAN_UUID 5d91b6ce_7db1_4e06_b8cb_d75e7dd49aae

#define CONFIG_FLAGS_01 d4b66bf4_3d8f_4746_b6a2_8a59d2eac3ce_01

namespace nevermore::gatt::configuration {

namespace {

constexpr array FLAGS{
        &sensors::g_config.fallback,
        &sensors::g_config.fallback_exhaust_mcu,
};

}  // namespace

bool init(async_context_t&) {
    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    switch (att_handle) {
        USER_DESCRIBE(CONFIG_FLAGS_01, "Configuration Flags (bitset)")

        READ_VALUE(CONFIG_FLAGS_01, ([]() -> uint16_t {
            uint64_t flags = 0;
            for (size_t i = 0; i < FLAGS.size(); ++i)
                flags |= uint64_t(*FLAGS.at(i)) << i;
            return flags;
        })())

    default: return {};
    }
}

optional<int> attr_write(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t const* buffer, uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;
    WriteConsumer consume{offset, buffer, buffer_size};

    switch (att_handle) {
    case HANDLE_ATTR(CONFIG_FLAGS_01, VALUE): {
        uint64_t const flags = consume;
        for (size_t i = 0; i < FLAGS.size(); ++i)
            *FLAGS.at(i) = !!(flags & uint64_t(1) << i);
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::configuration
