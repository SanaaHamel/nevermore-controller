#include "configuration.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "picowota/reboot.h"
#include "sensors.hpp"
#include "settings.hpp"
#include "utility/timer.hpp"
#include <array>
#include <cstdint>
#include <cstdio>

using namespace std;

#define CONFIG_REBOOT_01 f48a18bb_e03c_4583_8006_5b54422e2045_01
#define CONFIG_FLAGS_01 d4b66bf4_3d8f_4746_b6a2_8a59d2eac3ce_01
#define CONFIG_RESET_SENSOR_CALIBRATION 75bf055c_02be_466f_8c7d_6ebc72078048_01

namespace nevermore::gatt::configuration {

namespace {

constexpr auto REBOOT_DELAY = 200ms;

constexpr array FLAGS{
        &sensors::g_config.fallback,
        &sensors::g_config.fallback_exhaust_mcu,
};

void reboot_delayed(bool to_bootloader) {
    // if they want to race these, who cares, we're rebooting anyways
    printf("!! GATT - Reboot Requested; OTA=%d\n", int(to_bootloader));
    // flush/save settings before we reboot
    settings::save(settings::g_active);

    auto go = mk_timer("gatt-configuration-reboot", REBOOT_DELAY);
    if (to_bootloader)
        go([](auto* p) { picowota_reboot(true); });
    else
        go([](auto* p) { picowota_reboot(false); });
}

}  // namespace

bool init() {
    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    switch (att_handle) {
        USER_DESCRIBE(CONFIG_REBOOT_01, "Reboot")
        USER_DESCRIBE(CONFIG_FLAGS_01, "Configuration Flags (bitset)")
        USER_DESCRIBE(CONFIG_RESET_SENSOR_CALIBRATION, "Reset sensor calibration")

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
    case HANDLE_ATTR(CONFIG_REBOOT_01, VALUE): {
        switch (uint8_t(consume)) {
        case 0: reboot_delayed(false); return 0;
        case 1: reboot_delayed(true); return 0;
        default: return ATT_ERROR_VALUE_NOT_ALLOWED;
        }
    }
    case HANDLE_ATTR(CONFIG_FLAGS_01, VALUE): {
        uint64_t const flags = consume;
        uint64_t const mask = consume.or_default(std::numeric_limits<uint64_t>::max());
        for (size_t i = 0; i < FLAGS.size(); ++i)
            if (auto const bit = uint64_t(1) << i; mask & bit) {
                *FLAGS.at(i) = !!(flags & bit);
            }
        return 0;
    }
    case HANDLE_ATTR(CONFIG_RESET_SENSOR_CALIBRATION, VALUE): {
        // only accept zero data write for now.
        // maybe add an API for resetting specific sensors later.
        if (consume.remaining()) return ATT_ERROR_VALUE_NOT_ALLOWED;

        sensors::reset_calibrations();
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::configuration
