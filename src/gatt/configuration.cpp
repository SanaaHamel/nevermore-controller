#include "configuration.hpp"
#include "FreeRTOS.h"
#include "handler_helpers.hpp"
#include "picowota/reboot.h"
#include "portmacro.h"
#include "sdk/task.hpp"
#include "sensors.hpp"
#include "settings.hpp"
#include "utility/task.hpp"
#include <cstdio>

using namespace std;

namespace nevermore::gatt::configuration {

namespace {

// delay must be long enough let transports finish responding
constexpr auto REBOOT_DELAY = 200ms;

// Use a task instead of a timer in case a timer misbehaves and blocks.
Task g_reboot_task;

void reboot_delayed(bool to_bootloader) {
    xTaskNotify(g_reboot_task, to_bootloader, eSetValueWithoutOverwrite);
}

constexpr BLE::ValidRange VOC_GATING_THRESHOLD_RANGE{.min = settings::VOC_GATING_THRESHOLD_MIN,
        .max = sensors::VOCIndex(500)};

char const* g_pins_config_error = "";

}  // namespace

bool init() {
    g_reboot_task =
            mk_task("reboot-task", Priority(configTIMER_TASK_PRIORITY), configMINIMAL_STACK_SIZE)([]() {
                UBaseType_t to_bootloader;
                xTaskNotifyWait(0, 0, &to_bootloader, portMAX_DELAY);
                printf("!! reboot requested; to-bootloader=%d\n", int(to_bootloader));

                task_delay<REBOOT_DELAY>();

                // save settings before we reboot
                settings::save(settings::g_active);
                // wait for stdio to flush
                task_delay<10ms>();

                picowota_reboot(to_bootloader != 0);
            });
    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(
        hci_con_handle_t const conn, uint16_t const attr, uint16_t const offset, span<uint8_t> const buffer) {
    switch (attr) {
        USER_DESCRIBE(CONFIG_REBOOT, "Reboot")
        USER_DESCRIBE(CONFIG_FLAGS, "Configuration Flags (bitset)")
        USER_DESCRIBE(CONFIG_CHECKPOINT_SENSOR_CALIBRATION, "Force sensor calibration checkpoint")
        USER_DESCRIBE(CONFIG_RESET_SENSOR_CALIBRATION, "Reset sensor calibration")
        USER_DESCRIBE(CONFIG_RESET_SETTINGS, "Reset settings (bitset)")
        USER_DESCRIBE(CONFIG_VOC_GATING_THRESHOLD, "VOC Gating Threshold")
        USER_DESCRIBE(CONFIG_VOC_GATING_THRESHOLD_OVERRIDE, "VOC Gating Threshold Override")
        USER_DESCRIBE(CONFIG_VOC_CALIBRATE_ENABLED, "VOC Calibration Enabled")
        USER_DESCRIBE(CONFIG_PINS, "Pin Assignments")
        USER_DESCRIBE(CONFIG_PINS_DEFAULT, "Pin Assignment Defaults")
        USER_DESCRIBE(CONFIG_PINS_ERROR, "Pin Assignments Validation Message")

        // NOLINTNEXTLINE(bugprone-branch-clone)
        HANDLE_READ_BLOB(CONFIG_VOC_GATING_THRESHOLD, VALID_RANGE, VOC_GATING_THRESHOLD_RANGE)
        HANDLE_READ_BLOB(CONFIG_VOC_GATING_THRESHOLD_OVERRIDE, VALID_RANGE, VOC_GATING_THRESHOLD_RANGE)

        READ_VALUE(CONFIG_FLAGS, settings::g_active.flags.bitset)

        READ_VALUE(CONFIG_VOC_GATING_THRESHOLD, settings::g_active.voc_gating_threshold)
        READ_VALUE(CONFIG_VOC_GATING_THRESHOLD_OVERRIDE, settings::g_active.voc_gating_threshold_override)
        READ_VALUE(CONFIG_VOC_CALIBRATE_ENABLED, settings::g_active.voc_calibration_enabled)
        READ_VALUE(CONFIG_PINS, settings::g_active.pins)
        READ_VALUE(CONFIG_PINS_DEFAULT, PINS_DEFAULT)

    case HANDLE_ATTR(CONFIG_PINS_ERROR, VALUE):
        return att_read_callback_handle_blob(reinterpret_cast<uint8_t const*>(g_pins_config_error),
                strlen(g_pins_config_error), offset, buffer);

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t attr, span<uint8_t const> buffer) {
    WriteConsumer consume{buffer};

    switch (attr) {
    case HANDLE_ATTR(CONFIG_REBOOT, VALUE): {
        switch (uint8_t(consume)) {
        case 0: reboot_delayed(false); return 0;
        case 1: reboot_delayed(true); return 0;
        default: return ATT_ERROR_VALUE_NOT_ALLOWED;
        }
    }
    case HANDLE_ATTR(CONFIG_FLAGS, VALUE): {
        uint64_t const flags = consume;
        uint64_t const mask = consume.or_default(std::numeric_limits<uint64_t>::max());
        uint64_t const curr = settings::g_active.flags.bitset;
        settings::g_active.flags.bitset = (flags & mask) | (curr & ~mask);
        return 0;
    }
    case HANDLE_ATTR(CONFIG_RESET_SENSOR_CALIBRATION, VALUE): {
        // only accept zero data write for now.
        // maybe add an API for resetting specific sensors later.
        if (consume.remaining()) return ATT_ERROR_VALUE_NOT_ALLOWED;

        sensors::calibrations_reset();
        return 0;
    }
    case HANDLE_ATTR(CONFIG_CHECKPOINT_SENSOR_CALIBRATION, VALUE): {
        // only accept zero data write for now.
        // maybe add an API for resetting specific sensors later.
        if (consume.remaining()) return ATT_ERROR_VALUE_NOT_ALLOWED;

        sensors::calibrations_force_checkpoint();
        return 0;
    }
    case HANDLE_ATTR(CONFIG_RESET_SETTINGS, VALUE): {
        settings::ResetFlags const flags = consume;
        if (consume.remaining()) return ATT_ERROR_VALUE_NOT_ALLOWED;

        settings::g_active.reset(flags);
        return 0;
    }
    case HANDLE_ATTR(CONFIG_VOC_GATING_THRESHOLD, VALUE): {
        sensors::VOCIndex threshold = consume;
        if (!VOC_GATING_THRESHOLD_RANGE.in_range(threshold)) return ATT_ERROR_VALUE_NOT_ALLOWED;

        settings::g_active.voc_gating_threshold = threshold;
        return 0;
    }
    case HANDLE_ATTR(CONFIG_VOC_GATING_THRESHOLD_OVERRIDE, VALUE): {
        sensors::VOCIndex threshold = consume;
        if (!VOC_GATING_THRESHOLD_RANGE.in_range_or_not_known(threshold)) return ATT_ERROR_VALUE_NOT_ALLOWED;

        settings::g_active.voc_gating_threshold_override = threshold;
        return 0;
    }
    case HANDLE_ATTR(CONFIG_VOC_CALIBRATE_ENABLED, VALUE): {
        uint8_t value = consume;
        if (value != 0 && value != 1) return ATT_ERROR_VALUE_NOT_ALLOWED;

        settings::g_active.voc_calibration_enabled = !!value;
        return 0;
    }

    case HANDLE_ATTR(CONFIG_PINS, VALUE): {
        g_pins_config_error = "Incorrect `Pins` struct size.";
        Pins value = consume.exactly<Pins>();
        try {
            value.validate_or_throw();
        } catch (char const* msg) {
            g_pins_config_error = msg;
            return ATT_ERROR_VALUE_NOT_ALLOWED;
        }

        g_pins_config_error = "";
        settings::g_active.pins = value;
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::configuration
