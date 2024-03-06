#include "photocatalytic.hpp"
#include "config.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/pwm.hpp"
#include <cstdint>
#include <limits>

using namespace std;

#define PC_POWER 2B04_07
#define PC_POWER_OVERRIDE 2B04_08

namespace nevermore::gatt::photocatalytic {

namespace {

constexpr uint32_t PWM_HZ = 10'000;

BLE::Percentage8 g_power_override;  // not-known -> automatic control

void pc_power_set(BLE::Percentage8 const& power) {
    if constexpr (PIN_PHOTOCATALYTIC_PWM) {
        auto scale = power.value_or(0) / 100.;
        auto duty = uint16_t(numeric_limits<uint16_t>::max() * scale);
        pwm_set_gpio_duty(*PIN_PHOTOCATALYTIC_PWM, duty);
    }
}

void pc_power_override(BLE::Percentage8 override) {
    g_power_override = override;
    pc_power_set(override);
}

}  // namespace

bool init() {
    // setup PWM configurations for fan PWM and fan tachometer
    if constexpr (PIN_PHOTOCATALYTIC_PWM) {
        auto cfg_pwm = pwm_get_default_config();
        pwm_config_set_freq_hz(cfg_pwm, PWM_HZ);
        pwm_init(pwm_gpio_to_slice_num_(*PIN_PHOTOCATALYTIC_PWM), &cfg_pwm, true);
    }

    // set fan PWM level
    pc_power_override(g_power_override);

    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(
        hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    switch (att_handle) {
        USER_DESCRIBE(PC_POWER, "Photocatalytic %")
        USER_DESCRIBE(PC_POWER_OVERRIDE, "Photocatalytic % - Override")

        READ_VALUE(PC_POWER, g_power_override.or_(0))
        READ_VALUE(PC_POWER_OVERRIDE, g_power_override)

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t const* buffer,
        uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;
    WriteConsumer consume{offset, buffer, buffer_size};

    switch (att_handle) {
    case HANDLE_ATTR(PC_POWER_OVERRIDE, VALUE): {
        pc_power_override((BLE::Percentage8)consume);
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::photocatalytic
