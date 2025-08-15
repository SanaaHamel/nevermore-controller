#include "cooler.hpp"
#include "config/pins.hpp"
#include "handler_helpers.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/pwm.hpp"
#include <limits>

using namespace std;

namespace nevermore::gatt::cooler {

namespace {

constexpr uint32_t PWM_HZ = 10'000;

BLE::Percentage8 g_power_override;  // not-known -> automatic control

void power_set(BLE::Percentage8 const& power) {
    auto scale = power.value_or(0) / 100.;
    auto duty = uint16_t(numeric_limits<uint16_t>::max() * scale);
    if (auto pin = Pins::active().cooler_pwm) {
        pwm_set_gpio_duty(pin, duty);
    }
}

void power_override(BLE::Percentage8 override) {
    g_power_override = override;
    power_set(override);
}

}  // namespace

bool init() {
    // setup PWM configurations for fan PWM and fan tachometer
    if (auto pin = Pins::active().cooler_pwm) {
        auto cfg = pwm_get_default_config();
        pwm_config_set_freq_hz(cfg, PWM_HZ);
        pwm_init(pwm_gpio_to_slice_num_(pin), &cfg, true);
    }

    // set fan PWM level
    power_override(g_power_override);

    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(
        hci_con_handle_t const conn, uint16_t const attr, uint16_t const offset, span<uint8_t> const buffer) {
    switch (attr) {
        USER_DESCRIBE(COOLER_POWER, "Cooler %")
        USER_DESCRIBE(COOLER_POWER_OVERRIDE, "Cooler % - Override")

        READ_VALUE(COOLER_POWER, g_power_override.or_(0))
        READ_VALUE(COOLER_POWER_OVERRIDE, g_power_override)

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t attr, span<uint8_t const> buffer) {
    WriteConsumer consume{buffer};

    switch (attr) {
    case HANDLE_ATTR(COOLER_POWER_OVERRIDE, VALUE): {
        power_override((BLE::Percentage8)consume);
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::cooler
