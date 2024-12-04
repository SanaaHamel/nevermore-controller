#include "servo.hpp"
#include "config.hpp"
#include "handler_helpers.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "sdk/pwm.hpp"
#include "settings.hpp"
#include "utility/servo.hpp"

using namespace std;

namespace nevermore::gatt::servo {

namespace {

constexpr uint32_t SERVO_PWM_HZ = 50;

BLE::Percentage16_10 g_servo_vent_power;  // not-known -> not engaged

void servo_vent_set(BLE::Percentage16_10 power) {
    g_servo_vent_power = power;
    servo_set(Pins::active().vent_servo_pwm, settings::g_active.servo_vent, power);
}

}  // namespace

bool init() {
    // setup PWM configurations for fan PWM and fan tachometer
    if (auto pin = Pins::active().vent_servo_pwm) {
        auto cfg = pwm_get_default_config();
        pwm_config_set_freq_hz(cfg, SERVO_PWM_HZ);
        pwm_init(pwm_gpio_to_slice_num_(pin), &cfg, true);
    }

    servo_vent_set(BLE::NOT_KNOWN);

    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(hci_con_handle_t conn, uint16_t attr, span<uint8_t> buffer) {
    switch (attr) {
        USER_DESCRIBE(SERVO_VENT_RANGE, "Vent servo start/end range.")
        USER_DESCRIBE(SERVO_VENT_POWER, "Vent servo %")

        READ_VALUE(SERVO_VENT_RANGE, settings::g_active.servo_vent)
        READ_VALUE(SERVO_VENT_POWER, g_servo_vent_power)

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t attr, span<uint8_t const> buffer) {
    WriteConsumer consume{buffer};

    switch (attr) {
    case HANDLE_ATTR(SERVO_VENT_RANGE, VALUE): {
        ServoRange value = consume;
        if (!value.validate()) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        settings::g_active.servo_vent = value;
        servo_vent_set(g_servo_vent_power);
        return 0;
    }

    case HANDLE_ATTR(SERVO_VENT_POWER, VALUE): {
        servo_vent_set(BLE::Percentage16_10(consume));
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::servo
