#include "fan.hpp"
#include "characteristic_ids.hpp"
#include "config.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "sdk/pwm.hpp"
#include "sensors.hpp"
#include "sensors/tachometer.hpp"
#include "settings.hpp"
#include "utility/fan_policy.hpp"
#include "utility/fan_policy_thermal.hpp"
#include "utility/timer.hpp"
#include <cstdint>
#include <limits>

using namespace std;

namespace nevermore::gatt::fan {

namespace {

BLE_DECL_SCALAR(RPM16, uint16_t, 1, 0, 0);

constexpr uint8_t FAN_POLICY_UPDATE_RATE_HZ = 10;

constexpr uint8_t TACHOMETER_PULSE_PER_REVOLUTION = 2;
constexpr uint32_t FAN_PWM_HZ = 25'000;

BLE::Percentage8 g_fan_power = 0;
BLE::Percentage8 g_fan_power_override;  // not-known -> automatic control
sensors::Tachometer g_tachometer;

struct [[gnu::packed]] FanPowerTachoAggregate {
    BLE::Percentage8 power = g_fan_power;
    RPM16 tachometer = fan_rpm();
};

struct [[gnu::packed]] Aggregate {
    BLE::Percentage8 power = g_fan_power;
    BLE::Percentage8 power_override = g_fan_power_override;
    BLE::Percentage8 power_passive = settings::g_active.fan_power_passive;
    BLE::Percentage8 power_automatic = settings::g_active.fan_power_automatic;
    BLE::Percentage8 power_coefficient = settings::g_active.fan_power_coefficient;
    RPM16 tachometer = FanPowerTachoAggregate{}.tachometer;
};

auto g_notify_fan_power_tacho_aggregate = NotifyState<[](hci_con_handle_t conn) {
    att_server_notify(conn, HANDLE_ATTR(FAN_POWER_TACHO_AGGREGATE, VALUE), FanPowerTachoAggregate{});
}>();

// false positive:  indirect dependency on global `settings::g_active` only
//                  happens after global initialisation completes.
// NOLINTNEXTLINE(cppcoreguidelines-interfaces-global-init)
auto g_notify_aggregate = NotifyState<[](hci_con_handle_t conn) {
    att_server_notify(conn, HANDLE_ATTR(FAN_AGGREGATE, VALUE), Aggregate{});
}>();

void fan_power_set(BLE::Percentage8 power, sensors::Sensors const& sensors = sensors::g_sensors,
        settings::Settings const& settings = settings::g_active) {
    auto temperature = max(sensors.temperature_intake, sensors.temperature_exhaust);
    auto thermal_scaler = settings.fan_policy_thermal(temperature);

    power = power.value_or(0) * thermal_scaler;

    if (g_fan_power != power) {
        g_fan_power = power;
        g_notify_fan_power_tacho_aggregate.notify();  // `g_fan_power` changed
        g_notify_aggregate.notify();                  // `g_fan_power` changed
    }

    auto scale = (power.value_or(0) / 100.) * (settings.fan_power_coefficient.value_or(0) / 100.);
    auto duty = uint16_t(numeric_limits<uint16_t>::max() * scale);
    for (auto&& pin : Pins::active().fan_pwm)
        if (pin) pwm_set_gpio_duty(pin, duty);
}

}  // namespace

float fan_rpm() {
    return g_tachometer.revolutions_per_second() * 60;
}

float fan_power() {
    // NOLINTNEXTLINE(bugprone-narrowing-conversions, cppcoreguidelines-narrowing-conversions)
    return g_fan_power.value_or(0);
}

void fan_power_override(BLE::Percentage8 power) {
    if (g_fan_power_override == power) return;

    g_fan_power_override = power;
    g_notify_aggregate.notify();

    if (power != BLE::NOT_KNOWN) {
        fan_power_set(power);  // apply override
    }
}

BLE::Percentage8 fan_power_override() {
    return g_fan_power_override;
}

bool init() {
    // setup PWM configurations for fan PWM and fan tachometer
    for (auto&& pin : Pins::active().fan_pwm) {
        if (!pin) continue;

        auto cfg = pwm_get_default_config();
        pwm_config_set_freq_hz(cfg, FAN_PWM_HZ);
        pwm_init(pwm_gpio_to_slice_num_(pin), &cfg, true);
    }

    g_tachometer.setup(Pins::active().fan_tachometer, TACHOMETER_PULSE_PER_REVOLUTION);
    g_tachometer.start();

    // set fan PWM level
    fan_power_set(g_fan_power);

    // HACK:  We'd like to notify on write to tachometer changes, but the code base isn't setup
    //        for that yet. Internally poll and update based on diffs for now.
    mk_timer("gatt-fan-tachometer-notify", SENSOR_UPDATE_PERIOD)([](auto*) {
        static decltype(fan_rpm()) g_prev;
        if (g_prev == fan_rpm()) return;

        g_prev = fan_rpm();
        g_notify_fan_power_tacho_aggregate.notify();
        g_notify_aggregate.notify();
    });

    mk_timer("fan-policy", 1.s / FAN_POLICY_UPDATE_RATE_HZ)([](auto*) {
        static auto g_instance = settings::g_active.fan_policy_env.instance();
        // keep updating even w/ `g_fan_power_override` set b/c we need to
        // refresh to account for thermal throttling policy
        if (g_fan_power_override == BLE::NOT_KNOWN) {
            if (auto perc = g_instance(sensors::g_sensors); 0 < perc)
                fan_power_set(perc * settings::g_active.fan_power_automatic.value_or(0));
            else
                fan_power_set(settings::g_active.fan_power_passive.value_or(0));
        } else {
            fan_power_set(g_fan_power_override);
        }
    });

    return true;
}

void disconnected(hci_con_handle_t conn) {
    g_notify_fan_power_tacho_aggregate.unregister(conn);
    g_notify_aggregate.unregister(conn);
}

optional<uint16_t> attr_read(hci_con_handle_t conn, uint16_t attr, span<uint8_t> buffer) {
    switch (attr) {
        USER_DESCRIBE(FAN_POWER, "Fan %")
        USER_DESCRIBE(FAN_POWER_OVERRIDE, "Fan % - Override")
        USER_DESCRIBE(FAN_POWER_PASSIVE, "Fan % - Passive")
        USER_DESCRIBE(FAN_POWER_AUTOMATIC, "Fan % - Automatic")
        USER_DESCRIBE(FAN_POWER_COEFFICIENT, "Fan % - Limiting Coefficient")
        USER_DESCRIBE(FAN_TACHOMETER, "Fan RPM")
        USER_DESCRIBE(FAN_POWER_TACHO_AGGREGATE, "Aggregated Fan % and RPM")
        USER_DESCRIBE(FAN_AGGREGATE, "Aggregated Service Data")

        USER_DESCRIBE(FAN_POLICY_COOLDOWN, "How long to continue filtering after conditions are acceptable")
        USER_DESCRIBE(FAN_POLICY_VOC_PASSIVE_MAX, "Filter if any VOC sensor reaches this threshold")
        USER_DESCRIBE(FAN_POLICY_VOC_IMPROVE_MIN, "Filter if intake exceeds exhaust by this threshold")
        USER_DESCRIBE(FAN_POWER_THERMAL_LIMIT, "Thermal limiting cut-off")

        READ_VALUE(FAN_POWER, g_fan_power)
        READ_VALUE(FAN_POWER_OVERRIDE, g_fan_power_override)
        READ_VALUE(FAN_POWER_PASSIVE, settings::g_active.fan_power_passive)
        READ_VALUE(FAN_POWER_AUTOMATIC, settings::g_active.fan_power_automatic)
        READ_VALUE(FAN_POWER_COEFFICIENT, settings::g_active.fan_power_coefficient)
        READ_VALUE(FAN_TACHOMETER, FanPowerTachoAggregate{}.tachometer)
        READ_VALUE(FAN_POWER_TACHO_AGGREGATE, FanPowerTachoAggregate{})
        READ_VALUE(FAN_AGGREGATE, Aggregate{});  // default init populate from global state

        READ_VALUE(FAN_POLICY_COOLDOWN, settings::g_active.fan_policy_env.cooldown)
        READ_VALUE(FAN_POLICY_VOC_PASSIVE_MAX, settings::g_active.fan_policy_env.voc_passive_max)
        READ_VALUE(FAN_POLICY_VOC_IMPROVE_MIN, settings::g_active.fan_policy_env.voc_improve_min)
        READ_VALUE(FAN_POWER_THERMAL_LIMIT, settings::g_active.fan_policy_thermal)

        READ_CLIENT_CFG(FAN_POWER_TACHO_AGGREGATE, g_notify_fan_power_tacho_aggregate)
        READ_CLIENT_CFG(FAN_AGGREGATE, g_notify_aggregate)

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t attr, span<uint8_t const> buffer) {
    WriteConsumer consume{buffer};

    switch (attr) {
        WRITE_VALUE(FAN_POLICY_COOLDOWN, settings::g_active.fan_policy_env.cooldown)
        WRITE_VALUE(FAN_POLICY_VOC_PASSIVE_MAX, settings::g_active.fan_policy_env.voc_passive_max)
        WRITE_VALUE(FAN_POLICY_VOC_IMPROVE_MIN, settings::g_active.fan_policy_env.voc_improve_min)

        WRITE_CLIENT_CFG(FAN_POWER_TACHO_AGGREGATE, g_notify_fan_power_tacho_aggregate)
        WRITE_CLIENT_CFG(FAN_AGGREGATE, g_notify_aggregate)

    case HANDLE_ATTR(FAN_POWER_OVERRIDE, VALUE): {
        fan_power_override((BLE::Percentage8)consume);
        return 0;
    }

    case HANDLE_ATTR(FAN_POWER_PASSIVE, VALUE): {
        BLE::Percentage8 value = consume;
        if (value == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        settings::g_active.fan_power_passive = value;
        return 0;
    }

    case HANDLE_ATTR(FAN_POWER_AUTOMATIC, VALUE): {
        BLE::Percentage8 value = consume;
        if (value == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        settings::g_active.fan_power_automatic = value;
        return 0;
    }

    case HANDLE_ATTR(FAN_POWER_COEFFICIENT, VALUE): {
        BLE::Percentage8 value = consume;
        if (value == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        settings::g_active.fan_power_coefficient = value;
        return 0;
    }

    case HANDLE_ATTR(FAN_POWER_THERMAL_LIMIT, VALUE): {
        FanPolicyThermal value = consume;
        value = value.or_(settings::g_active.fan_policy_thermal);
        if (!value.validate()) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        settings::g_active.fan_policy_thermal = value;
        fan_power_set(g_fan_power);  // apply updated thermal coefficient
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::fan
