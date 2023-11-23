#include "fan.hpp"
#include "config.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "sdk/pwm.hpp"
#include "sensors.hpp"
#include "sensors/tachometer.hpp"
#include "utility/fan_policy.hpp"
#include "utility/timer.hpp"
#include <cstdint>
#include <limits>

using namespace std;

#define FAN_POWER 2B04_01
#define FAN_POWER_OVERRIDE 2B04_02
#define FAN_POWER_AUTOMATIC 2B04_03
#define FAN_POWER_COEFFICIENT 2B04_04
#define FAN_POWER_THERMAL_LIMIT 45d2e7d7_40c4_46a6_a160_43eb02d01e27_01
#define TACHOMETER 03f61fe0_9fe7_4516_98e6_056de551687f_01
#define FAN_POWER_TACHO_AGGREGATE 79cd747f_91af_49a6_95b2_5b597c683129_01
// NB: Error prone, but we're the 2nd aggregation char instance in the DB
#define FAN_AGGREGATE 75134bec_dd06_49b1_bac2_c15e05fd7199_02

#define FAN_POLICY_COOLDOWN 2B16_01
#define FAN_POLICY_VOC_PASSIVE_MAX 216aa791_97d0_46ac_8752_60bbc00611e1_03
#define FAN_POLICY_VOC_IMPROVE_MIN 216aa791_97d0_46ac_8752_60bbc00611e1_04

namespace nevermore::gatt::fan {

namespace {

BLE_DECL_SCALAR(RPM16, uint16_t, 1, 0, 0);

constexpr uint8_t FAN_POLICY_UPDATE_RATE_HZ = 10;

constexpr uint8_t TACHOMETER_PULSE_PER_REVOLUTION = 2;
constexpr uint32_t FAN_PWN_HZ = 25'000;

constexpr auto SLICE_PWM = pwm_gpio_to_slice_num_(PIN_FAN_PWM);
constexpr auto SLICE_TACHOMETER = pwm_gpio_to_slice_num_(PIN_FAN_TACHOMETER);
static_assert(pwm_gpio_to_channel_(PIN_FAN_TACHOMETER) == PWM_CHAN_B, "can only read from B channel");

// not included in the fan aggregation - technically a separate service
FanPolicyEnvironmental g_fan_policy;

BLE::Percentage8 g_fan_power = 0;
BLE::Percentage8 g_fan_power_override;           // not-known -> automatic control
BLE::Percentage8 g_fan_power_automatic = 100;    // not-known -> disallowed
BLE::Percentage8 g_fan_power_coefficient = 100;  // not-known -> disallowed
nevermore::sensors::Tachometer g_tachometer{PIN_FAN_TACHOMETER, TACHOMETER_PULSE_PER_REVOLUTION};

struct [[gnu::packed]] ThermalLimit {
    BLE::Temperature min = 50;             // not-known -> disallowed
    BLE::Temperature max = 60;             // not-known -> disallowed
    BLE::Percentage16_10 coefficient = 0;  // not-known -> disabled (equiv to 100%)

    //  [0, 1] or None
    [[nodiscard]] std::optional<double> percent(BLE::Temperature current) const {
        if (current == BLE::NOT_KNOWN) return {};
        if (max <= current) return 1;  // trivially 0, also handles case where max <= min
        if (current < min) return 0;   // trivially 1, below limiter kicks in

        auto range = max.value_or(0) - min.value_or(0);
        auto x = current.value_or(0) - min.value_or(0);
        assert(0 < max && "`0 < range` due to above checks");
        return x / range;
    }
} g_fan_power_thermal_limit;

struct [[gnu::packed]] FanPowerTachoAggregate {
    BLE::Percentage8 power = g_fan_power;
    RPM16 tachometer = g_tachometer.revolutions_per_second() * 60;
};

struct [[gnu::packed]] Aggregate {
    BLE::Percentage8 power = g_fan_power;
    BLE::Percentage8 power_override = g_fan_power_override;
    BLE::Percentage8 power_automatic = g_fan_power_automatic;
    BLE::Percentage8 power_coefficient = g_fan_power_coefficient;
    RPM16 tachometer = FanPowerTachoAggregate{}.tachometer;
};

auto g_notify_fan_power_tacho_aggregate = NotifyState<[](hci_con_handle_t conn) {
    att_server_notify(conn, HANDLE_ATTR(FAN_POWER_TACHO_AGGREGATE, VALUE), FanPowerTachoAggregate{});
}>();

auto g_notify_aggregate = NotifyState<[](hci_con_handle_t conn) {
    att_server_notify(conn, HANDLE_ATTR(FAN_AGGREGATE, VALUE), Aggregate{});
}>();

void fan_power_set(BLE::Percentage8 power) {
    auto thermal_scaler = 1.;
    if (auto coeff = g_fan_power_thermal_limit.coefficient.value_or(100); coeff < 100) {
        auto temp = max(nevermore::sensors::g_sensors.temperature_intake,
                nevermore::sensors::g_sensors.temperature_exhaust);
        if (auto perc = g_fan_power_thermal_limit.percent(temp)) {
            thermal_scaler = lerp(1, coeff / 100, *perc);
        }
    }

    power = power.value_or(0) * thermal_scaler;

    if (g_fan_power != power) {
        g_fan_power = power;
        g_notify_fan_power_tacho_aggregate.notify();  // `g_fan_power` changed
        g_notify_aggregate.notify();                  // `g_fan_power` changed
    }

    auto scale = (power.value_or(0) / 100.) * (g_fan_power_coefficient.value_or(0) / 100.);
    auto duty = uint16_t(numeric_limits<uint16_t>::max() * scale);
    pwm_set_gpio_duty(PIN_FAN_PWM, duty);
}

}  // namespace

double fan_power() {
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
    auto cfg_pwm = pwm_get_default_config();
    pwm_config_set_freq_hz(cfg_pwm, FAN_PWN_HZ);
    pwm_init(SLICE_PWM, &cfg_pwm, true);

    auto cfg_tachometer = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg_tachometer, PWM_DIV_B_FALLING);
    pwm_init(SLICE_TACHOMETER, &cfg_tachometer, false);

    // set fan PWM level
    fan_power_set(g_fan_power);

    g_tachometer.start();

    // HACK:  We'd like to notify on write to tachometer changes, but the code base isn't setup
    //        for that yet. Internally poll and update based on diffs for now.
    mk_timer("gatt-fan-tachometer-notify", SENSOR_UPDATE_PERIOD)([](auto*) {
        static double g_prev;
        if (g_prev == g_tachometer.revolutions_per_second()) return;

        g_prev = g_tachometer.revolutions_per_second();
        g_notify_fan_power_tacho_aggregate.notify();
        g_notify_aggregate.notify();
    });

    mk_timer("fan-policy", 1.s / FAN_POLICY_UPDATE_RATE_HZ)([](auto*) {
        static auto g_instance = g_fan_policy.instance();
        // keep updating even w/ `g_fan_power_override` set b/c we need to
        // refresh to account for thermal throttling policy
        if (g_fan_power_override == BLE::NOT_KNOWN) {
            fan_power_set(g_instance(nevermore::sensors::g_sensors) * g_fan_power_automatic.value_or(0));
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

optional<uint16_t> attr_read(
        hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    switch (att_handle) {
        USER_DESCRIBE(FAN_POWER, "Fan %")
        USER_DESCRIBE(FAN_POWER_OVERRIDE, "Fan % - Override")
        USER_DESCRIBE(FAN_POWER_AUTOMATIC, "Fan % - Automatic Coefficient")
        USER_DESCRIBE(FAN_POWER_COEFFICIENT, "Fan % - Limiting Coefficient")
        USER_DESCRIBE(TACHOMETER, "Fan RPM")
        USER_DESCRIBE(FAN_POWER_TACHO_AGGREGATE, "Aggregated Fan % and RPM")
        USER_DESCRIBE(FAN_AGGREGATE, "Aggregated Service Data")

        USER_DESCRIBE(FAN_POLICY_COOLDOWN, "How long to continue filtering after conditions are acceptable")
        USER_DESCRIBE(FAN_POLICY_VOC_PASSIVE_MAX, "Filter if any VOC sensor reaches this threshold")
        USER_DESCRIBE(FAN_POLICY_VOC_IMPROVE_MIN, "Filter if intake exceeds exhaust by this threshold")
        USER_DESCRIBE(FAN_POWER_THERMAL_LIMIT, "Thermal limiting cut-off")

        READ_VALUE(FAN_POWER, g_fan_power)
        READ_VALUE(FAN_POWER_OVERRIDE, g_fan_power_override)
        READ_VALUE(FAN_POWER_AUTOMATIC, g_fan_power_automatic)
        READ_VALUE(FAN_POWER_COEFFICIENT, g_fan_power_coefficient)
        READ_VALUE(TACHOMETER, FanPowerTachoAggregate{}.tachometer)
        READ_VALUE(FAN_POWER_TACHO_AGGREGATE, FanPowerTachoAggregate{})
        READ_VALUE(FAN_AGGREGATE, Aggregate{});  // default init populate from global state

        READ_VALUE(FAN_POLICY_COOLDOWN, g_fan_policy.cooldown)
        READ_VALUE(FAN_POLICY_VOC_PASSIVE_MAX, g_fan_policy.voc_passive_max)
        READ_VALUE(FAN_POLICY_VOC_IMPROVE_MIN, g_fan_policy.voc_improve_min)
        READ_VALUE(FAN_POWER_THERMAL_LIMIT, g_fan_power_thermal_limit)

        READ_CLIENT_CFG(FAN_POWER_TACHO_AGGREGATE, g_notify_fan_power_tacho_aggregate)
        READ_CLIENT_CFG(FAN_AGGREGATE, g_notify_aggregate)

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t const* buffer,
        uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;
    WriteConsumer consume{offset, buffer, buffer_size};

    switch (att_handle) {
        WRITE_VALUE(FAN_POLICY_COOLDOWN, g_fan_policy.cooldown)
        WRITE_VALUE(FAN_POLICY_VOC_PASSIVE_MAX, g_fan_policy.voc_passive_max)
        WRITE_VALUE(FAN_POLICY_VOC_IMPROVE_MIN, g_fan_policy.voc_improve_min)

        WRITE_CLIENT_CFG(FAN_POWER_TACHO_AGGREGATE, g_notify_fan_power_tacho_aggregate)
        WRITE_CLIENT_CFG(FAN_AGGREGATE, g_notify_aggregate)

    case HANDLE_ATTR(FAN_POWER_OVERRIDE, VALUE): {
        fan_power_override((BLE::Percentage8)consume);
        return 0;
    }

    case HANDLE_ATTR(FAN_POWER_AUTOMATIC, VALUE): {
        BLE::Percentage8 value = consume;
        if (value == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        g_fan_power_automatic = value;
        return 0;
    }

    case HANDLE_ATTR(FAN_POWER_COEFFICIENT, VALUE): {
        BLE::Percentage8 value = consume;
        if (value == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        g_fan_power_coefficient = value;
        return 0;
    }

    case HANDLE_ATTR(FAN_POWER_THERMAL_LIMIT, VALUE): {
        ThermalLimit value = consume;
        if (value.min == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);
        if (value.max == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);
        if (value.max < value.min) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        g_fan_power_thermal_limit = value;
        fan_power_set(g_fan_power);  // apply updated thermal coefficient
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::fan
