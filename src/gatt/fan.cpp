#include "fan.hpp"
#include "config.hpp"
#include "handler_helpers.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "sdk/pwm.hpp"
#include "sensors.hpp"
#include "sensors/tachometer.hpp"
#include "settings.hpp"
#include "utility/fan_policy.hpp"
#include "utility/fan_policy_thermal.hpp"
#include "utility/timer.hpp"
#include <chrono>
#include <limits>

using namespace std;

namespace nevermore::gatt::fan {

namespace {

BLE_DECL_SCALAR(RPM16, uint16_t, 1, 0, 0);

constexpr uint8_t FAN_CONTROL_UPDATE_HZ = 10;

constexpr uint8_t TACHOMETER_PULSE_PER_REVOLUTION = 2;
constexpr uint32_t FAN_PWM_HZ = 25'000;

sensors::Tachometer g_tachometer;

struct FanPowerControl {
    using Clock = std::chrono::steady_clock;

    void update(sensors::Sensors const& sensors = sensors::g_sensors,
            settings::Settings const& settings = settings::g_active);

    void target(BLE::Percentage8 target) {
        _target = target;
    }

    [[nodiscard]] BLE::Percentage8 power() const {
        return _power;
    }

private:
    [[nodiscard]] bool kick_start_required(double target, settings::Settings const& settings) const {
        // we're already kick-starting
        if (kick_starting()) return false;
        // stronger than kick-start power, just use that
        if (settings.fan_power_kick_start_min <= target) return false;
        // can't sustain target, no point kick-starting
        if (target < settings.fan_power_min) return false;

        // HEURISTIC: RPM is high enough that we're don't need to overcome start-up inertia
        constexpr auto RPM_MIN = 100;

        // if tacho available then we can precisely avoid unnecessary kick-starts to overcome inertia
        if (g_tachometer.available()) return fan_rpm() < RPM_MIN;

        // otherwise we have to use heuristics
        // (`_power == 0` fallback as a failsafe for incorrectly configured controllers)
        return _power == 0 || _power < settings.fan_power_min;
    }

    [[nodiscard]] double kick_start_power(double target, settings::Settings const& settings) const {
        auto kick_start_power = kick_starting() ? settings.fan_power_kick_start_min.value_or(100) : 0;
        return max(target, kick_start_power);
    }

    Clock::time_point kick_start_end = Clock::time_point::min();
    // b/c the update timer might be delayed, only mark it as requested instead
    // of updating the timeout period immediately and potentially not kick
    // starting for the full period
    bool kick_start_requested = false;
    BLE::Percentage8 _power = 0;
    BLE::Percentage8 _target = 0;

    [[nodiscard]] bool kick_starting() const {
        return Clock::now() < kick_start_end;
    }
};

FanPowerControl g_fan;
BLE::Percentage8 g_fan_power_override;  // not-known -> automatic control

struct [[gnu::packed]] FanPowerTachoAggregate {
    BLE::Percentage8 power = g_fan.power();
    RPM16 tachometer = fan_rpm();
};

struct [[gnu::packed]] Aggregate {
    BLE::Percentage8 power = g_fan.power();
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

void fan_power_set(BLE::Percentage8 target) {
    g_fan.target(target);
}

void FanPowerControl::update(sensors::Sensors const& sensors, settings::Settings const& settings) {
    double power = 100;
    if (!kick_starting()) {
        auto temperature = max(sensors.temperature_intake, sensors.temperature_exhaust);
        auto thermal_scaler = settings.fan_policy_thermal(temperature);
        power = _target.value_or(0) * thermal_scaler;
    }

    if (_power != power) {
        _power = power;
        g_notify_fan_power_tacho_aggregate.notify();  // `g_fan_power` changed
        g_notify_aggregate.notify();                  // `g_fan_power` changed
    }

    power *= settings.fan_power_coefficient.value_or(100) / 100.;
    if (kick_start_required(power, settings)) {
        kick_start_end =
                Clock::now() + std::chrono::milliseconds(uint64_t(1000 * settings.fan_kick_start_sec));
    }

    power = kick_start_power(power, settings);

    auto duty = uint16_t(numeric_limits<uint16_t>::max() * (power / 100.));
    for (auto&& pin : Pins::active().fan_pwm)
        if (pin) pwm_set_gpio_duty(pin, duty);
}

}  // namespace

float fan_rpm() {
    return g_tachometer.revolutions_per_second() * 60;
}

float fan_power() {
    // NOLINTNEXTLINE(bugprone-narrowing-conversions, cppcoreguidelines-narrowing-conversions)
    return g_fan.power().value_or(0);
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

    // HACK:  We'd like to notify on write to tachometer changes, but the code base isn't setup
    //        for that yet. Internally poll and update based on diffs for now.
    mk_timer("gatt-fan-tachometer-notify", SENSOR_UPDATE_PERIOD)([](auto*) {
        static decltype(fan_rpm()) g_prev;
        if (g_prev == fan_rpm()) return;

        g_prev = fan_rpm();
        g_notify_fan_power_tacho_aggregate.notify();
        g_notify_aggregate.notify();
    });

    mk_timer("fan-control", 1.s / FAN_CONTROL_UPDATE_HZ)([](auto*) {
        static auto g_instance = settings::g_active.fan_policy_env.instance();
        // keep updating even w/ `g_fan_power_override` set b/c:
        // * need to refresh to account for thermal throttling policy
        // * automatic PID needs to be kept up to date for when we disengage
        auto perc = ({
            // HACK: FIXME: The fan control timer has been observed to stall.
            //              (Presumably this means all timers stall, but I haven't been able to confirm.)
            //              I've  had no luck reproducing this w/ debugger.
            //              For now, disable the guard. This'll allow read races,
            //              but those should be benign.
            // auto _ = sensors::sensors_guard();
            g_instance(sensors::g_sensors);
        });

        if (g_fan_power_override == BLE::NOT_KNOWN) {
            if (0 < perc)
                fan_power_set(perc * settings::g_active.fan_power_automatic.value_or(0));
            else
                fan_power_set(settings::g_active.fan_power_passive.value_or(0));
        }

        g_fan.update();
    });

    return true;
}

void disconnected(hci_con_handle_t conn) {
    g_notify_fan_power_tacho_aggregate.unregister(conn);
    g_notify_aggregate.unregister(conn);
}

optional<uint16_t> attr_read(
        hci_con_handle_t const conn, uint16_t const attr, uint16_t const offset, span<uint8_t> const buffer) {
    switch (attr) {
        USER_DESCRIBE(FAN_POWER, "Fan %")
        USER_DESCRIBE(FAN_POWER_OVERRIDE, "Fan % - Override")
        USER_DESCRIBE(FAN_POWER_PASSIVE, "Fan % - Passive")
        USER_DESCRIBE(FAN_POWER_AUTOMATIC, "Fan % - Automatic")
        USER_DESCRIBE(FAN_POWER_COEFFICIENT, "Fan % - Limiting Coefficient")
        USER_DESCRIBE(FAN_POWER_ABS_MIN, "Fan % Abs. - Minimum Power")
        USER_DESCRIBE(FAN_POWER_ABS_KICK_START_MIN, "Fan % Abs. - Minimum Kick-start Power")
        USER_DESCRIBE(FAN_KICK_START_TIME, "Fan - Kick-start Time")
        USER_DESCRIBE(FAN_TACHOMETER, "Fan RPM")
        USER_DESCRIBE(FAN_POWER_TACHO_AGGREGATE, "Aggregated Fan % and RPM")
        USER_DESCRIBE(FAN_AGGREGATE, "Aggregated Service Data")

        USER_DESCRIBE(FAN_POLICY_COOLDOWN, "How long to continue filtering after conditions are acceptable")
        USER_DESCRIBE(FAN_POLICY_VOC_PASSIVE_MAX, "Filter if any VOC sensor reaches this threshold")
        USER_DESCRIBE(FAN_POLICY_VOC_IMPROVE_MIN, "Filter if intake exceeds exhaust by this threshold")
        USER_DESCRIBE(FAN_POWER_THERMAL_LIMIT, "Thermal limiting cut-off")

        READ_VALUE(FAN_POWER, g_fan.power())
        READ_VALUE(FAN_POWER_OVERRIDE, g_fan_power_override)
        READ_VALUE(FAN_POWER_PASSIVE, settings::g_active.fan_power_passive)
        READ_VALUE(FAN_POWER_AUTOMATIC, settings::g_active.fan_power_automatic)
        READ_VALUE(FAN_POWER_COEFFICIENT, settings::g_active.fan_power_coefficient)
        READ_VALUE(FAN_POWER_ABS_MIN, settings::g_active.fan_power_min)
        READ_VALUE(FAN_POWER_ABS_KICK_START_MIN, settings::g_active.fan_power_kick_start_min)
        READ_VALUE(FAN_KICK_START_TIME, BLE::TimeMilli24(1000.f * settings::g_active.fan_kick_start_sec))
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

    case HANDLE_ATTR(FAN_POWER_ABS_MIN, VALUE): {
        BLE::Percentage8 value = consume;
        if (value == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        settings::g_active.fan_power_min = value;
        return 0;
    }

    case HANDLE_ATTR(FAN_POWER_ABS_KICK_START_MIN, VALUE): {
        BLE::Percentage8 value = consume;
        if (value == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        settings::g_active.fan_power_kick_start_min = value;
        return 0;
    }

    case HANDLE_ATTR(FAN_KICK_START_TIME, VALUE): {
        BLE::TimeMilli24 value = consume;
        if (value == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        auto sec = float(value.value_or(0)) / 1000;
        if (sec < 0 || 1 < sec) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        settings::g_active.fan_kick_start_sec = sec;
        return 0;
    }

    case HANDLE_ATTR(FAN_POWER_THERMAL_LIMIT, VALUE): {
        FanPolicyThermal value = consume;
        value = value.or_(settings::g_active.fan_policy_thermal);
        if (!value.validate()) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        settings::g_active.fan_policy_thermal = value;
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::fan
