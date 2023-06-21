#include "fan.hpp"
#include "config.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "sdk/pwm.hpp"
#include "sensors/tachometer.hpp"
#include "utility/fan_policy.hpp"
#include <cstdint>
#include <limits>
#include <utility>

using namespace std;

#define FAN_POWER 2B04_01
#define FAN_POWER_OVERRIDE 2B04_02
#define TACHOMETER 03f61fe0_9fe7_4516_98e6_056de551687f_01
// NB: Error prone, but we're the 2nd aggregation char instance in the DB
#define FAN_AGGREGATE 75134bec_dd06_49b1_bac2_c15e05fd7199_02

namespace {

BLE_DECLARE_SCALAR_TYPE(RPM16, uint16_t, 1, 0, 0);

constexpr uint8_t FAN_POLICY_UPDATE_RATE_HZ = 10;

constexpr uint8_t TACHOMETER_PULSE_PER_REVOLUTION = 2;
constexpr uint32_t FAN_PWN_HZ = 25'000;

constexpr auto SLICE_PWM = pwm_gpio_to_slice_num_(PIN_FAN_PWM);
constexpr auto SLICE_TACHOMETER = pwm_gpio_to_slice_num_(PIN_FAN_TACHOMETER);
static_assert(pwm_gpio_to_channel_(PIN_FAN_TACHOMETER) == PWM_CHAN_B, "can only read from B channel");

BLE::Percentage8 g_fan_power = 0;
BLE::Percentage8 g_fan_power_override;  // not-known -> automatic control
Tachometer g_tachometer{PIN_FAN_TACHOMETER, TACHOMETER_PULSE_PER_REVOLUTION};

struct Aggregate {
    BLE::Percentage8 power = g_fan_power;
    BLE::Percentage8 power_override = g_fan_power_override;
    RPM16 tachometer = g_tachometer.revolutions_per_second() * 60;
};

auto g_notify_aggregate = NotifyState<[](hci_con_handle_t conn) {
    att_server_notify(conn, HANDLE_ATTR(FAN_AGGREGATE, VALUE), Aggregate{});
}>();

void fan_power_set(BLE::Percentage8 power) {
    if (g_fan_power == power) return;
    g_fan_power = power;
    g_notify_aggregate.notify();  // `g_fan_power` changed

    auto scale = power.value_or(0) / 100;  // enable automatic control if `NOT_KNOWN`
    auto duty = uint16_t(numeric_limits<uint16_t>::max() * scale);
    pwm_set_gpio_duty(PIN_FAN_PWM, duty);
}

// HACK:  We'd like to notify on write to tachometer changes, but the code base isn't setup
//        for that yet. Internally poll and update based on diffs for now.
btstack_timer_source_t g_notify_pump_hack{.process = [](auto* timer) {
    btstack_run_loop_set_timer(timer, SENSOR_UPDATE_PERIOD / 1ms);
    btstack_run_loop_add_timer(timer);

    static double g_prev;
    if (g_prev == g_tachometer.revolutions_per_second()) return;

    g_prev = g_tachometer.revolutions_per_second();
    g_notify_aggregate.notify();
}};

btstack_timer_source_t g_fan_policy_update{.process = [](auto* timer) {
    // fan is manually controlled, no need to schedule any further updates until this changes
    if (g_fan_power_override != BLE::NOT_KNOWN) return;

    static_assert(0 < FAN_POLICY_UPDATE_RATE_HZ, "`FAN_POLICY_UPDATE_RATE_HZ` cannot be zero");
    btstack_run_loop_set_timer(timer, 1000 / FAN_POLICY_UPDATE_RATE_HZ);
    btstack_run_loop_add_timer(timer);

    // ask the oracle what we should be doing
    fan_power_set(fan_power_oracle() * 100);
}};

void fan_automatic_start() {
    // not quite idempotent, but close. won't cause a buildup of pending timers
    g_fan_policy_update.process(&g_fan_policy_update);
}

void fan_automatic_stop() {
    // idempotent
    btstack_run_loop_remove_timer(&g_fan_policy_update);
}

}  // namespace

bool FanService::init(async_context& ctx_async) {
    // setup PWM configurations for fan PWM and fan tachometer
    auto cfg_pwm = pwm_get_default_config();
    pwm_config_set_freq_hz(cfg_pwm, FAN_PWN_HZ);
    pwm_init(SLICE_PWM, &cfg_pwm, true);

    auto cfg_tachometer = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg_tachometer, PWM_DIV_B_FALLING);
    pwm_init(SLICE_TACHOMETER, &cfg_tachometer, false);

    // set fan PWM level
    fan_power_set(g_fan_power);
    fan_automatic_start();

    g_tachometer.register_(ctx_async);
    g_notify_pump_hack.process(&g_notify_pump_hack);

    return true;
}

void FanService::disconnected(hci_con_handle_t conn) {
    g_notify_aggregate.unregister(conn);
}

optional<uint16_t> FanService::attr_read(
        hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    switch (att_handle) {
        USER_DESCRIBE(FAN_POWER, "Fan %")
        USER_DESCRIBE(FAN_POWER_OVERRIDE, "Fan % - Override")
        USER_DESCRIBE(TACHOMETER, "Fan RPM")
        USER_DESCRIBE(FAN_AGGREGATE, "Aggregated Service Data")

        READ_VALUE(FAN_POWER, g_fan_power)
        READ_VALUE(FAN_POWER_OVERRIDE, g_fan_power_override)
        READ_VALUE(TACHOMETER, Aggregate{}.tachometer)
        READ_VALUE(FAN_AGGREGATE, Aggregate{});  // default init populate from global state

        READ_CLIENT_CFG(FAN_AGGREGATE, g_notify_aggregate)

        default: return {};
    }
}

optional<int> FanService::attr_write(hci_con_handle_t conn, uint16_t att_handle, uint16_t offset,
        uint8_t const* buffer, uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;
    WriteConsumer consume{offset, buffer, buffer_size};

    switch (att_handle) {
        WRITE_CLIENT_CFG(FAN_AGGREGATE, g_notify_aggregate)

        case HANDLE_ATTR(FAN_POWER_OVERRIDE, VALUE): {
            BLE::Percentage8 power = consume;
            if (g_fan_power_override == power) return 0;  // no-op

            if (g_fan_power_override == BLE::NOT_KNOWN) {
                fan_automatic_stop();
            }

            g_fan_power_override = power;
            g_notify_aggregate.notify();

            if (power != BLE::NOT_KNOWN)
                fan_power_set(power);  // apply override
            else
                fan_automatic_start();

            return 0;
        }

        default: return {};
    }
}
