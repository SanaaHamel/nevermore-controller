#include "fan.hpp"
#include "config.hpp"
#include "gatt.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "sdk/pwm.hpp"
#include "sensors/tachometer.hpp"
#include <cstdint>
#include <limits>
#include <utility>

// Basically PERCENTAGE8_01, but we don't have a SDK def for percentage-8
#define PWM_01 2B04_01
#define TACHOMETER_UUID 03f61fe0_9fe7_4516_98e6_056de551687f
#define TACHOMETER_01 03f61fe0_9fe7_4516_98e6_056de551687f_01

namespace {

BLE_DECLARE_SCALAR_TYPE(RPM16, uint16_t, 1, 0, 0);

constexpr uint8_t TACHOMETER_PULSE_PER_REVOLUTION = 2;
constexpr uint32_t FAN_PWN_HZ = 25'000;

constexpr auto SLICE_PWM = pwm_gpio_to_slice_num_(PIN_FAN_PWM);
constexpr auto CHANNEL_PWM = pwm_gpio_to_channel_(PIN_FAN_PWM);
constexpr auto SLICE_TACHOMETER = pwm_gpio_to_slice_num_(PIN_FAN_TACHOMETER);
constexpr auto CHANNEL_TACHOMETER = pwm_gpio_to_channel_(PIN_FAN_TACHOMETER);
static_assert(CHANNEL_TACHOMETER == PWM_CHAN_B, "can only read from B channel");

BLE::Percentage8 g_fan_pwm_percent = 0;

Tachometer g_tachometer{PIN_FAN_TACHOMETER, TACHOMETER_PULSE_PER_REVOLUTION};

void fan_pwm_update(BLE::Percentage8 percent) {
    auto scale = percent.value_or(0) / 100;  // enable automatic control if `NOT_KNOWN`
    auto duty = uint16_t(std::numeric_limits<uint16_t>::max() * scale);
    pwm_set_gpio_duty(PIN_FAN_PWM, duty);
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
    fan_pwm_update(g_fan_pwm_percent);

    g_tachometer.register_(ctx_async);

    return true;
}

std::optional<uint16_t> FanService::attr_read(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    auto readBlob = [&](auto&& datum) -> uint16_t {
        return att_read_callback_handle_blob(
                std::forward<decltype(datum)>(datum), offset, buffer, buffer_size);
    };

    switch (att_handle) {
        USER_DESCRIBE(PWM_01, "Fan %")
        USER_DESCRIBE(TACHOMETER_01, "Fan RPM")

        READ_VALUE(PWM_01, g_fan_pwm_percent)
        READ_VALUE(TACHOMETER_01, RPM16{g_tachometer.revolutions_per_second() * 60})

        default: return {};
    }
}

std::optional<int> FanService::attr_write(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t const* buffer, uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;
    WriteConsumer consume{offset, buffer, buffer_size};

    switch (att_handle) {
        case HANDLE_ATTR(PWM_01, VALUE): {
            BLE::Percentage8 const* percent = consume;
            if (!percent) return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;

            g_fan_pwm_percent = *percent;
            fan_pwm_update(g_fan_pwm_percent);
            return 0;
        }

        default: return {};
    }
}
