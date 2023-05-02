#include "ble/att_db.h"
#include "ble/att_server.h"
#include "ble/sm.h"
#include "boards/pico_w.h"
#include "btstack_event.h"
#include "config.hpp"
#include "gap.h"
#include "gatt.hpp"
#include "gatt/environmental.hpp"
#include "gatt/fan.hpp"
#include "gatt/neopixel.hpp"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "l2cap.h"
#include "nevermore.h"
#include "pico/binary_info.h"  // IWYU pragma: keep
#include "pico/cyw43_arch.h"
#include "pico/stdio.h"
#include "sensors.hpp"
#include <array>
#include <cstdint>
#include <cstdio>

using namespace std::literals;

namespace {

constexpr uint32_t LED = CYW43_WL_GPIO_LED_PIN;
constexpr uint32_t I2C_INTAKE_SCL = 0;
constexpr uint32_t I2C_INTAKE_SDA = 0;
constexpr uint32_t I2C_EXHAUST_SCL = 0;
constexpr uint32_t I2C_EXHAUST_SDA = 0;

}  // namespace

bi_decl(bi_program_description("Nevermore BLE Controller"));
// clang-format off
bi_decl(bi_4pins_with_names(
    I2C_INTAKE_SCL , "Intake I2C SCL",
    I2C_INTAKE_SDA , "Intake I2C SDA",
    I2C_EXHAUST_SCL, "Exhaust I2C SCL",
    I2C_EXHAUST_SDA, "Exhaust I2C SDA"));
// clang-format on

AdvertiseData g_advertise_data;

namespace {

bool g_led_on = true;

void sensor_update_handler(btstack_timer_source_t* ts) {
    g_led_on = !g_led_on;
    cyw43_arch_gpio_put(LED, g_led_on);

    btstack_run_loop_set_timer(ts, SENSOR_UPDATE_PERIOD / 1ms);
    btstack_run_loop_add_timer(ts);
}

void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    if (packet_type != HCI_EVENT_PACKET) return;

    auto const event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
        case HCI_EVENT_DISCONNECTION_COMPLETE: break;  // TODO: spin-down fans & shutdown LEDs?

        case BTSTACK_EVENT_STATE: {
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;

            bd_addr_t local_addr;
            gap_local_bd_addr(local_addr);
            printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

            // setup advertisements
            bd_addr_t null_addr{};
            // 0.625ms seems to be a standard constant for scaling advertising interval.
            // Why? IDK. Haven't found the spot in the spec.
            gap_advertisements_set_params(ADVERTISE_INTERVAL_MIN / 0.625ms, ADVERTISE_INTERVAL_MAX / 0.625ms,
                    0, 0, null_addr, 0x07, 0x00);
            gap_advertisements_set_data(
                    sizeof(g_advertise_data), reinterpret_cast<uint8_t*>(&g_advertise_data));
            gap_advertisements_enable(1);
        } break;
    }
}

uint16_t attr_read(
        hci_con_handle_t conn, uint16_t attr, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    constexpr std::array HANDLERS{
            EnvironmentService::attr_read,
            FanService::attr_read,
            NeoPixelService::attr_read,
    };
    for (auto handler : HANDLERS)
        if (auto r = handler(conn, attr, offset, buffer, buffer_size)) return *r;

    printf("WARN - BLE GATT - attr_read unhandled attr 0x%04x\n", int(attr));
    return 0;
}

int attr_write(hci_con_handle_t conn, uint16_t attr, uint16_t transaction_mode, uint16_t offset,
        uint8_t* buffer, uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;

    // We don't support any kind of transaction modes.
    if (transaction_mode != ATT_TRANSACTION_MODE_NONE) {
        printf("WARN - BLE GATT - attr_write unhandled transaction mode 0x%04x\n", int(transaction_mode));
        return 0;
    }

    constexpr std::array HANDLERS{
            EnvironmentService::attr_write,
            FanService::attr_write,
            NeoPixelService::attr_write,
    };

    for (auto handler : HANDLERS)
        if (auto r = handler(conn, attr, offset, buffer, buffer_size)) return *r;

    printf("WARN - BLE GATT - attr_write unhandled attr 0x%04x\n", int(attr));
    return 0;
}

// so far PIN config can be statically checked, so no risk of runtime error
void pins_setup() {
    // Leave pins {0, 1} set to UART TX/RX.
    // Clear everything else.
    for (GPIO_Pin pin = 2; pin < PIN_MAX; ++pin) {
        gpio_set_function(pin, GPIO_FUNC_NULL);
        gpio_set_pulls(pin, false, false);
    }

    for (auto pin : PINS_I2C) {
        gpio_set_function(pin, GPIO_FUNC_I2C);
        gpio_pull_up(pin);
    }

    gpio_set_function(PIN_FAN_PWM, GPIO_FUNC_PWM);
    gpio_set_function(PIN_FAN_TACHOMETER, GPIO_FUNC_PWM);
    gpio_pull_up(PIN_FAN_TACHOMETER);

    // we're setting up the WS2812 controller on PIO0
    gpio_set_function(PIN_NEOPIXEL_DATA_IN, GPIO_FUNC_PIO0);
}

}  // namespace

int main() {
    stdio_init_all();
    adc_init();

    if (auto err = cyw43_arch_init()) {
        printf("cyw43_arch_init failed = 0x%08x\n", err);
        return -1;
    }

    l2cap_init();
    sm_init();

    // GCC 10.3.1 bug: -Werror=format reports that `I2C_BAUD_RATE_HZ` is a `long unsigned int`.
    // This is technically true on this platform, see static-assert below, but it is benign since
    // `unsigned == long unsigned int` is also true on this platform. Pedant.
    // Fix by casting to unsigned instead of changing format specifier, this keeps clangd happy
    // since `clangd`, incorrectly, thinks that `unsigned != long unsigned int` on this platform.
    static_assert(sizeof(I2C_BAUD_RATE_HZ) == sizeof(unsigned));
    printf("I2C bus 0 running at %u hz (requested %u hz)\n", i2c_init(i2c0, I2C_BAUD_RATE_HZ),
            unsigned(I2C_BAUD_RATE_HZ));
    printf("I2C bus 1 running at %u hz (requested %u hz)\n", i2c_init(i2c1, I2C_BAUD_RATE_HZ),
            unsigned(I2C_BAUD_RATE_HZ));

    auto& ctx_async = *cyw43_arch_async_context();

    pins_setup();
    if (!FanService::init(ctx_async)) return -1;
    if (!sensors_init(ctx_async, g_advertise_data.environment_service_data)) return -1;

    btstack_packet_callback_registration_t callback_handler{.callback = &packet_handler};
    hci_add_event_handler(&callback_handler);

    att_server_init(profile_data, attr_read, attr_write);
    att_server_register_packet_handler(packet_handler);

    btstack_timer_source_t sensor_timer{.process = &sensor_update_handler};
    btstack_run_loop_set_timer(&sensor_timer, SENSOR_UPDATE_PERIOD / 1ms);
    btstack_run_loop_add_timer(&sensor_timer);

    // turn on bluetooth
    if (auto err = hci_power_control(HCI_POWER_ON)) {
        printf("hci_power_control failed = 0x%08x\n", err);
        return -1;
    }

    btstack_run_loop_execute();  // !! NO-RETURN
    return 0;
}
