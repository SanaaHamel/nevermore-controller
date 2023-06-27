#include "gatt.hpp"
#include "ble/att_server.h"
#include "ble/sm.h"
#include "bluetooth_gatt.h"
#include "boards/pico_w.h"
#include "btstack_event.h"
#include "config.hpp"
#include "gatt/environmental.hpp"
#include "gatt/fan.hpp"
#include "gatt/handler_helpers.hpp"
#include "gatt/neopixel.hpp"
#include "hci_dump.h"
#include "hci_dump_embedded_stdout.h"
#include "l2cap.h"
#include "nevermore.h"
#include "pico/cyw43_arch.h"
#include "sdk/gap.hpp"
#include "utility/bt_advert.hpp"
#include <array>
#include <cstdint>
#include <cstdio>
#include <tuple>

using namespace std;
using namespace bt::advert;

namespace {

// coincidentally packed b/c all `bt::advert` funcs return only tuples of packed members
constexpr tuple ADVERT{
        flags({
                Flag::LE_DISCOVERABLE,
                Flag::EDR_NOT_SUPPORTED,
        }),
        shortened_local_name("Nevermore"),
        services<ORG_BLUETOOTH_SERVICE_ENVIRONMENTAL_SENSING>(),
};

void hci_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    UNUSED(size);
    UNUSED(channel);
    if (packet_type != HCI_EVENT_PACKET) return;

    auto const event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
    case BTSTACK_EVENT_STATE: {
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;

        bd_addr_t local_addr;
        gap_local_bd_addr(local_addr);
        printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

        // setup advertisements
        static_assert(sizeof(ADVERT) <= 31, "too large for non-extended advertisement");
        gap_advertisements_set_params(ADVERTISE_INTERVAL_MIN, ADVERTISE_INTERVAL_MAX);
        gap_advertisements_set_data(sizeof(ADVERT), (uint8_t*)&ADVERT);  // NOLINT
        gap_advertisements_enable(1);
    } break;

    case ATT_EVENT_DISCONNECTED: {
        auto conn = att_event_disconnected_get_handle(packet);
        EnvironmentService::disconnected(conn);
        FanService::disconnected(conn);
        NeoPixelService::disconnected(conn);
    };
    }
}

uint16_t attr_read(
        hci_con_handle_t conn, uint16_t attr, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    constexpr array HANDLERS{
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

    constexpr array HANDLERS{
            EnvironmentService::attr_write,
            FanService::attr_write,
            NeoPixelService::attr_write,
    };

    try {
        for (auto handler : HANDLERS)
            if (auto r = handler(conn, attr, offset, buffer, buffer_size)) return *r;
    } catch (AttrWriteException const& e) {
        return e.error;
    }

    printf("WARN - BLE GATT - attr_write unhandled attr 0x%04x\n", int(attr));
    return 0;
}

btstack_packet_callback_registration_t g_hci_handler{.callback = &hci_handler};

btstack_timer_source_t g_led_blink{.process = [](auto* timer) {
    static bool led_on = false;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    btstack_run_loop_set_timer(timer, SENSOR_UPDATE_PERIOD / 1ms);
    btstack_run_loop_add_timer(timer);
}};

}  // namespace

bool gatt_init(async_context_t& ctx_async) {
    // must explicitly set, otherwise we get no error/info/debug msgs from btstack
    hci_dump_init(hci_dump_embedded_stdout_get_instance());

#ifndef ENABLE_LOG_DEBUG
    hci_dump_enable_packet_log(false);  // don't spam out packet logs unless we're low level debugging
#endif

    l2cap_init();
    sm_init();  // FUTURE WORK: do we even need a security manager? can we ditch this?

    EnvironmentService::init();
    if (!FanService::init(ctx_async)) return false;

    hci_add_event_handler(&g_hci_handler);

    att_server_init(profile_data, attr_read, attr_write);
    // not interested in attribute events for now, we have no indicator/notify attributes
    // att_server_register_packet_handler(att_handler);

    gap_set_max_number_peripheral_connections(MAX_NR_HCI_CONNECTIONS);

    // turn on bluetooth
    if (auto err = hci_power_control(HCI_POWER_ON)) {
        printf("hci_power_control failed = 0x%08x\n", err);
        return false;
    }

    g_led_blink.process(&g_led_blink);
    return true;
}
