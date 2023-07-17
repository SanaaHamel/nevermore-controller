#include "display.hpp"
#include "../display.hpp"
#include "bluetooth.h"
#include "config.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/pwm.hpp"
#include <cstdint>
#include <limits>

using namespace std;
using namespace BLE;

#define DISPLAY_BRIGHTNESS 2B04_03

namespace nevermore::gatt::display {

bool init(async_context_t&) {
    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(
        hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    switch (att_handle) {
        USER_DESCRIBE(DISPLAY_BRIGHTNESS, "Display Brightness %")
        READ_VALUE(DISPLAY_BRIGHTNESS, Percentage8(nevermore::display::brightness() * 100));

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t const* buffer,
        uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;
    WriteConsumer consume{offset, buffer, buffer_size};

    switch (att_handle) {
    case HANDLE_ATTR(DISPLAY_BRIGHTNESS, VALUE): {
        Percentage8 const power = consume;
        if (power == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        nevermore::display::brightness(float(power.value_or(0) / 100.f));
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::display
