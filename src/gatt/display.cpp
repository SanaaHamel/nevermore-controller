#include "display.hpp"
#include "../display.hpp"
#include "handler_helpers.hpp"
#include "sdk/ble_data_types.hpp"
#include "settings.hpp"

using namespace std;
using namespace BLE;

namespace nevermore::gatt::display {

bool init() {
    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(hci_con_handle_t conn, uint16_t attr, span<uint8_t> buffer) {
    switch (attr) {
        USER_DESCRIBE(DISPLAY_BRIGHTNESS, "Display Brightness %")
        USER_DESCRIBE(DISPLAY_UI, "Display UI")
        READ_VALUE(DISPLAY_BRIGHTNESS, Percentage8(nevermore::display::brightness() * 100));
        READ_VALUE(DISPLAY_UI, settings::g_active.display_ui);

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t attr, span<uint8_t const> buffer) {
    WriteConsumer consume{buffer};

    switch (attr) {
    case HANDLE_ATTR(DISPLAY_BRIGHTNESS, VALUE): {
        Percentage8 const power = consume;
        if (power == BLE::NOT_KNOWN) throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        nevermore::display::brightness(float(power.value_or(0) / 100.f));
        return 0;
    }

    case HANDLE_ATTR(DISPLAY_UI, VALUE): {
        settings::DisplayUI const ui = consume;
        if (!settings::validate(settings::g_active.display_hw, ui))
            throw AttrWriteException(ATT_ERROR_VALUE_NOT_ALLOWED);

        // FUTURE WORK: support UI switch w/o reboot
        settings::g_active.display_ui = ui;
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::display
