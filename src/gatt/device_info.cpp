#include "device_info.hpp"
#include "handler_helpers.hpp"
#include "pico/unique_id.h"
#include <algorithm>

using namespace std;

namespace nevermore::gatt::device_info {

bool init() {
    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(hci_con_handle_t conn, uint16_t attr, span<uint8_t> buffer) {
    switch (attr) {
    case HANDLE_ATTR(DEVICE_INFO_SERIAL, VALUE): {
        pico_get_unique_board_id_string(reinterpret_cast<char*>(buffer.data()), buffer.size());
        return min<uint16_t>(buffer.size(), 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
    }

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t, uint16_t, span<uint8_t const>) {
    return {};
}

}  // namespace nevermore::gatt::device_info
