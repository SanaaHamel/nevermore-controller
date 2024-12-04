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
        // if buffer is nullptr (not just zero-sized) then this is a size query
        constexpr size_t SERIAL_SIZE = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES;
        if (buffer.data() == nullptr) return SERIAL_SIZE;

        char temp[SERIAL_SIZE + 1];  // +1 b/c SDK call wants to null-terminate
        pico_get_unique_board_id_string(temp, sizeof(temp));
        return att_read_callback_handle_blob(temp, SERIAL_SIZE, buffer);
    }

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t, uint16_t, span<uint8_t const>) {
    return {};
}

}  // namespace nevermore::gatt::device_info
