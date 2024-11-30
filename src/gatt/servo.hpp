#pragma once

#include "bluetooth.h"
#include "sdk/ble_data_types.hpp"
#include <cstdint>
#include <optional>

namespace nevermore::gatt::servo {

std::optional<uint16_t> attr_read(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size);

std::optional<int> attr_write(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t const* buffer, uint16_t buffer_size);

bool init();
void disconnected(hci_con_handle_t);

}  // namespace nevermore::gatt::servo
