#pragma once

#include "bluetooth.h"
#include "gatt.hpp"
#include "sdk/ble_data_types.hpp"
#include <cstdint>
#include <optional>
#include <span>

namespace nevermore::gatt::fan {

std::optional<uint16_t> attr_read(hci_con_handle_t, uint16_t attr, uint16_t offset, std::span<uint8_t>);
std::optional<int> attr_write(hci_con_handle_t, uint16_t attr, std::span<uint8_t const>);

bool init();
void disconnected(hci_con_handle_t);

// Current fan power. [0, 100]
float fan_power();
float fan_rpm();

void fan_power_override(BLE::Percentage8 power);  // `NOT_KNOWN` to clear override
BLE::Percentage8 fan_power_override();

}  // namespace nevermore::gatt::fan
