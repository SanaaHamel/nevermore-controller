#pragma once

#include "bluetooth.h"
#include "pico/async_context.h"
#include <cstdint>
#include <optional>

namespace FanService {

bool init(async_context_t&);

std::optional<uint16_t> attr_read(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size);

std::optional<int> attr_write(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t const* buffer, uint16_t buffer_size);

void disconnected(hci_con_handle_t);

// Current fan power. [0, 100]
double fan_power();

}  // namespace FanService
