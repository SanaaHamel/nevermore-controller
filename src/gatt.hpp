#pragma once

#include "btstack_config.h"
#include <cstdint>
#include <optional>
#include <span>

namespace nevermore::gatt {

constexpr size_t ATTR_SIZE_MAX = HCI_ACL_PAYLOAD_SIZE;

// Setup bluetooth and GATT services.
// Caller is responsible for subsequently calling `btstack_run_loop_execute`.
bool init();

// used by stdio-serial
std::optional<uint16_t> read(uint16_t attr, std::span<uint8_t>);
std::optional<uint16_t> write(uint16_t attr, std::span<uint8_t const>);

}  // namespace nevermore::gatt
