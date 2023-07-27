#pragma once

#include "FreeRTOS.h"
#include "hardware/i2c.h"
#include "semphr.h"  // IWYU pragma: keep [doesn't notice `SemaphoreHandle_t`]
#include "utility/crc.hpp"
#include "utility/packed_tuple.hpp"
#include "utility/scope_guard.hpp"
#include <cstdio>
#include <optional>
#include <type_traits>

// NB: These variants are lock guarded to prevent both cores from using the same bus.

namespace nevermore {

enum class I2C_Pin : uint8_t { SDA = 0, SCL = 1 };

constexpr uint8_t i2c_gpio_bus_num(uint8_t pin) {
    return (pin / 2) & 1u;
}

constexpr I2C_Pin i2c_gpio_kind(uint8_t pin) {
    return I2C_Pin(pin % 2);
}

extern SemaphoreHandle_t g_i2c_locks[NUM_I2CS];

inline auto i2c_guard(i2c_inst_t& i2c) {
    auto& lock = g_i2c_locks[i2c_hw_index(&i2c)];  // NOLINT
    return ScopeGuard{[&] { xSemaphoreGive(lock); }};
}

// I2C reserves some addresses for special purposes.
// These are any addresses of the form: 000 0xxx, 111 1xxx
constexpr bool i2c_address_reserved(uint8_t addr) {
    constexpr uint8_t MASK = 0b1111000;
    auto const masked = addr & MASK;
    return masked == 0 || masked == MASK;
}

template <typename A>
int i2c_write_blocking(i2c_inst_t& i2c, uint8_t addr, A const& blob, bool nostop = false)
    requires(!std::is_pointer_v<A>)
{
    auto _ = i2c_guard(i2c);
    return i2c_write_blocking(&i2c, addr, reinterpret_cast<uint8_t const*>(&blob), sizeof(A), nostop);
}

template <typename A>
int i2c_read_blocking(i2c_inst_t& i2c, uint8_t addr, A& blob, bool nostop = false)
    requires(!std::is_pointer_v<A>)
{
    auto _ = i2c_guard(i2c);
    return i2c_read_blocking(&i2c, addr, reinterpret_cast<uint8_t*>(&blob), sizeof(A), nostop);
}

template <CRC8_t CRC_INIT, typename... A>
std::optional<PackedTuple<A...>> i2c_read_blocking_crc(i2c_inst_t& i2c, uint8_t addr, bool nostop = false) {
    static_assert(sizeof(PackedTuple<A...>) == (sizeof(A) + ...));  // sancheck packing

    ResponseCRC<PackedTuple<A...>, CRC_INIT> response;
    auto ret = i2c_read_blocking(i2c, addr, response, nostop);
    if (sizeof(response) != ret) return {};

    if (!response.verify()) {
        // really should show up in a log if they've noise in their wiring
        printf("ERR - I2C%d - read failed CRC; device=0x%02x crc-computed=0x%02x crc-expected=0x%02x\n",
                i2c_hw_index(&i2c), addr, response.data_crc(), response.crc);
        return {};
    }

    return response.data;
}

}  // namespace nevermore
