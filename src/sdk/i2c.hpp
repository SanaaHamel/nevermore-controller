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

constexpr uint32_t I2C_TIMEOUT_US = 1000000;

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
    xSemaphoreTake(lock, portMAX_DELAY);
    return ScopeGuard{[&] { xSemaphoreGive(lock); }};
}

// I2C reserves some addresses for special purposes.
// These are any addresses of the form: 000 0xxx, 111 1xxx
constexpr bool i2c_address_reserved(uint8_t addr) {
    constexpr uint8_t MASK = 0b1111000;
    auto const masked = addr & MASK;
    return masked == 0 || masked == MASK;
}

inline bool i2c_write(char const* name, i2c_inst_t& i2c, uint8_t addr, uint8_t const* value, size_t len,
        bool nostop = false) {
    auto _ = i2c_guard(i2c);
    int r = i2c_write_timeout_us(&i2c, addr, value, len, nostop, I2C_TIMEOUT_US);
    if (r < 0 || size_t(r) != len) {
        printf("ERR - I2C%d - %s write failed; device=0x%02x len=%d result=%d\n", i2c_hw_index(&i2c), name,
                addr, len, r);
        return false;
    }

    return true;
}

inline bool i2c_read(char const* name, i2c_inst_t& i2c, uint8_t addr, uint8_t* dest, size_t len,
        bool nostop = false, char const* extra = "") {
    auto _ = i2c_guard(i2c);
    int r = i2c_read_timeout_us(&i2c, addr, dest, len, nostop, I2C_TIMEOUT_US);
    if (r < 0 || size_t(r) != len) {
        printf("ERR - I2C%d - %s read failed; device=0x%02x len=%d result=%d\n", i2c_hw_index(&i2c), extra,
                addr, len, r);
        return false;
    }

    return true;
}

template <typename A, bool report_error = true>
bool i2c_write(char const* name, i2c_inst_t& i2c, uint8_t addr, A const& blob, bool nostop = false)
    requires(!std::is_pointer_v<A>)
{
    return i2c_write(name, i2c, addr, reinterpret_cast<uint8_t const*>(&blob), sizeof(A), nostop);
}

template <typename A>
bool i2c_read(char const* name, i2c_inst_t& i2c, uint8_t addr, A& blob, bool nostop = false)
    requires(!std::is_pointer_v<A>)
{
    return i2c_read(name, i2c, addr, reinterpret_cast<uint8_t*>(&blob), sizeof(A), nostop);
}

template <typename A>
std::optional<A> i2c_read(char const* name, i2c_inst_t& i2c, uint8_t addr, bool nostop = false)
    requires(!std::is_pointer_v<A>)
{
    A result;
    if (!i2c_read(name, i2c, addr, result, nostop)) return {};
    return {std::move(result)};
}

template <CRC8_t CRC_INIT, typename... A>
std::optional<PackedTuple<A...>> i2c_read_blocking_crc(
        char const* name, i2c_inst_t& i2c, uint8_t addr, bool nostop = false) {
    static_assert(sizeof(PackedTuple<A...>) == (sizeof(A) + ...));  // sancheck packing

    ResponseCRC<PackedTuple<A...>, CRC_INIT> response;
    if (!i2c_read(name, i2c, addr, response, nostop)) return {};

    if (!response.verify()) {
        // really should show up in a log if they've noise in their wiring
        printf("ERR - I2C%d - %s read failed CRC; device=0x%02x crc-reported=0x%02x crc-computed=0x%02x\n",
                i2c_hw_index(&i2c), name, addr, response.crc, response.data_crc());
        return {};
    }

    return response.data;
}

}  // namespace nevermore
