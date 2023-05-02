#pragma once

#include "hardware/i2c.h"
#include "utility/misc.hpp"
#include "utility/packed_tuple.hpp"
#include <optional>
#include <type_traits>

template <typename A>
int i2c_write_blocking(i2c_inst_t* i2c, uint8_t addr, A const& blob, bool nostop = false) {
    static_assert(!std::is_pointer_v<A>, "probably a mistake, pass blob by ref");
    return i2c_write_blocking(i2c, addr, reinterpret_cast<uint8_t const*>(&blob), sizeof(A), nostop);
}

template <typename A>
int i2c_read_blocking(i2c_inst_t* i2c, uint8_t addr, A& blob, bool nostop = false) {
    static_assert(!std::is_pointer_v<A>, "probably a mistake, pass blob by ref");
    return i2c_read_blocking(i2c, addr, reinterpret_cast<uint8_t*>(&blob), sizeof(A), nostop);
}

template <CRC8_t CRC_INIT, typename... A>
std::optional<PackedTuple<A...>> i2c_read_blocking_crc(i2c_inst_t* i2c, uint8_t addr, bool nostop = false) {
    static_assert(sizeof(PackedTuple<A...>) == (sizeof(A) + ...));  // sancheck packing

    ResponseCRC<PackedTuple<A...>, CRC_INIT> response;
    auto ret = i2c_read_blocking(i2c, addr, response, nostop);
    if (sizeof(response) != ret) return {};

    if (!response.verify()) {
        printf("CRC failed\n");  // really should show up in a log if they've noise in their wiring
        return {};
    }

    return response.data;
}
