#pragma once

#include <cstdint>

// grab bag of misc helpers.

template <typename T>
constexpr T clamp(T n, T min, T max) {
    if (n < min) return min;
    if (max < n) return max;
    return n;
}

// I2C reserves some addresses for special purposes.
// These are any addresses of the form: 000 0xxx, 111 1xxx
constexpr inline bool i2c_address_reserved(uint8_t addr) {
    constexpr uint8_t MASK = 0b1111000;
    auto const masked = addr & MASK;
    return masked == 0 || masked == MASK;
}

// included in C++23
constexpr inline uint8_t byteswap(uint8_t x) {
    return x;
}

constexpr inline uint16_t byteswap(uint16_t x) {
    return (x & 0xFF) << 8 | (x & 0xFF00) >> 8;
}

constexpr inline uint32_t byteswap(uint32_t x) {
    return byteswap(uint16_t(x & 0x0000FFFFu)) | (byteswap(uint16_t(x & 0xFFFF0000u >> 16)) << 16);
}
