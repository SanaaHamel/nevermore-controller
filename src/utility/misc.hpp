#pragma once

#include <cstdint>

// grab bag of misc helpers.

template <typename T>
constexpr T clamp(T n, T min, T max) {
    if (n < min) return min;
    if (max < n) return max;
    return n;
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
