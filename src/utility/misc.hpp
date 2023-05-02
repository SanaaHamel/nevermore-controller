#pragma once

#include <cstdint>
#include <span>

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

using CRC8_t = uint8_t;

// polynomial x^8 + x^5 + x^4 + 1
constexpr inline CRC8_t crc8(std::span<uint8_t const> data, CRC8_t init) {
    CRC8_t crc = init;
    for (auto x : data) {
        crc ^= x;

        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31u;
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

template <typename A>
constexpr CRC8_t crc8(A const& blob, CRC8_t init) {
    static_assert(!std::is_pointer_v<A>, "probably a mistake, pass blob by ref");
    return crc8(std::span{reinterpret_cast<uint8_t const*>(&blob), sizeof(A)}, init);
}

template <typename A, CRC8_t CRC_INIT>
struct [[gnu::packed]] ResponseCRC {
    A data;
    CRC8_t crc;

    [[nodiscard]] bool verify() const {
        return crc == crc8(data, CRC_INIT);
    };
};
