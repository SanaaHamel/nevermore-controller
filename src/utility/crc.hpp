#pragma once

#include <cstdint>
#include <span>

namespace nevermore {

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

    [[nodiscard]] CRC8_t data_crc() const {
        return crc8(data, CRC_INIT);
    }

    [[nodiscard]] bool verify() const {
        return crc == data_crc();
    };
};

}  // namespace nevermore
