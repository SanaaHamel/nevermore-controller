#pragma once

#include <array>
#include <bit>
#include <cassert>
#include <cstdint>
#include <tuple>

namespace BT {

struct [[gnu::packed]] uint128_t {
    std::array<uint8_t, 16> octets{};

    // parse a xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx UUID to little endian
    constexpr static uint128_t uuid(uint32_t a, uint16_t b, uint16_t c, uint16_t d, uint64_t e) {
        assert(e <= 0xFFFF'FFFF'FFFF);
        // octet order is ffeeddcc-bbaa-9988-7766-554433221100
        return std::array<uint8_t, 16>{
                uint8_t((e >> 0) & 0xFF),
                uint8_t((e >> 8) & 0xFF),
                uint8_t((e >> 16) & 0xFF),
                uint8_t((e >> 24) & 0xFF),
                uint8_t((e >> 32) & 0xFF),
                uint8_t((e >> 40) & 0xFF),
                uint8_t((d >> 0) & 0xFF),
                uint8_t((d >> 8) & 0xFF),
                uint8_t((c >> 0) & 0xFF),
                uint8_t((c >> 8) & 0xFF),
                uint8_t((b >> 0) & 0xFF),
                uint8_t((b >> 8) & 0xFF),
                uint8_t((a >> 0) & 0xFF),
                uint8_t((a >> 8) & 0xFF),
                uint8_t((a >> 16) & 0xFF),
                uint8_t((a >> 24) & 0xFF),
        };
    }

    constexpr uint128_t() = default;
    constexpr uint128_t(uint64_t x)
            : octets(std::bit_cast<std::array<uint8_t, 16>>(std::array<uint64_t, 2>{x, 0})) {}

    constexpr uint128_t(std::array<uint8_t, 16> const& octets) : octets(octets) {}

    constexpr auto operator<=>(uint128_t const& rhs) const {
        auto xs = std::bit_cast<std::array<uint64_t, 2>>(octets);
        auto ys = std::bit_cast<std::array<uint64_t, 2>>(rhs.octets);
        return std::tie(xs[1], xs[0]) <=> std::tie(ys[1], ys[0]);
    }
};

static_assert(sizeof(uint128_t) == 16);

static_assert(uint128_t::uuid(0xFFEEDDCC, 0xBBAA, 0x9988, 0x7766, 0x554433221100).octets ==
                      std::array<uint8_t, 16>{0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99,
                              0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF},
        "uint128::uuid failed parse test");

}  // namespace BT
