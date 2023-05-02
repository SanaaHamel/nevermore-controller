#pragma once

#include <cstdint>

constexpr uint8_t operator"" _u8(unsigned long long x) {
    return x;
}

constexpr uint16_t operator"" _u16(unsigned long long x) {
    return x;
}

constexpr uint32_t operator"" _u32(unsigned long long x) {
    return x;
}

constexpr int8_t operator"" _s8(unsigned long long x) {
    return x;
}

constexpr int16_t operator"" _s16(unsigned long long x) {
    return x;
}

constexpr int32_t operator"" _s32(unsigned long long x) {
    return x;
}
