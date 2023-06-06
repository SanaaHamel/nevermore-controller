#pragma once

#include "bluetooth_data_types.h"
#include "sdk/bt_data_types.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <initializer_list>

namespace bt::advert {

namespace internal {

template <typename A>
constexpr auto blob(uint8_t typ, A const& data) {
    static_assert(sizeof(A) <= UINT8_MAX - 1);
    std::array<uint8_t, sizeof(A) + 2> xs;  // NOLINT(cppcoreguidelines-pro-type-member-init)
    xs[0] = 1 + sizeof(A);
    xs[1] = typ;
    auto ys = std::bit_cast<std::array<uint8_t, sizeof(A)>>(data);
    std::copy(std::begin(ys), std::end(ys), xs.begin() + 2);
    return xs;
}

template <size_t M>
constexpr auto string(uint8_t typ, char const (&str)[M]) {
    // we're given a null terminated string, but we don't it b/c we're length prefixed
    assert(str[M - 1] == '\0' && "string should be null terminated");
    std::array<char, M - 1> xs;  // NOLINT(cppcoreguidelines-pro-type-member-init)
    std::copy(std::begin(str), std::end(str) - 1, xs.begin());  // NOLINT
    return blob(typ, xs);
}

}  // namespace internal

enum class Flag : uint8_t {
    LE_LIMITED_DISCOVERABLE = 1 << 0,
    LE_DISCOVERABLE = 1 << 1,
    EDR_NOT_SUPPORTED = 1 << 2,
    LE_EDR_CAPABLE_CONTROLLER = 1 << 3,
    LE_EDR_CAPABLE_HOST = 1 << 4,
};

constexpr auto flags(std::initializer_list<Flag> flags) {
    uint8_t x = 0;
    for (auto f : flags)
        x |= uint8_t(f);

    return internal::blob(BLUETOOTH_DATA_TYPE_FLAGS, x);
}
static_assert(sizeof(flags({})) == 3);
static_assert(std::get<0>(flags({})) == 2);
static_assert(std::get<1>(flags({})) == 1);
static_assert(std::get<2>(flags({
                      Flag::LE_DISCOVERABLE,
                      Flag::EDR_NOT_SUPPORTED,
              })) == 0b110);

// must be a prefix of the complete name
template <size_t M>
constexpr auto shortened_local_name(char const (&name)[M]) {
    return internal::string(BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME, name);
}

template <size_t M>
constexpr auto complete_local_name(char const (&name)[M]) {
    return internal::string(BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, name);
}

template <uint16_t... xs>
constexpr auto services() {
    return internal::blob(BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, std::array{xs...});
}

template <uint32_t... xs>
    requires((UINT16_MAX < xs) && ...)  // ensure can't be expressed as a 16 bit service ID
constexpr auto services() {
    return internal::blob(BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS, std::array{xs...});
}

template <BT::uint128_t... xs>
    requires((UINT32_MAX < xs) && ...)  // ensure can't be expressed as a 32 or 16 bit service ID
constexpr auto services() {
    return internal::blob(
            BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, std::array{xs...});
}

}  // namespace bt::advert
