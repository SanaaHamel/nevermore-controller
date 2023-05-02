
#pragma once

#include <cstdint>
#include <span>

#define BT(x) ORG_BLUETOOTH_CHARACTERISTIC_##x
#define HANDLE_ATTR_(attr, kind) ATT_CHARACTERISTIC_##attr##_##kind##_HANDLE
#define HANDLE_ATTR(attr, kind) HANDLE_ATTR_(attr, kind)

#define HANDLE_READ_BLOB(attr, kind, expr) \
    case HANDLE_ATTR(attr, kind): return readBlob(expr);

#define ESM_DESCRIBE(attr, desc) HANDLE_READ_BLOB(attr, ENVIRONMENTAL_SENSING_MEASUREMENT, desc)
#define READ_VALUE(attr, expr) HANDLE_READ_BLOB(attr, VALUE, expr)
#define SERVER_CFG_ALWAYS_BROADCAST(attr) HANDLE_READ_BLOB(attr, SERVER_CONFIGURATION, uint16_t(0x0001))
#define USER_DESCRIBE(attr, desc) HANDLE_READ_BLOB(attr, USER_DESCRIPTION, desc "")

struct WriteConsumer {
    uint16_t& offset;
    uint8_t const* const buffer;
    uint16_t const buffer_size;

    template <typename A>
    operator A const*() const {
        if (!has_available(sizeof(A))) return {};

        auto* const ptr = reinterpret_cast<A const*>(
                buffer + offset);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        offset += sizeof(A);
        return ptr;
    }

    template <typename A>
    std::span<A const> span(size_t length) const {
        if (!has_available(sizeof(A) * length)) return {};

        auto* const ptr = reinterpret_cast<A const*>(
                buffer + offset);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        offset += sizeof(A) * length;
        return {ptr, ptr + length};
    }

private:
    [[nodiscard]] constexpr bool has_available(size_t n) const {
        if (buffer_size < offset) return false;

        auto const available = size_t(buffer_size) - offset;
        return available <= n;
    }
};
