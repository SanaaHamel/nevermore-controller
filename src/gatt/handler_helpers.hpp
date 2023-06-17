#pragma once

#include <cstdint>
#include <span>
#include <type_traits>

#define BT(x) ORG_BLUETOOTH_CHARACTERISTIC_##x
#define HANDLE_ATTR_(attr, kind) ATT_CHARACTERISTIC_##attr##_##kind##_HANDLE
#define HANDLE_ATTR(attr, kind) HANDLE_ATTR_(attr, kind)

#define HANDLE_READ_BLOB(attr, kind, expr) \
    case HANDLE_ATTR(attr, kind): return readBlob(expr);

#define ESM_DESCRIBE(attr, desc) HANDLE_READ_BLOB(attr, ENVIRONMENTAL_SENSING_MEASUREMENT, desc)
#define READ_VALUE(attr, expr) HANDLE_READ_BLOB(attr, VALUE, expr)
#define SERVER_CFG_ALWAYS_BROADCAST(attr) HANDLE_READ_BLOB(attr, SERVER_CONFIGURATION, uint16_t(0x0001))
#define USER_DESCRIBE(attr, desc) HANDLE_READ_BLOB(attr, USER_DESCRIPTION, desc "")

struct AttrWriteException {
    int error;
};

struct WriteConsumer {
    struct NotEnoughException {};

    uint16_t offset;
    uint8_t const* buffer;
    uint16_t buffer_size;

    template <typename A>
        requires(std::is_standard_layout_v<A> && !std::is_pointer_v<A> && !std::is_reference_v<A>)
    operator A() {
        if (!has_available(sizeof(A))) throw AttrWriteException(ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH);

        // Ideally we'd like to just return a ptr within the buffer, but sadly
        // ARM has stricter alignment requirements than x86's *ANYTHING-GOES!* approach.
        A value;
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        memcpy(&value, buffer + offset, sizeof(value));
        offset += sizeof(A);
        return value;
    }

    std::span<uint8_t const> span(size_t length) {
        if (!has_available(sizeof(uint8_t) * length))
            throw AttrWriteException(ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH);

        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        auto const* ptr = buffer + offset;
        offset += sizeof(uint8_t) * length;
        return {ptr, ptr + length};  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }

    [[nodiscard]] uint16_t remaining() const {
        if (buffer_size < offset) return 0;

        return buffer_size - offset;
    }

private:
    [[nodiscard]] constexpr bool has_available(size_t n) const {
        // Always return false, even for 0 byte reads, if beyond end of buffer.
        // This prevents creating pointers beyond the last of an array + 1 (which is UB).
        if (buffer_size < offset) return false;

        auto const available = size_t(buffer_size) - offset;
        return n <= available;
    }
};
