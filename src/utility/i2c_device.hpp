#pragma once

#include "sdk/i2c.hpp"
#include "sdk/task.hpp"
#include "utility/crc.hpp"
#include "utility/template_string_literal.hpp"
#include <concepts>
#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <optional>
#include <type_traits>
#include <utility>

namespace nevermore {

template <typename Register, TemplateStringLiteral name, CRC8_t CRC_Init = 0>
    requires(std::is_scoped_enum_v<Register>)
struct I2CDevice {
    template <typename A>
    static constexpr CRC8_t crc(A&& x) {
        return crc8(std::forward<A>(x), CRC_Init);
    }

    template <typename A>
    using ResponseCRC = ResponseCRC<A, CRC_Init>;

    I2C_Bus& bus;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    uint8_t address;

    [[nodiscard]] bool write(std::unsigned_integral auto reg, uint8_t const* data, size_t len) const {
        uint8_t cmd[sizeof(reg) + len];
        memcpy(cmd, &reg, sizeof(reg));
        memcpy(cmd + sizeof(reg), data, len);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        return bus.write(name, address, cmd, sizeof(cmd));
    }

    [[nodiscard]] bool read(std::unsigned_integral auto reg, uint8_t* dest, size_t len) const {
        return bus.read(name, address, reg, dest, len);
    }

    template <I2C_Data A>
    [[nodiscard]] bool write(Register reg, std::span<A const> xs) const {
        uint8_t cmd[sizeof(reg) + xs.size_bytes()];
        memcpy(cmd, &reg, sizeof(reg));
        memcpy(cmd + sizeof(reg), &*xs.begin(), xs.size_bytes());
        return bus.write(name, address, cmd, sizeof(cmd));
    }

    template <I2C_Data A>
    [[nodiscard]] bool write(Register reg, A value) const {
        uint8_t cmd[sizeof(reg) + sizeof(value)];
        memcpy(cmd, &reg, sizeof(reg));
        memcpy(cmd + sizeof(reg), &value, sizeof(value));
        return bus.write(name, address, cmd, sizeof(cmd));
    }

    [[nodiscard]] bool touch(Register reg) const {
        return bus.write(name, address, reg);
    }

    template <I2C_Data A>
    std::optional<A> read() const {
        A value;
        if (!bus.read(name, address, value)) return {};
        return value;
    }

    template <I2C_Data A>
    std::optional<A> read_crc() const {
        return bus.read_crc<CRC_Init, A>(name, address);
    }

    template <I2C_Data A, TaskDelayArg delay = {}>
    std::optional<A> read(Register reg) const {
        if constexpr (delay.us == 0) {
            A value;
            if (!bus.read(name, address, std::to_underlying(reg), value)) return {};
            return value;
        } else {
            if (!touch(reg)) return {};

            task_delay<delay>();
            return read<A>();
        }
    }

    template <I2C_Data A, TaskDelayArg delay = {}>
    std::optional<A> read_crc(Register reg) const {
        if constexpr (delay.us == 0) {
            return bus.read_crc<CRC_Init, A>(name, address, std::to_underlying(reg));
        } else {
            if (!touch(reg)) return {};

            task_delay<delay>();
            return read_crc<A>();
        }
    }

    template <typename A>
    bool verify(ResponseCRC<A> const& x) {
        return bus.verify(name, address, x);
    }

#define DEFINE_I2C_DEVICE_LOG(fn_name)            \
    [[gnu::format(printf, 2, 3)]]                 \
    void fn_name(const char* format, ...) const { \
        va_list xs;                               \
        va_start(xs, format);                     \
        bus.fn_name(name, address, format, xs);   \
        va_end(xs);                               \
    }

    DEFINE_I2C_DEVICE_LOG(log)
    DEFINE_I2C_DEVICE_LOG(log_warn)
    DEFINE_I2C_DEVICE_LOG(log_error)

#undef DEFINE_I2C_DEVICE_LOG
};

}  // namespace nevermore
