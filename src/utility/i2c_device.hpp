#pragma once

#include "sdk/i2c.hpp"
#include "sdk/task.hpp"
#include "utility/crc.hpp"
#include "utility/template_string_literal.hpp"
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

    I2C_Bus& bus;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    uint8_t address;

    [[nodiscard]] bool write(uint8_t reg, uint8_t const* data, size_t len) const {
        uint8_t cmd[len + 1];
        cmd[0] = reg;
        memcpy(cmd + 1, data, len);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        return bus.write(name, address, cmd, sizeof(cmd));
    }

    template <typename A>
    [[nodiscard]] bool write(Register reg, A value) const {
        uint8_t cmd[sizeof(reg) + sizeof(value)];
        memcpy(cmd, &reg, sizeof(reg));
        memcpy(cmd + sizeof(reg), &value, sizeof(value));
        return bus.write(name, address, cmd);
    }

    [[nodiscard]] bool touch(Register reg) const {
        return bus.write(name, address, reg);
    }

    [[nodiscard]] bool read(uint8_t reg, uint8_t* dest, size_t len) const {
        return bus.read(name, address, dest, len);
    }

    template <typename A>
    std::optional<A> read() const {
        A value;
        if (!bus.read(name, address, value)) return {};
        return value;
    }

    template <typename A>
    std::optional<A> read_crc() const {
        auto result = bus.read_crc<CRC_Init, A>(name, address);
        if (!result) return {};
        return *result;
    }

    template <typename A>
    std::optional<A> read(Register reg) const {
        if (!touch(reg)) return {};
        return read<A>();
    }

    template <typename A>
    std::optional<A> read_crc(Register reg) const {
        if (!touch(reg)) return {};
        return read_crc<A>();
    }

    template <typename A, typename Delay>
    std::optional<A> read(Register reg, Delay&& delay) const {
        if (!touch(reg)) return {};

        task_delay(std::forward<Delay>(delay));
        return read<A>();
    }

    template <typename A, typename Delay>
    std::optional<A> read_crc(Register reg, Delay&& delay) const {
        if (!touch(reg)) return {};

        task_delay(std::forward<Delay>(delay));
        return read_crc<A>();
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
