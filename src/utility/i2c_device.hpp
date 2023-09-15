#pragma once

#include "sdk/i2c.hpp"
#include "sdk/task.hpp"
#include "utility/crc.hpp"
#include "utility/template_string_literal.hpp"
#include <cstdarg>
#include <cstdint>
#include <cstdio>
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

    i2c_inst_t& bus;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    uint8_t address;

    template <typename A>
    bool write(Register reg, A value) const {
        uint8_t cmd[sizeof(reg) + sizeof(value)];
        memcpy(cmd, &reg, sizeof(reg));
        memcpy(cmd + sizeof(reg), &value, sizeof(value));
        return i2c_write(name, bus, address, cmd);
    }

    bool touch(Register reg) const {
        return i2c_write(name, bus, address, reg);
    }

    template <typename A>
    std::optional<A> read() const {
        A value;
        if (!i2c_read(name, bus, address, value)) return {};
        return value;
    }

    template <typename A>
    std::optional<A> read_crc() const {
        auto result = i2c_read_blocking_crc<CRC_Init, A>(name, bus, address);
        if (!result) return {};
        return get<0>(*result);
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

#define DEFINE_I2C_DEVICE_LOGGER(fn_name, prefix)                                               \
    void fn_name(const char* format, ...) const __attribute__((__format__(__printf__, 2, 3))) { \
        /* HACK: race-y, but whatever, stdio is race-y... */                                    \
        printf(prefix "%s [I2C%d 0x%02x] - ", (char const*)name, i2c_hw_index(&bus), address);  \
        va_list arglist;                                                                        \
        va_start(arglist, format);                                                              \
        vprintf(format, arglist);                                                               \
        va_end(arglist);                                                                        \
        puts(""); /* `puts` implicitly emits a newline after argument */                        \
    }

    DEFINE_I2C_DEVICE_LOGGER(log, "")
    DEFINE_I2C_DEVICE_LOGGER(log_warn, "WARN - ")
    DEFINE_I2C_DEVICE_LOGGER(log_error, "INFO - ")

#undef DEFINE_I2C_DEVICE_LOGGER
};

}  // namespace nevermore
