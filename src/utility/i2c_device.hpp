#pragma once

#include "sdk/i2c.hpp"
#include "utility/template_string_literal.hpp"
#include <cstdint>
#include <optional>
#include <type_traits>

namespace nevermore {

template <typename Register, TemplateStringLiteral name>
    requires(std::is_scoped_enum_v<Register>)
struct I2CDevice {
    i2c_inst_t& bus;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    uint8_t address;

    template <typename A>
    bool write(Register reg, A value) const {
        uint8_t cmd[sizeof(reg) + sizeof(value)];
        memcpy(cmd, &reg, sizeof(reg));
        memcpy(cmd + sizeof(reg), &value, sizeof(value));
        return i2c_write(name, bus, address, cmd);
    }

    template <typename A>
    std::optional<A> read() const {
        A value;
        if (!i2c_read(name, bus, address, value)) return {};
        return value;
    }

    template <typename A>
    std::optional<A> read(Register reg) const {
        if (!i2c_write(name, bus, address, reg)) return {};
        return read<A>();
    }
};

}  // namespace nevermore
