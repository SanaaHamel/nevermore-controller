#pragma once

#include "sdk/i2c.hpp"
#include <cstdint>
#include <optional>
#include <type_traits>

namespace nevermore {

template <typename Register>
    requires(std::is_scoped_enum_v<Register>)
struct I2CDevice {
    i2c_inst_t& bus;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    uint8_t address;

    template <typename A>
    bool write(Register reg, A value) const {
        if (1 != i2c_write_blocking(bus, address, reg)) return false;
        if (sizeof(A) != i2c_write_blocking(bus, address, value)) return false;
        return true;
    }

    template <typename A>
    std::optional<A> read(Register reg) const {
        A value;
        if (1 != i2c_write_blocking(bus, address, uint8_t(reg))) return {};
        if (sizeof(value) != i2c_read_blocking(bus, address, value)) return {};
        return value;
    }
};

}  // namespace nevermore
