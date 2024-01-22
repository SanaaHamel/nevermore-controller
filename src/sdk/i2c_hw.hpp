#pragma once

#include "hardware/i2c.h"
#include "i2c.hpp"

namespace nevermore {

struct I2C_HW final : I2C_Bus {  // NOLINT(cppcoreguidelines-special-member-functions)
    i2c_inst_t& i2c;
    I2C_HW(i2c_inst_t& i2c) : i2c(i2c){};

    [[nodiscard]] char const* name() const override {
        switch (i2c_hw_index(&i2c)) {
        case 0: return "I2C0";
        case 1: return "I2C1";
        default: assert(false && "unhandled"); return "";
        }
    }

protected:
    [[nodiscard]] int write(uint8_t addr, uint8_t const* src, size_t len) override {
        return i2c_write_timeout_us(&i2c, addr, src, len, false, I2C_TIMEOUT_US);
    }

    [[nodiscard]] int read(uint8_t addr, uint8_t* dest, size_t len) override {
        return i2c_read_timeout_us(&i2c, addr, dest, len, false, I2C_TIMEOUT_US);
    }
};

extern std::array<I2C_HW, 2> i2c;

}  // namespace nevermore
