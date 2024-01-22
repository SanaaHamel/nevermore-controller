#pragma once

#include "config/pins.hpp"
#include "hardware/pio.h"
#include "i2c.hpp"
#include <string>

namespace nevermore {

struct I2C_PIO final : I2C_Bus {  // NOLINT(cppcoreguidelines-special-member-functions)
    I2C_PIO(PIO, GPIO_Pin pin_sda, GPIO_Pin pin_scl);
    I2C_PIO(PIO, uint sm, GPIO_Pin pin_sda, GPIO_Pin pin_scl);

    [[nodiscard]] char const* name() const override;

protected:
    [[nodiscard]] int write(uint8_t addr, uint8_t const* src, size_t len) override;
    [[nodiscard]] int read(uint8_t addr, uint8_t* dst, size_t len) override;

private:
    PIO pio;
    uint sm;
    std::string _name;
};

}  // namespace nevermore
