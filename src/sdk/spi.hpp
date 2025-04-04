#pragma once

#include "hardware/spi.h"
#include <cstdint>

namespace nevermore {

enum class SPI_Pin : uint8_t { RECV = 0, SELECT = 1, CLOCK = 2, SEND = 3 };

constexpr uint8_t spi_gpio_bus_num(uint8_t pin) {
    return (pin / 8) & 1u;
}

constexpr SPI_Pin spi_gpio_kind(uint8_t pin) {
    return SPI_Pin(pin & 0b11);  // it's two blocks of `[RX, CS, CLK, TX]`, then the next bus
}

inline spi_inst_t* spi_gpio_bus(uint8_t pin) {
    switch (spi_gpio_bus_num(pin)) {
    default: return nullptr;
    case 0: return spi0;  // NOLINT(cppcoreguidelines-pro-type-cstyle-cast)
    case 1: return spi1;  // NOLINT(cppcoreguidelines-pro-type-cstyle-cast)
    }
}

}  // namespace nevermore
