#include "i2c_pio.hpp"
#include "config.hpp"
#include "hardware/pio.h"
#include "lib/pio_i2c.h"
#include "pio_i2c.pio.h"
#include "sdk/i2c.hpp"
#include "utility/format.hpp"

namespace nevermore {

I2C_PIO::I2C_PIO(PIO pio, GPIO_Pin pin_sda, GPIO_Pin pin_scl)
        : I2C_PIO(pio, pio_claim_unused_sm(pio, true), pin_sda, pin_scl) {}

I2C_PIO::I2C_PIO(PIO pio, uint sm, GPIO_Pin pin_sda, GPIO_Pin pin_scl)
        : pio(pio), sm(sm), _name(format_string("PIO-%02d/%02d", pin_sda, pin_scl)) {
    uint offset = pio_add_program(pio, &i2c_program);
    i2c_program_init(I2C_BAUD_RATE, pio, sm, offset, pin_sda, pin_scl);
}

int I2C_PIO::write(uint8_t addr, uint8_t const* src, size_t len) {
    return pio_i2c_write_timeout_us(pio, sm, addr, src, len, I2C_TIMEOUT_US);
}

int I2C_PIO::read(uint8_t addr, uint8_t* dst, size_t len) {
    return pio_i2c_read_timeout_us(pio, sm, addr, dst, len, I2C_TIMEOUT_US);
}

char const* I2C_PIO::name() const {
    return _name.c_str();
}

}  // namespace nevermore
