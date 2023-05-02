#include "pico/binary_info.h"  // IWYU pragma: keep
#include "config.hpp"
#include "sdk/i2c.hpp"

namespace {

constexpr GPIO_Pin first_i2c_pin(uint8_t bus, I2C_Pin kind) {
    for (auto pin : PINS_I2C) {
        if (i2c_gpio_bus_num(pin) != bus) continue;
        if (i2c_gpio_kind(pin) != kind) continue;
        return pin;
    }

    return PIN_MAX;
}

}  // namespace

bi_decl(bi_program_description("Nevermore Controller"));
bi_decl(bi_program_url("https://github.com/SanaaHamel/nevermore-controller"));

// clang-format off
bi_decl(bi_4pins_with_names(
    first_i2c_pin(0, I2C_Pin::SDA), "Intake I2C SDA",
    first_i2c_pin(0, I2C_Pin::SCL), "Intake I2C SCL",
    first_i2c_pin(1, I2C_Pin::SDA), "Exhaust I2C SDA",
    first_i2c_pin(1, I2C_Pin::SCL), "Exhaust I2C SCL"));

bi_decl(bi_2pins_with_names(
    PIN_FAN_PWM         , "Fan PWM",
    PIN_FAN_TACHOMETER  , "Fan Tachometer"));

bi_decl(bi_1pin_with_name(
    PIN_NEOPIXEL_DATA_IN, "NeoPixel Data In"));
// clang-format on
