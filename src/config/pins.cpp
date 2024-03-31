#include "pins.hpp"
#include "config.hpp"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include <cassert>
#include <cstdint>

#ifndef NDEBUG
#include "utility/square_wave.hpp"
#endif

namespace nevermore {

namespace {
// place in BSS w/ zero init.
// TODO: Does the loader on the pico zero out the BSS before transfering control to CRT?
// Either way this is at best an assert aid.
bool g_pins_assigned = false;
Pins g_pins;

void bind_logical(GPIO const& gpio, bool dir) {
    if (gpio) {
        gpio_set_dir(gpio, dir);
        if (dir == GPIO_OUT) gpio_put(gpio, false);
        gpio_set_function(gpio, GPIO_FUNC_SIO);
    }
};

void bind_bus(auto const& xs, auto&& using_hw, auto&& using_pio) {
    uint32_t hw_used = 0;
    for (auto&& bus : xs) {
        if (!bus) continue;

        if (auto hw = bus.hardware_bus_num(); hw && !(hw_used & (1u << *hw))) {
            hw_used |= 1u << *hw;
            using_hw(bus, *hw);
        } else
            using_pio(bus);
    }
}

void bind_i2c_hw(Pins::BusI2C const& bus, uint8_t const hw) {
    gpio_set_function(bus.clock, GPIO_FUNC_I2C);
    gpio_set_function(bus.data, GPIO_FUNC_I2C);
    gpio_pull_up(bus.clock);
    gpio_pull_up(bus.data);

    char const* kind = "";
    switch (bus.kind) {
    case Pins::BusI2C::Kind::generic: kind = "generic"; break;
    case Pins::BusI2C::Kind::intake: kind = "intake"; break;
    case Pins::BusI2C::Kind::exhaust: kind = "exhaust"; break;
    }

    assert(hw <= 1);
    auto* i2c_inst = hw == 0 ? i2c0 : i2c1;
    printf("I2C%u (%s) running at %u baud/s (requested %u baud/s)\n", hw, kind,
            i2c_init(i2c_inst, bus.baud_rate), unsigned(bus.baud_rate));
}

void bind_spi_hw(Pins::BusSPI const& bus, uint8_t const hw) {
    gpio_set_function(bus.clock, GPIO_FUNC_SPI);
    if (bus.send) gpio_set_function(bus.send, GPIO_FUNC_SPI);
    if (bus.recv) gpio_set_function(bus.recv, GPIO_FUNC_SPI);
    if (bus.select) gpio_set_function(bus.select, GPIO_FUNC_SPI);

    char const* kind = "";
    switch (bus.kind) {
    case Pins::BusSPI::Kind::generic: kind = "generic"; break;
    case Pins::BusSPI::Kind::display: kind = "display"; break;
    }

    assert(hw <= 1);
    auto* spi = hw == 0 ? spi0 : spi1;
    printf("SPI%u (%s) running at %u baud/s (requested %u baud/s)\n", hw, kind, spi_init(spi, bus.baud_rate),
            unsigned(bus.baud_rate));
}

void bind_i2c_pio(Pins::BusI2C const&) {
    assert(false && "PIO I2C not implemented");
}

void bind_spi_pio(Pins::BusSPI const&) {
    assert(false && "PIO SPI not implemented");
}

}  // namespace

Pins const& Pins::active() {
    assert(g_pins_assigned);
    return g_pins;
}

bool Pins::setup(Pins const& pins) {
    assert(!g_pins_assigned);
    if (!pins.apply()) return false;

    g_pins_assigned = true;
    g_pins = pins;
    return true;
}

bool Pins::apply() const {
    try {
        validate_or_throw();
    } catch (char const* const) {
        return false;
    }

    bind_bus(i2c, bind_i2c_hw, bind_i2c_pio);
    bind_bus(spi, bind_spi_hw, bind_spi_pio);

    foreach_pwm_function([](auto&& pins, bool) {
        for (auto&& pin : pins)
            if (pin) gpio_set_function(pin, GPIO_FUNC_PWM);
    });

    for (auto&& pin : fan_tachometer)
        if (pin) gpio_pull_up(pin);

    // we're setting up the WS2812 controller on PIO0
    for (auto&& pin : neopixel_data)
        if (pin) gpio_set_function(pin, GPIO_FUNC_PIO0);

    bind_logical(display_command, GPIO_OUT);
    bind_logical(display_reset, GPIO_OUT);
    bind_logical(touch_interrupt, GPIO_IN);
    bind_logical(touch_reset, GPIO_OUT);

#if defined(PICO_DEFAULT_LED_PIN)
    bind_logical(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#elif defined(PICO_DEFAULT_WS2812_PIN)
    // FUTURE WORK: implement WS2812 LED
#endif

#ifndef NDEBUG
    if (auto pin = square_wave_pwm_first_available_pin())
        square_wave_pwm_init(pin, 30);
    else
        printf("!! No available PWM slice for square wave generator.\n");
#endif

    return true;
}

}  // namespace nevermore
