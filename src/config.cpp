#include "config.hpp"
#include "sdk/gap.hpp"
#include "sdk/i2c.hpp"
#include "sdk/pwm.hpp"
#include "sdk/spi.hpp"
#include <array>
#include <bitset>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <ranges>

using namespace std;

static_assert(PIN_MAX == 30, "Told you not to alter this!");

static_assert(ADVERTISE_INTERVAL_MIN <= ADVERTISE_INTERVAL_MAX,
        "`config.hpp`'s `ADVERTISE_INTERVAL_MIN` must be <= `ADVERTISE_INTERVAL_MAX`");
static_assert(BT_ADVERTISEMENT_INTERVAL_MIN <= ADVERTISE_INTERVAL_MIN,
        "`config.hpp`'s `ADVERTISE_INTERVAL_MIN` is set too low. Minimum is 100ms.");

static_assert(
        (I2C_BAUD_RATE == 100 * 1000) || (I2C_BAUD_RATE == 400 * 1000) || (I2C_BAUD_RATE == 1000 * 1000),
        "`config.hpp`'s `I2C_BAUD_RATE` isn't a valid mode (100 kbit/s, 400 kbit/s, or 1000 kbit/s)");

namespace {

// RP2040 has 30 GPIO pins, but the pico doesn't expose all of them.
constexpr array PINS_RESERVED_PICO_W{23, 24, 25, 29};
// Don't ever allow these pins to be used.
constexpr array PINS_RESERVED_UART{0, 1};

template <typename Xs>
constexpr auto contains(Xs const& xs) {
    return [=](auto&& x) { return find(begin(xs), end(xs), x) != end(xs); };
}

constexpr bool all_pins_valid() {
    return pins_forall([](GPIO_Pin pin) { return pin < PIN_MAX; });
}

constexpr bool all_pins_unique() {
    uint32_t used = 0;
    return pins_forall([&](GPIO_Pin pin) {
        if (PIN_MAX <= pin) return true;  // ignore, `all_pins_valid` will pick up problem
        if (used & (1u << pin)) return false;
        used |= (1u << pin);
        return true;
    });
}

constexpr uint8_t i2c_bus_pins_defined(uint8_t bus) {
    uint8_t defined = 0;
    for (auto pin : PINS_I2C) {
        auto pin_bus = i2c_gpio_bus_num(pin);
        auto pin_kind = 1u << (pin % 2);
        if (pin_bus == bus) defined |= pin_kind;
    }

    return defined;
}

constexpr uint8_t spi_has_at_least(span<GPIO_Pin const> pins, initializer_list<SPI_Pin> required) {
    auto mask = [](SPI_Pin kind) { return 1 << uint8_t(kind); };
    uint8_t missing = 0;
    for (auto kind : required)
        missing |= mask(kind);
    for (auto pin : pins)
        missing &= ~mask(spi_gpio_kind(pin));

    return missing == 0;
}

static_assert(all_pins_valid(), "`config.hpp` uses a GPIO pin outside of range [0, 29].");
static_assert(all_pins_unique(), "`config.hpp` uses duplicate pins. A pin can be used at most once.");
static_assert(!pin_exists(contains(PINS_RESERVED_PICO_W)),
        "`config.hpp` uses a pin not exposed on the Pico W. This is likely a mistake.");
static_assert(!pin_exists(contains(PINS_RESERVED_UART)),
        "`config.hpp` uses pin 0 or pin 1. These are reserved for UART and cannot be used.");

static_assert(i2c_bus_pins_defined(0) & 0b01, "`config.hpp` has no pins defined for I2C0 SDA.");
static_assert(i2c_bus_pins_defined(0) & 0b10, "`config.hpp` has no pins defined for I2C0 SCL.");
static_assert(i2c_bus_pins_defined(1) & 0b01, "`config.hpp` has no pins defined for I2C1 SDA.");
static_assert(i2c_bus_pins_defined(1) & 0b10, "`config.hpp` has no pins defined for I2C1 SCL.");

// can't use the same slice to both drive a signal and read a signal
static_assert(pwm_gpio_to_slice_num_(PIN_FAN_PWM) != pwm_gpio_to_slice_num_(PIN_FAN_TACHOMETER),
        "`config.hpp` specifies `PIN_FAN_PWM` and `PIN_FAN_TACHOMETER` on the same PWM slice. "
        "They must be on separate slices.");
// PWM slice can only read from B channel
static_assert(pwm_gpio_to_channel_(PIN_FAN_TACHOMETER) == PWM_CHAN_B,
        "`config.hpp` specifies `PIN_FAN_TACHOMETER` on a A channel pin instead of a B channel pin . "
        "Move `PIN_FAN_TACHOMETER` to an odd # pin to fix this.");

static_assert(
        ranges::all_of(PINS_DISPLAY_SPI,
                [](auto pin) { return spi_gpio_bus_num(PINS_DISPLAY_SPI[0]) == spi_gpio_bus_num(pin); }),
        "`config.hpp`'s `PINS_DISPLAY_SPI` aren't all on the same bus");
static_assert(spi_has_at_least(PINS_DISPLAY_SPI, {SPI_Pin::CLOCK, SPI_Pin::SEND}),
        "`config.hpp`'s `PINS_DISPLAY_SPI` doesn't specify at least a clock and send pin");
}  // namespace
