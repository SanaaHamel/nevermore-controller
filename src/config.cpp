#include "config.hpp"
#include "sdk/pwm.hpp"
#include <algorithm>
#include <array>
#include <cstdint>

static_assert(PIN_MAX == 30, "Told you not to alter this!");

using namespace std;

namespace {

// RP2040 has 30 GPIO pins, but the pico doesn't expose all of them.
constexpr array PINS_RESERVED_PICO_W{23, 24, 25, 29};

template <typename F>
constexpr bool pins_forall(F&& go) {
    if (!all_of(begin(PINS_I2C), end(PINS_I2C), go)) return false;
    if (!(go(PIN_FAN_PWM) && go(PIN_FAN_TACHOMETER))) return false;
    if (!go(PIN_NEOPIXEL_DATA_IN)) return false;

    return true;
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

constexpr bool all_pins_available() {
    return pins_forall([](GPIO_Pin pin) {
        return std::find(PINS_RESERVED_PICO_W.begin(), PINS_RESERVED_PICO_W.end(), pin) ==
               PINS_RESERVED_PICO_W.end();
    });
}

constexpr uint8_t i2c_bus_pins_defined(uint8_t bus) {
    uint8_t defined = 0;
    for (auto pin : PINS_I2C) {
        auto pin_bus = (pin / 2) & 1u;
        auto pin_kind = 1u << (pin % 2);
        if (pin_bus == bus) defined |= pin_kind;
    }

    return defined;
}

static_assert(all_pins_valid(), "`config.hpp` uses a GPIO pin outside of range [0, 29].");
static_assert(all_pins_unique(), "`config.hpp` uses duplicate pins. A pin can be used at most once.");
static_assert(
        all_pins_available(), "`config.hpp` uses a pin not exposed on the Pico W. This is likely a mistake.");

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

}  // namespace
