#pragma once

#include "config.hpp"
#include "hardware/platform_defs.h"
#include "sdk/i2c.hpp"
#include "sdk/pwm.hpp"
#include "sdk/spi.hpp"
#include "utility/container_misc.hpp"
#include <array>
#include <cassert>
#include <compare>
#include <cstdint>
#include <optional>
#include <span>

namespace nevermore {

constexpr uint32_t PIN_MAX = 30;

struct [[gnu::packed]] GPIO {
    static constexpr GPIO none() {
        return {};
    }

    // offset +1 b/c we need 0 to mean not-set (persisted storage initialises to 0)
    uint8_t gpio = 0;

    constexpr GPIO() = default;
    constexpr GPIO(uint8_t gpio) : gpio(gpio + 1) {
        assert(gpio < 0xFF);
        assert(validate() && !not_set());
    }

    // HACK: Compiler bug workaround - GCC 12.2.1
    // `gpio(gpio + 1)` instead of `gpio(gpio)` in `uint8_t -> GPIO` ctor seems
    // to cause the default copy ctor to fail to be instantiated, resulting in
    // the compile error:   `non-constant condition for static assertion ...
    //                       in 'constexpr' expansion of 'nevermore::GPIO()'`
    // Explicitly default declaring the copy ctor doesn't seem to help,
    // but explicitly defining it seems avoid the bug.
    // NOLINTNEXTLINE(modernize-use-equals-default)
    constexpr GPIO(GPIO const& x) : gpio(x.gpio) {}

    // INVARIANT: `not_set() => validate()`
    [[nodiscard]] constexpr bool validate() const {
        return gpio <= PIN_MAX;  // RP2040 doesn't support GPIOs above 29
    }

    [[nodiscard]] constexpr bool not_set() const {
        return gpio == 0;
    }

    explicit constexpr operator bool() const {
        return !not_set();
    }

    constexpr operator uint8_t() const {
        if (not_set()) throw "Config uses unset pin.";

        return gpio - 1;
    }

    [[nodiscard]] constexpr std::optional<uint8_t> i2c_bus_num() const {
        if (not_set()) return std::nullopt;
        return i2c_gpio_bus_num(*this);
    }

    [[nodiscard]] constexpr std::optional<uint8_t> spi_bus_num() const {
        if (not_set()) return std::nullopt;
        return spi_gpio_bus_num(*this);
    }

    constexpr std::partial_ordering operator<=>(GPIO const& rhs) const {
        if (not_set() != rhs.not_set()) return std::partial_ordering::unordered;
        return gpio <=> rhs.gpio;
    }

    constexpr bool operator!=(GPIO const&) const = default;
    constexpr bool operator==(GPIO const&) const = default;
};

// Don't ever allow these pins to be used.
constexpr std::array<GPIO, 2> PINS_RESERVED_UART{0, 1};

// Overkill b/c we want to reserve space for expansion.
// This is dumped straight into persisted data, which makes changing the layout
// a pain.
struct [[gnu::packed]] Pins {
    // Active pin assignments. *Do not use during static initialisation.*
    // INVARIANT: Constant after reading settings from flash.
    static Pins const& active();
    static bool setup(Pins const&);

    // For outliers who want multiple pins for a function.
    // Is 8 excessive? Stupidly so, and we won't have to extend it in the future.
    static constexpr size_t ALTERNATIVES_MAX = 8;

    using GPIOs = GPIO[ALTERNATIVES_MAX];

    // reserve for future expansion where you can use PIO-driven pins
    struct [[gnu::packed]] BusI2C {
        // generic buses can run faster, but we don't verify that it'll support all sensors.
        static constexpr uint32_t BAUD_RATE_MAX = 1'000'000;
        static constexpr uint32_t BAUD_RATE_GENERIC_MAX = BAUD_RATE_MAX;
        static constexpr uint32_t BAUD_RATE_SENSOR_MAX = I2C_BAUD_RATE_SENSOR_MAX;

        enum class Kind : uint8_t { generic = 0, intake = 1, exhaust = 2 };

        Kind kind = Kind::generic;
        GPIO clock;
        GPIO data;
        uint32_t baud_rate = kind == Kind::generic ? BAUD_RATE_GENERIC_MAX : BAUD_RATE_SENSOR_MAX;

        [[nodiscard]] explicit constexpr operator bool() const {
            return clock || data;
        }

        [[nodiscard]] constexpr std::optional<uint8_t> hardware_bus_num() const {
            if (!clock || !data) return {};                       // need both to map onto HW
            if (i2c_gpio_kind(clock) != I2C_Pin::SCL) return {};  // doesn't map onto a clock pin
            if (i2c_gpio_kind(data) != I2C_Pin::SDA) return {};   // doesn't map onto a data pin

            return Pins::hardware_bus_num(
                    [](GPIO const& p) { return p.i2c_bus_num(); }, std::array{clock, data});
        }
    };

    // reserve for future expansion where you can use PIO-driven pins
    struct [[gnu::packed]] BusSPI {
        static constexpr uint32_t BAUD_RATE_MAX = SYS_CLK_KHZ * 1000;
        enum class Kind : uint8_t { generic = 0, display = 1 };

        Kind kind;
        GPIO clock;
        GPIO send;
        GPIO recv;
        GPIO select;  // not used by anyone in practice
        uint32_t baud_rate = BAUD_RATE_MAX;

        [[nodiscard]] explicit constexpr operator bool() const {
            return clock || send || recv || select;
        }

        [[nodiscard]] constexpr std::optional<uint8_t> hardware_bus_num() const {
            // always need a clock pin
            if (!clock || spi_gpio_kind(clock) != SPI_Pin::CLOCK) return {};
            // send/recv though are optional, but if set must map to appropriate HW pins
            if (send && spi_gpio_kind(send) != SPI_Pin::SEND) return {};
            if (recv && spi_gpio_kind(recv) != SPI_Pin::RECV) return {};
            if (select && spi_gpio_kind(select) != SPI_Pin::SELECT) return {};

            return Pins::hardware_bus_num(
                    [](GPIO const& p) { return p.spi_bus_num(); }, std::array{clock, send, recv, select});
        }
    };

    BusI2C i2c[ALTERNATIVES_MAX]{};
    BusSPI spi[ALTERNATIVES_MAX]{};

    GPIOs fan_pwm{};             // mirrored
    GPIOs fan_tachometer{};      // summed in SW
    GPIOs neopixel_data{};       // reserve space, but disallow multiple pins
    GPIOs photocatalytic_pwm{};  // mirrored

    GPIO display_command;
    GPIO display_reset;
    GPIO display_brightness_pwm;
    GPIO touch_interrupt;
    GPIO touch_reset;

    // HACK: reserve for future expansion w/o disturbing peers in setting struct
    // +3 bytes since it's included with padding
    std::array<uint8_t, 35> unused_spare_space{};

    constexpr void validate_or_throw() const;

    constexpr void foreach_pwm_function(auto&& go) const {
        go(fan_pwm, true);
        go(photocatalytic_pwm, true);
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        go(std::span{&display_brightness_pwm, &display_brightness_pwm + 1}, true);
    }

    [[nodiscard]] constexpr bool pins_forall(auto&& go) const {
        auto apply = [&](auto&& xs) -> bool {
            if constexpr (std::is_same_v<GPIO, std::decay_t<decltype(xs)>>)
                return go(xs);
            else
                return std::ranges::all_of(xs, go);
        };

        for (auto&& bus : i2c) {
            if (!apply(bus.clock)) return false;
            if (!apply(bus.data)) return false;
        }

        for (auto&& bus : spi) {
            if (!apply(bus.clock)) return false;
            if (!apply(bus.send)) return false;
            if (!apply(bus.recv)) return false;
        }

        if (!apply(fan_pwm)) return false;
        if (!apply(fan_tachometer)) return false;
        if (!apply(neopixel_data)) return false;
        if (!apply(photocatalytic_pwm)) return false;

        if (!apply(display_brightness_pwm)) return false;
        if (!apply(display_command)) return false;
        if (!apply(display_reset)) return false;
        if (!apply(touch_interrupt)) return false;
        if (!apply(touch_reset)) return false;

        return true;
    }

    [[nodiscard]] constexpr bool pins_exist(auto&& go) const {
        return !pins_forall([&](auto const& p) { return !go(p); });
    }

    // just here for the `static_assert` call
    [[nodiscard]] constexpr bool validate_or_throw_() const {
        validate_or_throw();
        return true;
    }

private:
    [[nodiscard]] bool apply() const;

    [[nodiscard]] constexpr bool pins_distinct() const {
        uint32_t used = 0;
        return pins_forall([&](GPIO const& pin) {
            if (!pin || !pin.validate()) return true;
            if (used & (1u << pin)) return false;
            used |= 1u << pin;
            return true;
        });
    }

    [[nodiscard]] static constexpr std::optional<uint8_t> hardware_bus_num(
            std::optional<uint8_t> (*bus)(GPIO const&), std::span<GPIO const> pins) {
        std::optional<uint8_t> x;
        for (auto const& pin : pins)
            if (auto y = bus(pin)) {
                if (!x) x = y;
                if (x != y) return {};
            }

        return x;
    }

    static constexpr void validate_or_throw(BusI2C const& bus) {
        // no pins defined for bus -> not in use
        if (!bus) return;
        // clock & data are always required
        if (!bus.clock) throw "Config uses an I2C bus without a clock GPIO.";
        if (!bus.data) throw "Config uses an I2C bus without a data GPIO.";
        if (bus.baud_rate <= 0) throw "Config uses an I2C bus with a baud-rate <= 0.";
        if (BusI2C::BAUD_RATE_MAX < bus.baud_rate)
            throw "Config uses an SPI bus with a baud rate exceeding system max.";
        // currently HW bus is required. future work to support PIO buses
        if (!bus.hardware_bus_num()) throw "Config uses an I2C bus with GPIOs that cannot map to a HW bus.";

        switch (bus.kind) {
        case BusI2C::Kind::generic: {
            if (BusI2C::BAUD_RATE_GENERIC_MAX < bus.baud_rate)
                throw "Config uses an generic I2C bus with a baud rate exceeding generic max.";
        } break;
        case BusI2C::Kind::intake:  // FALL THRU: exhaust
        case BusI2C::Kind::exhaust: {
            if (BusI2C::BAUD_RATE_SENSOR_MAX < bus.baud_rate)
                throw "Config uses an I2C bus with a baud rate exceeding sensor max.";
        } break;
        }
    }

    static constexpr void validate_or_throw(BusSPI const& bus) {
        // no pins defined for bus -> not in use
        if (!bus) return;
        // clock is always required
        if (!bus.clock) throw "Config uses an SPI bus without a clock GPIO.";
        if (bus.baud_rate <= 0) throw "Config uses an I2C bus with a baud-rate <= 0.";
        if (BusSPI::BAUD_RATE_MAX < bus.baud_rate)
            throw "Config uses an SPI bus with a baud rate exceeding system max.";
        // currently HW bus is required. future work to support PIO buses
        if (!bus.hardware_bus_num()) throw "Config uses an SPI bus with GPIOs that cannot map to a HW bus.";

        using enum BusSPI::Kind;
        switch (bus.kind) {
        case generic: break;  // no requirements for generic bus
        case display: {
            if (!bus.send) throw "Config uses a display SPI bus without a MOSI GPIO.";
        } break;
        }
    }
};

static_assert(sizeof(Pins) == 200, "can't resize this w/o bumping `Settings` version");

}  // namespace nevermore

#ifndef NEVERMORE_BOARD_HEADER
#error "`NEVERMORE_BOARD_HEADER` not defined"
#endif

#include NEVERMORE_BOARD_HEADER  // IWYU pragma: keep

inline constexpr void nevermore::Pins::validate_or_throw() const {
    if (pins_exist(contains(PINS_RESERVED_UART))) throw "Config uses a GPIO reserved for UART (0, 1).";
    if (pins_exist(contains(PINS_RESERVED_BOARD))) throw "Config uses a GPIO not exposed on this board.";
    if (!pins_forall([](auto p) { return p.validate(); })) throw "Config uses an invalid GPIO (>= 30).";
    if (!pins_distinct()) throw "Config uses a GPIO more than once.";

    uint32_t i2c_hw_bus = 0;
    for (auto&& bus : i2c) {
        validate_or_throw(bus);
        if (!bus) continue;

        auto bus_num = bus.hardware_bus_num();
        if (!bus_num) throw "Config uses an I2C bus that cannot be mapped to a hardware bus.";
        if (i2c_hw_bus & (1u << *bus_num))
            throw "Config uses multiple I2C buses mapped to the same hardware bus.";

        i2c_hw_bus |= 1u << *bus_num;
    }

    uint32_t spi_hw_bus = 0;
    for (auto&& bus : spi) {
        validate_or_throw(bus);
        if (!bus) continue;

        auto bus_num = bus.hardware_bus_num();
        if (!bus_num) throw "Config uses an SPI bus that cannot be mapped to a hardware bus.";
        if (spi_hw_bus & (1u << *bus_num))
            throw "Config uses multiple SPI buses mapped to the same hardware bus.";

        spi_hw_bus |= 1u << *bus_num;
    }

    uint32_t display_buses = 0;
    for (auto&& bus : spi)
        display_buses += bus && bus.kind == BusSPI::Kind::display;
    if (1 < display_buses) throw "Config uses multiple display SPI buses.";

    uint32_t pwm_slice_claimed = 0;
    foreach_pwm_function([&](std::span<GPIO const> pins, bool allow_sharing) {
        for (auto&& pin : pins) {
            if (pin && pwm_slice_claimed & (1u << pwm_gpio_to_slice_num_(pin)))
                throw "Config uses a PWM slice for multiple non-shared functions.";
            if (pin && !allow_sharing) pwm_slice_claimed |= 1u << pwm_gpio_to_slice_num_(pin);
        }

        // claim it regardless of sharing, we don't share between functions
        for (auto&& pin : pins)
            if (pin) pwm_slice_claimed |= 1u << pwm_gpio_to_slice_num_(pin);
    });

    auto disallow_multiples = [](std::span<GPIO const> pins, char const* msg) {
        bool found = false;
        for (auto&& pin : pins) {
            if (pin && found) throw msg;
            found = !!pin;
        }
    };

    disallow_multiples(neopixel_data, "Config uses multiple NeoPixel data GPIOs.");
}
