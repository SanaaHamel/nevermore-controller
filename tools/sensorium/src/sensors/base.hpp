
#pragma once

#include "config/pins.hpp"
#include "hardware/gpio.h"
#include "sdk/i2c_hw.hpp"
#include "sensors.hpp"
#include "utility/crc.hpp"
#include "utility/i2c_device.hpp"
#include "utility/scope_guard.hpp"
#include "utility/template_string_literal.hpp"
#include <cstdio>
#include <utility>

namespace nevermore::sensorium::sensors {

struct Sensor {
    static auto using_(Pins::BusI2C const& pins, auto&& go) {
        gpio_set_function(pins.clock, GPIO_FUNC_I2C);
        gpio_set_function(pins.data, GPIO_FUNC_I2C);
        SCOPE_GUARD {
            gpio_set_function(pins.clock, GPIO_FUNC_NULL);
            gpio_set_function(pins.data, GPIO_FUNC_NULL);
        };

        return go();
    }

    Sensor(Pins::BusI2C pins) : pins(std::move(pins)) {}
    virtual ~Sensor() = default;

    virtual bool issue(EnvState const&) = 0;
    [[nodiscard]] virtual std::pair<EnvState, std::optional<uint16_t>> readback() = 0;
    [[nodiscard]] virtual char const* name() const = 0;

    virtual void log_reading(uint16_t x) = 0;

    template <typename A>
    void using_(A&& go) {
        return using_(pins, std::forward<A>(go));
    }

protected:
    // NOLINTNEXTLINE(cppcoreguidelines-non-private-member-variables-in-classes)
    Pins::BusI2C pins;
};

template <typename Register, TemplateStringLiteral Name, CRC8_t CRC_Init = 0>
struct SensorI2C : Sensor {
    using I2CDevice = nevermore::I2CDevice<Register, Name, CRC_Init>;

    template <typename A>
    static constexpr CRC8_t crc(A x) {
        return I2CDevice::crc(std::move(x));
    }

    template <typename A>
    bool crc(A const& x, CRC8_t expected) {
        auto x_crc = crc(x);
        if (x_crc == expected) return true;

        i2c.log_error("crc failed expected=0x%02x computed=0x%02x\n", expected, x_crc);
        return false;
    }

    I2CDevice i2c;

    SensorI2C(Pins::BusI2C const& pins, uint8_t address)
            : Sensor(pins), i2c{nevermore::i2c.at(*pins.hardware_bus_num()), address} {}

    [[nodiscard]] char const* name() const final {
        return Name;
    }

    void log_reading(uint16_t x) final {
        printf("SENSORIUM READBACK value=%05u clk=%02u dat=%02u addr=0x%02x name=%s\n", unsigned(x),
                unsigned(pins.clock), unsigned(pins.data), unsigned(i2c.address), name());
    }
};

}  // namespace nevermore::sensorium::sensors
