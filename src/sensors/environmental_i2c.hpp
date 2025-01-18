#pragma once

#include "async_sensor.hpp"
#include "sdk/i2c.hpp"
#include "sensors/environmental.hpp"
#include "utility/i2c_device.hpp"

namespace nevermore::sensors {

template <typename Register, TemplateStringLiteral Name, CRC8_t CRC_Init = 0>
struct SensorPeriodicEnvI2C : SensorPeriodic {
    using I2CDevice = nevermore::I2CDevice<Register, Name, CRC_Init>;

    template <typename A>
    using ResponseCRC = typename I2CDevice::template ResponseCRC<A>;

    template <typename A>
    static constexpr CRC8_t crc(A x) {
        return I2CDevice::crc(std::move(x));
    }

    template <typename A>
    static bool crc(A const& x, CRC8_t expected) {
        auto x_crc = crc(x);
        if (x_crc == expected) return true;

        printf("ERR - %s - crc failed expected=0x%02x computed=0x%02x\n", static_cast<char const*>(Name),
                expected, x_crc);
        return false;
    }

    I2CDevice i2c;
    EnvironmentalFilter side;

    SensorPeriodicEnvI2C(I2C_Bus& bus, uint8_t address, EnvironmentalFilter side)
            : i2c{bus, address}, side(side) {}

    [[nodiscard]] char const* name() const final {
        return Name;
    }
};

}  // namespace nevermore::sensors
