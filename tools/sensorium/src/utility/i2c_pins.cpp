#include "i2c_pins.hpp"
#include "config/pins.hpp"
#include <cstdint>
#include <iterator>
#include <ranges>

using namespace std;

namespace nevermore::sensorium {

std::vector<Pins::BusI2C> i2c_pins() {
    auto reserved = [](GPIO const& pin) {
        return !pin || ranges::find(PINS_RESERVED_BOARD, pin) != end(PINS_RESERVED_BOARD);
    };

    std::vector<Pins::BusI2C> pins;
    for (uint8_t pin = 2; pin < PIN_MAX - 1; pin += 2) {
        if (reserved(pin) || reserved(pin + 1)) continue;

        pins.push_back(Pins::BusI2C{
                .clock = pin + 1,
                .data = pin,
                .baud_rate = I2C_BAUD_RATE_SENSOR_MAX,
        });
    }

    return pins;
}

}  // namespace nevermore::sensorium
