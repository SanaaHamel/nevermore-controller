#include "htu2xd.hpp"
#include "config.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/i2c.hpp"
#include "sdk/task.hpp"
#include <bit>
#include <cassert>
#include <cstdint>
#include <tuple>
#include <utility>

using namespace std;
using namespace BLE;

namespace nevermore::sensors {

// HTU21D supports "up to 400 kbits/s"
static_assert(I2C_BAUD_RATE_SENSOR_MAX <= 400'000,
        "`config.hpp`'s `I2C_BAUD_RATE_SENSOR_MAX` is too high for HTU2x (max 400 kbit/s)");

namespace {

constexpr uint8_t HTU2xD_I2C_ADDRESS = 0x40;

enum class Cmd : uint8_t {
    MEASURE_TEMPERATURE_BLOCKING = 0xE3,
    MEASURE_HUMIDITY_BLOCKING = 0xE5,
    MEASURE_TEMPERATURE_NON_BLOCKING = 0xF3,
    MEASURE_HUMIDITY_NON_BLOCKING = 0xF5,
    REGISTER_READ = 0xE6,
    REGISTER_WRITE = 0xE7,
    SOFT_RESET = 0xFE,
};

constexpr double HTU2xD_HUMIDITY_COMPENSATION_ZERO_POINT = 25;
constexpr double HTU2xD_HUMIDITY_COMPENSATION_COEFFICIENT = -0.15;

// 50ms @ 14 bits, 25ms @ 13 bits, 13ms @ 12 bits, 7ms @ 11 bits
constexpr auto HTU2xD_MEASURE_TEMPERATURE_DELAY = 50ms;
// 16ms @ 12 bits,  8ms @ 11 bits,  5ms @ 10 bits, 3ms @  8 bits
constexpr auto HTU2xD_MEASURE_HUMIDITY_DELAY = 16ms;

enum class HTU2xD_Measure { Humidity, Temperature };

template <typename A>
constexpr CRC8_t htu2xd_crc(A&& x) {
    return crc8(std::forward<A>(x), 0);
}

// verify spec-provided test vectors
static_assert(0x79 == htu2xd_crc(span{array<uint8_t const, 1>{0xDC}}));
static_assert(0x7C == htu2xd_crc(span{array<uint8_t const, 2>{0x68, 0x3A}}));
static_assert(0x6B == htu2xd_crc(span{array<uint8_t const, 2>{0x4E, 0x85}}));

double htu2xd_humidity_compensated(double humidity_uncompensated, double temperature) {
    auto bias = (HTU2xD_HUMIDITY_COMPENSATION_ZERO_POINT - temperature) *
                HTU2xD_HUMIDITY_COMPENSATION_COEFFICIENT;
    return clamp(humidity_uncompensated + bias, 0., 100.);
}

bool htu2xd_reset(I2C_Bus& bus) {
    return bus.write("HTU2xD", HTU2xD_I2C_ADDRESS, Cmd::SOFT_RESET);
}

// Once a measure is enqueued, call await to retrieve the value.
// You cannot interweave multiple measurements to the same device.
// Returns `false` on failure to enqueue.
bool htu2xd_issue(I2C_Bus& bus, HTU2xD_Measure kind) {
    Cmd cmd;
    switch (kind) {
    case HTU2xD_Measure::Temperature: cmd = Cmd::MEASURE_TEMPERATURE_NON_BLOCKING; break;
    case HTU2xD_Measure::Humidity: cmd = Cmd::MEASURE_HUMIDITY_NON_BLOCKING; break;
    }

    return bus.write("HTU2xD", HTU2xD_I2C_ADDRESS, cmd);
}

optional<tuple<HTU2xD_Measure, double>> htu2xd_read_compensated(
        I2C_Bus& bus, double temperature = HTU2xD_HUMIDITY_COMPENSATION_ZERO_POINT) {
    // in either case we're waiting for the same kind of payload
    auto response = bus.read_crc<0, uint16_t>("HTU2xD", HTU2xD_I2C_ADDRESS);
    if (!response) return {};
    auto const data = byteswap(*response);

    [[maybe_unused]] auto const reserved_flag = (data & 0b01) == 0b01;  // should be zero
    auto const is_humidity = (data & 0b10) == 0b10;
    auto const datum_f = (data & ~0b11) / double(1 << 16);

    // printf("HTU2xD read-back: 0x%04x [0x%02x]\n", int(data), int(result.checksum));
    // printf("HTU2xD status: kind=%s reserved=%c (expected 0)\n", is_humidity ? "HUM" : "TEMP",
    //         reserved_flag ? '1' : '0');
    // printf("HTU2xD value=%f\n", is_humidity ? (-6 + 125 * datum_f) : (-46.85 + 175.72 * datum_f));

    if (is_humidity) {
        // printf("HTY2xD temp = %f\n", temperature);
        // printf("HTU2xD compensated = %f\n", htu2xd_humidity_compensated(-6 + 125 * datum_f, temperature));
        return tuple{HTU2xD_Measure::Humidity, htu2xd_humidity_compensated(-6 + 125 * datum_f, temperature)};
    }

    return tuple{HTU2xD_Measure::Temperature, -46.85 + 175.72 * datum_f};
}

struct HTU2xDSensor final : SensorPeriodic {
    I2C_Bus& bus;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    EnvironmentalFilter side;

    HTU2xDSensor(I2C_Bus& bus, EnvironmentalFilter side) : bus(bus), side(side) {}

    [[nodiscard]] char const* name() const override {
        return "HTU2xD";
    }

    void read() override {
        auto htu2xd_fetch = [&](auto kind, TaskDelayArg delay) {
            if (!htu2xd_issue(bus, kind)) return;
            task_delay(delay);

            // the sensor could return either data. take what we can get.
            auto response = htu2xd_read_compensated(
                    bus, side.get<Temperature>().value_or(HTU2xD_HUMIDITY_COMPENSATION_ZERO_POINT));
            if (!response) return;

            auto [response_kind, value] = *response;
            assert(kind == response_kind && "htu2xd_fetch - response kind mismatch");
            switch (response_kind) {
            case HTU2xD_Measure::Temperature: side.set(Temperature(value)); break;
            case HTU2xD_Measure::Humidity: side.set(Humidity(value)); break;
            }
        };

        htu2xd_fetch(HTU2xD_Measure::Temperature, HTU2xD_MEASURE_TEMPERATURE_DELAY);
        htu2xd_fetch(HTU2xD_Measure::Humidity, HTU2xD_MEASURE_HUMIDITY_DELAY);
    }
};

bool htu2xd_exists(I2C_Bus& bus) {
    return htu2xd_reset(bus);
}

}  // namespace

unique_ptr<SensorPeriodic> htu2xd(I2C_Bus& bus, EnvironmentalFilter side) {
    if (!htu2xd_exists(bus)) return {};  // nothing found

    return make_unique<HTU2xDSensor>(bus, side);
}

}  // namespace nevermore::sensors
