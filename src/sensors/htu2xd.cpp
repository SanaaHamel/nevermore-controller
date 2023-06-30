#include "htu2xd.hpp"
#include "config.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/i2c.hpp"
#include "sdk/timer.hpp"
#include <bit>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <tuple>
#include <utility>

using namespace std;

// HTU21D supports "up to 400 kbits/s"
static_assert(
        I2C_BAUD_RATE <= 400 * 1000, "`config.hpp`'s `I2C_BAUD_RATE` is too high for SGP40 (max 400 kbit/s)");

namespace {

constexpr uint8_t HTU1xD_I2C_ADDRESS = 0x40;

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
constexpr double HTU2xD_HUMIDITY_COMPENSATION_COEFFICIENT = 25;

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

bool htu2xd_reset(i2c_inst_t& bus) {
    return 1 == i2c_write_blocking(bus, HTU1xD_I2C_ADDRESS, Cmd::SOFT_RESET, false);
}

// Once a measure is enqueued, call await to retrieve the value.
// You cannot interweave multiple measurements to the same device.
// Returns `false` on failure to enqueue.
bool htu2xd_issue(i2c_inst_t& bus, HTU2xD_Measure kind) {
    Cmd cmd;
    switch (kind) {
    case HTU2xD_Measure::Temperature: cmd = Cmd::MEASURE_TEMPERATURE_NON_BLOCKING; break;
    case HTU2xD_Measure::Humidity: cmd = Cmd::MEASURE_HUMIDITY_NON_BLOCKING; break;
    }

    if (auto r = i2c_write_blocking(bus, HTU1xD_I2C_ADDRESS, cmd); r < 0) {
        printf("HTU2xD failed to enqueue: %d\n", r);
        return false;
    }

    return true;
}

optional<tuple<HTU2xD_Measure, double>> htu2xd_read_compensated(
        i2c_inst_t& bus, double temperature = HTU2xD_HUMIDITY_COMPENSATION_ZERO_POINT) {
    // in either case we're waiting for the same kind of payload
    auto response = i2c_read_blocking_crc<0, uint16_t>(bus, HTU1xD_I2C_ADDRESS);
    if (!response) return {};
    auto const data = byteswap(get<0>(*response));

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
    i2c_inst_t& bus;               // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
    EnvironmentalSensorData data;  // tiny bit wasteful, but terser to manage

    HTU2xDSensor(i2c_inst_t& bus, EnvironmentalSensorData data) : bus(bus), data(std::move(data)) {}

    [[nodiscard]] char const* name() const override {
        return "HTU2xD";
    }

    // TODO: This is a multi-query sensor: we can't batch read issues and then batch read-backs
    //       Ideally we'd like to be able to chain issue/read pairs.
    //       For now, just bite the bullet, we're spending ~66ms blocked.
    void read() override {
        auto htu2xd_fetch = [&](auto kind, auto delay) {
            [[maybe_unused]] auto&& [temperature, humidity, pressure, voc_index] = data;

            htu2xd_issue(bus, kind);
            busy_wait(delay);  // must busy wait, can't sleep inside an interrupt handler

            // the sensor could return either data. take what we can get.
            auto response = htu2xd_read_compensated(
                    bus, temperature.value_or(HTU2xD_HUMIDITY_COMPENSATION_ZERO_POINT));
            if (!response) return;

            auto [response_kind, value] = *response;
            assert(kind == response_kind && "htu2xd_fetch - response kind mismatch");
            switch (response_kind) {
            case HTU2xD_Measure::Temperature: temperature = value; break;
            case HTU2xD_Measure::Humidity: humidity = value; break;
            }
        };

        htu2xd_fetch(HTU2xD_Measure::Temperature, HTU2xD_MEASURE_TEMPERATURE_DELAY);
        htu2xd_fetch(HTU2xD_Measure::Humidity, HTU2xD_MEASURE_HUMIDITY_DELAY);
    }
};

bool htu2xd_exists(i2c_inst_t& bus) {
    return htu2xd_reset(bus);
}

}  // namespace

unique_ptr<SensorPeriodic> htu2xd(i2c_inst_t& bus, EnvironmentalSensorData state) {
    if (!htu2xd_exists(bus)) return {};  // nothing found

    return make_unique<HTU2xDSensor>(bus, state);
}
