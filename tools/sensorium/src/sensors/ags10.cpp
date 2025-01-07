#include "config.hpp"
#include "sgp40.hpp"
#include "utility/numeric_suffixes.hpp"
#include "utility/packed_tuple.hpp"
#include <bit>
#include <cstdint>

using namespace std;

namespace nevermore::sensorium::sensors {

// SGP40 supports std-mode 100 kbits/s and fast mode 400 kbits/s
static_assert(I2C_BAUD_RATE_SENSOR_MAX <= 15'000,
        "`config.hpp`'s `I2C_BAUD_RATE_SENSOR_MAX` is too high for SGP40 (max 400 kbit/s)");

namespace {

constexpr uint8_t ADDRESSES[]{0x1A};

// clangd bug: crash if `byteswap` is used w/o `std::` prefix in enum RHS.
// SGP40 wants its cmds in BE order
enum class Cmd : uint8_t {
    TVOC = 0x00,
    Calibration = 0x01,
    Version = 0x11,
    Resistance = 0x20,
    Address = 0x21,  // weird cmd: addr, addr-inv, addr, addr-inv, crc
};

struct VersionInfo {
    uint8_t reserved[3];
    uint8_t version;
};

struct AGS10 final : SensorI2C<Cmd, "AGS10", 0xFF> {
    using SensorI2C::SensorI2C;

    bool setup() {
        auto info = i2c.read_crc<VersionInfo, 30ms>(Cmd::Version);
        if (!info) return false;

        i2c.log("version %u", info->version);
        return true;
    }

    bool issue(EnvState const&) override {
        return i2c.touch(Cmd::Resistance);
    }

    pair<EnvState, optional<uint16_t>> readback() override {
        auto response = i2c.read_crc<uint32_t>();
        if (!response) return {};

        auto raw = byteswap(*response);
        i2c.log("full value %u", unsigned(raw));
        return {{}, raw & 0xFFFF};
    }
};

}  // namespace

unique_ptr<Sensor> ags10(Pins::BusI2C const& pins) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<AGS10>(pins, address); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensorium::sensors
