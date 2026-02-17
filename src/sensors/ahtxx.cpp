#include "ahtxx.hpp"
#include "config.hpp"
#include "environmental_i2c.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/task.hpp"
#include "utility/numeric_suffixes.hpp"
#include <cstdint>
#include <optional>

using namespace std;
using namespace BLE;

// Constants, payloads, and timings were draw from Klipper's `aht10.py` module and the Linux kernel driver.

namespace nevermore::sensors {

static_assert(I2C_BAUD_RATE_SENSOR_MAX <= 400'000,
        "`config.hpp`'s `I2C_BAUD_RATE_SENSOR_MAX` is too high for AHTxx (max 400 kbit/s)");

namespace {

constexpr auto MEASURE_READ_RETRIES = 5u;

// Reportedly AHT10 can have a address of 0x39 in certain cases?
constexpr array ADDRESSES{0x38_u8, 0x39_u8};

// From AHT21 datasheet
// AHT10 doesn't say anything about a payload, but AHT20+ says to send `0x33 0x00`
constexpr array CMD_PAYLOAD_MEASURE{0x33_u8, 0x00_u8};
constexpr auto DELAY_MEASURE = 110ms;  // AHT21 spec says 80ms (and wait again if busy), Klipper does 110ms.
constexpr auto DELAY_RESET = 20ms;     // AHT10 and AHT20 spec says < 20ms
constexpr auto DELAY_KLIPPER_INIT = 100ms;

// ASAIR's AHT20 demo is crazy and is either broken or uses registers that no other driver seems to reference.
// That said, the datasheets for AHT10 and AHT20 both say to "initialise registers 0x1B, 0x1C, and 0x1E" when
// `Status != 0x18`, which I don't see *anyone* else doing, including the Linux driver.
enum class Reg : uint8_t {
    Status = 0x71,
    // DHT20 seems to be a minor variant of the AHT20, but it inits by a direct write to `Status?`
    // (See Linux driver.)
    // Init_DHT20 = 0x71,
    StartMeasurement = 0xAC,
    Reset = 0xBA,  // AHT10 doc only? Not mentioned in AHT20 or DHT20.
    Init_AHT2x = 0xBE,
    Init_AHT1x = 0xE1,
};

enum class Status : uint8_t {
    Unk1_OutOfThreshold = 1u << 2,  // *UNVERIFIED* src: comment in expressif's AHT20 driver
    Calibrated = 1u << 3,
    Unk5_CrcOkay = 1u << 4,  // *UNVERIFIED* src: comment in expressif's AHT20 driver
    ModeCyclic = 1u << 5,    // Self-issues measurements? No info in datasheet. Linux driver uses this.
    ModeCommand = 1u << 6,   // AHT20. Overrides Cyclic. (i.e. ignore cyclic if cmd is set)
    Busy = 1u << 7,
};

constexpr uint8_t operator&(Status a, Status b) {
    return uint8_t(a) & uint8_t(b);
}

// Inferring constant based on value. Spec says send 0x08 0x00.
// Linux driver also uses this, but punts it into Cyclic Mode by default, which I guess is a thing after all.
constexpr array CMD_PAYLOAD_INIT{(uint8_t)Status::Calibrated, 0x00_u8};

struct [[gnu::packed]] State {
    Status status;
    // fields are stored in big endian order
    uint8_t humidity0;
    uint8_t humidity1;
    // layout is `0bHHHH'TTTT`
    uint8_t temperature0 : 4;
    uint8_t humidity2 : 4;
    uint8_t temperature1;
    uint8_t temperature2;
};
static_assert(sizeof(State) == 6);

struct AHTxxSensor final : SensorPeriodicEnvI2C<Reg, "AHTxx"> {
    using SensorPeriodicEnvI2C::SensorPeriodicEnvI2C;

    bool setup() {  // NOLINT(readability-make-member-function-const)
        // 'AHT2x' (likely clones) devices have been found to happily init with AHT1x sequences.
        // Reportedly others have work w/ a fallback to AHT2x official init seq.
        // Honestly I have no idea what's going on here, and ASAIR's official docs are garbage.
        // Given that there's no device identification feature, we have little choice but to blindly
        // shove some init seqs into what we hope are the correct registers. Lovely.
        // FUTURE WORK: AHT30 reportedly requires no initialisation. Figure out how to fingerprint.
        if (!i2c.write(Reg::Init_AHT1x, CMD_PAYLOAD_INIT))
            if (!i2c.write(Reg::Init_AHT2x, CMD_PAYLOAD_INIT)) return false;

        task_delay<DELAY_KLIPPER_INIT>();
        return true;
    }

    bool reset() {  // NOLINT(readability-make-member-function-const)
        if (!i2c.touch(Reg::Reset)) return false;

        task_delay<DELAY_RESET>();
        return true;
    }

    void read() override {
        auto state = measure();
        if (!state) return;

        auto t_raw = state->temperature2 | (uint32_t(state->temperature1) << 8) |
                     (uint32_t(state->temperature0) << 16);
        auto h_raw =
                state->humidity2 | (uint32_t(state->humidity1) << 4) | (uint32_t(state->humidity0) << 12);
        auto t = (double(t_raw) / (1 << 20)) * 200 - 50;
        auto h = (double(h_raw) / (1 << 20)) * 100;

        auto _ = side.guard();
        side.set(Temperature(t));
        side.set(Humidity(clamp(h, 0., 100.)));
    }

    optional<State> measure() {
        if (i2c.write(Reg::StartMeasurement, CMD_PAYLOAD_MEASURE)) {
            for (unsigned i = MEASURE_READ_RETRIES; 0 < i; --i) {
                task_delay<DELAY_MEASURE>();

                // FUTURE WORK: AHT20+ has a CRC at the end. Verify it.
                auto result = i2c.read<State>();
                if (result && !(result->status & Status::Busy)) return result;
            }
        }

        reset();  // just try resetting the bloody thing...
        return {};
    }
};

}  // namespace

unique_ptr<SensorPeriodic> ahtxx(I2C_Bus& bus, EnvironmentalFilter side) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<AHTxxSensor>(bus, address, side); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensors
