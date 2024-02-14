#include "sgp30.hpp"
#include "config.hpp"
#include "environmental_i2c.hpp"
#include "sensors/gas_index.hpp"
#include "utility/crc.hpp"
#include "utility/humidity.hpp"
#include "utility/numeric_suffixes.hpp"
#include <chrono>
#include <cstdint>
#include <optional>

using namespace std;
using namespace BLE;

// Following code uses Sensirion's SGP library as a reference
// https://github.com/Sensirion/embedded-sgp/blob/master/sgp30/sgp30.c

namespace nevermore::sensors {

static_assert(
        I2C_BAUD_RATE <= 400'000, "`config.hpp`'s `I2C_BAUD_RATE` is too high for SGP30 (max 400 kbit/s)");

namespace {

// Based on German federal standards (DOI 10.1007/s00103-007-0290-y)
constexpr uint16_t TVOC_UG_PER_M3_CLEAN_INDOORS = 300;

constexpr array ADDRESSES{0x58_u8};

constexpr uint8_t SGP30_PRODUCT_TYPE = 0;
constexpr uint8_t SGP30_VERSION_MIN = 0x20;
// Weird 1: Seems like only 0x20 and 0x22 are in the wild?
// Weird 2: Reference lib from mfg looks for 0x21 (33), but spec says 0x22 (34)...
[[maybe_unused]] constexpr uint8_t SGP30_VERSION_TVOC_EXTENDED_SUPPORT = 0x22;
constexpr uint16_t SGP30_SELF_TEST_OK = 0x00D4;  // byte-swapped

constexpr auto IAQ_STARTUP_DURATION = 15s;

// NB: byte-swapped b/c this device works in big endian
enum class Reg : uint16_t {
    // args are always uint16_t followed by CRC
    // Name = Code,             // [ min-delay, max-delay), in-args, out-args
    SerialID = 0x8236u,          // [ 0.5ms,  1ms), 0, 3
    FeatureSet = 0x2F20u,        // [  1ms,   2ms), 0, 2
    SelfTest = 0x3220u,          // [200ms, 220ms), 0, 1
    RawMeasure = 0x5020u,        // [ 20ms,  25ms), 0, 2
    HumiditySet = 0x6120u,       // [  1ms,  10ms), 1, 0
    IAQ_Init = 0x0320u,          // [  2ms,  10ms), 0, 0
    IAQ_Measure = 0x0820u,       // [ 10ms,  12ms), 0, 2
    IAQ_Baseline = 0x1520u,      // [  1ms,  10ms), 0, 2
    IAQ_BaselineSet = 0x1E20u,   // [  1ms,  10ms), 2, 0; reverse order from `IAQ_Baseline` -> TVOC, co2
    TVOC_Baseline = 0xB320u,     // [  1ms,  10ms), 1, 0
    TVOC_BaselineSet = 0x7720u,  // [  1ms,  10ms), 0, 1
};

using Version = uint8_t;

struct [[gnu::packed]] FeatureSet {
    uint8_t product_type : 4;  // expected to be 0
    uint8_t _reserved : 3;
    uint8_t _should_be_zero : 1;
    Version product_version;
};
static_assert(sizeof(FeatureSet) == sizeof(uint16_t));

struct [[gnu::packed]] Measurement {
    uint16_t co2_eq_ppm;
    CRC8_t co2_crc;
    uint16_t tvoc_ppb;
    CRC8_t tvoc_crc;
};
static_assert(sizeof(Measurement) == 6);

struct SGP30Sensor final : SensorPeriodicEnvI2C<Reg, "SGP30", 0xFF> {
    using Clock = chrono::steady_clock;

    static_assert(crc(0xEFBE_u16) == 0x92);

    using SensorPeriodicEnvI2C::SensorPeriodicEnvI2C;

    GasIndex index;
    Clock::time_point start = {};
    Version version = 0;

    bool setup() {
        auto features = feature_set();
        if (!features) return false;

        if (features->product_type != SGP30_PRODUCT_TYPE) {
            i2c.log_error("unrecognised product type %u", features->product_type);
            return false;
        }

        if (features->product_version < SGP30_VERSION_MIN) {
            i2c.log_error(
                    "unsupported version %u (minimum is %u)", features->product_version, SGP30_VERSION_MIN);
            return false;
        }

        version = features->product_version;  // stash this for future reference
        i2c.log("product=%u version=%u", features->product_type, features->product_version);

        if (!self_test()) return false;

        if (!i2c.touch(Reg::IAQ_Init)) {
            i2c.log_error("failed to init IAQ");
            return false;
        }
        task_delay(10ms);  // spec says 10ms for IAQ init
        start = Clock::now();

        reset_calibration();
        index.restore(side.voc_calibration_blob(), i2c);

        return true;
    }

    void reset_calibration() override {
        index = {GasIndexAlgorithm_ALGORITHM_TYPE_VOC};
        // default VOC is 20'000, which is lower than how this sensor reacts
        index.gia.mSraw_Minimum = 10'000;
    }

    void read() override {
        // NB: clamp to a minimum rel humidity b/c 0% disables humidity compensation
        constexpr float MIN_REL_HUMIDITY = 0.25f;  // 0.25% is all we need for a lower bound.
        // compute abs humidity in float, double could get pointlessly expensive...
        auto abs_humidity_g_m3 =
                humidity::absolute_fast(max(MIN_REL_HUMIDITY, (float)side.compensation_humidity()),
                        (float)side.compensation_temperature());
        if (!humidity_absolute_set(uint32_t(abs_humidity_g_m3 * 1000))) {
            i2c.log_error("failed to set humidity compensation");
            return;
        }

        auto raw = measure(Reg::RawMeasure, 25ms);
        if (!raw) return;

        side.set(VOCRaw(raw->tvoc_ppb));
        if (side.was_voc_breakdown_measurement()) return;

        side.set(GIAState(index.gia));
        side.set(index.process(raw->tvoc_ppb));
        index.checkpoint(side.voc_calibration_blob(), i2c);

        /*
        auto baseline = measure(Reg::IAQ_Baseline, 10ms);  // spec says 10ms
        auto result = measure(Reg::IAQ_Measure, 12ms);  // spec says 12ms max wait
        if (!(baseline && result)) return;

        auto idx = voc_index(*result);
        i2c.log("baseline - co^2-eq=%6d tvoc-ppb=%6d", baseline->co2_eq_ppm, baseline->tvoc_ppb);
        i2c.log("measure  - co^2-eq=%6d tvoc-ppb=%6d", result->co2_eq_ppm, result->tvoc_ppb);
        i2c.log("raw value-     H^2=%6d  ethanol=%6d", raw->co2_eq_ppm, raw->tvoc_ppb);
        i2c.log("index    -    chip=%6d    sgp40=%6d", int(idx.raw_value), int(gas_index));
        */
    }

    // not used. we're using the software VOC index library instead.
    [[nodiscard]] VOCIndex voc_index(Measurement const& result) const {
        if (Clock::now() - start < IAQ_STARTUP_DURATION) return VOCIndex::not_known_value;

        // FIXME: Needs tuning.
        constexpr float M_g = 110;     // mean molar mas of gas mix, value drawn from Mølhave et al., g / mol
        constexpr float V_m = 0.0244;  // m^3 / mol
        float tvoc_ug_per_m3 = (M_g / (V_m * 1000)) * result.tvoc_ppb;
        auto voc_index = lerp(0.f, 100.f, tvoc_ug_per_m3 / float(TVOC_UG_PER_M3_CLEAN_INDOORS));
        return clamp(voc_index, 1.f, 500.f);
    }

    [[nodiscard]] optional<FeatureSet> feature_set() const {
        return i2c.read_crc<FeatureSet>(Reg::FeatureSet, 10ms);  // spec says max 2ms
    }

    [[nodiscard]] optional<uint16_t> self_test() const {
        auto result = i2c.read_crc<uint16_t>(Reg::SelfTest, 220ms);
        if (!result) {
            i2c.log_error("self-test request failed");
        } else if (*result != SGP30_SELF_TEST_OK) {
            i2c.log_error("self-test failed, result 0x%04x (expected 0x%04x)", *result, SGP30_SELF_TEST_OK);
        }

        return result;
    }

    // PRECONDITION: `co2_eq_ppm` and `tvoc_ppb` populated. CRC is computed for you.
    bool baseline_set(Measurement const& m) {
        // NB: IAQ_BaselineSet expects [TVOC, CO2] instead of the usual [CO2, TVOC].
        i2c.log("set baseline co2-eq-ppm=%d tvoc-ppb=%d", m.co2_eq_ppm, m.tvoc_ppb);

        if (!i2c.write(Reg::IAQ_BaselineSet, Measurement{
                                                     .co2_eq_ppm = byteswap(m.tvoc_ppb),
                                                     .co2_crc = crc(byteswap(m.tvoc_ppb)),
                                                     .tvoc_ppb = byteswap(m.co2_eq_ppm),
                                                     .tvoc_crc = crc(byteswap(m.co2_eq_ppm)),
                                             }))
            return false;

        task_delay(10ms);  // spec says 10ms
        return true;
    }

    bool humidity_absolute_set(uint32_t abs_humidity_mg_m3) {
        abs_humidity_mg_m3 = min<uint32_t>(abs_humidity_mg_m3, 256'000);
        // auto scaled = uint16_t((abs_humidity_mg_m3 / 1'000) * 256);
        auto scaled = (uint16_t)((abs_humidity_mg_m3 * 16'777) >> 16);
        scaled = byteswap(scaled);

        struct [[gnu::packed]] Payload {
            uint16_t abs_humidity;
            CRC8_t crc;
        };
        if (!i2c.write(Reg::HumiditySet, Payload{scaled, crc(scaled)})) return false;

        task_delay(10ms);
        return true;
    }

    optional<Measurement> measure(Reg reg, std::chrono::milliseconds delay) {
        auto result = i2c.read<Measurement>(reg, delay);
        if (!result) return {};
        if (!crc(result->co2_eq_ppm, result->co2_crc)) return {};
        if (!crc(result->tvoc_ppb, result->tvoc_crc)) return {};

        result->co2_eq_ppm = byteswap(result->co2_eq_ppm);
        result->tvoc_ppb = byteswap(result->tvoc_ppb);
        return result;
    }
};

}  // namespace

unique_ptr<SensorPeriodic> sgp30(I2C_Bus& bus, EnvironmentalFilter side) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<SGP30Sensor>(bus, address, side); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensors
