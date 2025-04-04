// Shout out to Renesas for being the shittiest mfg I've had to deal with (so far).
//  * Doesn't publish register maps.
//  * Requires registration for very basic SDK samples.
//  * Requires "authorisation" for other SDK samples.
//  * Claims the ASIC is so complicated and delicate that they have to hide
//    everything behind binary libraries and shitty APIs or the user might break
//    inadvertently damage the hardware.
//    (And then doesn't even CRC their I2C comms.)
//
// See `dev/zmod4410.py` for reverse engineering notes.
// Most of the structures/bitsets/enums listed here are subsets of what was
// discovered.

#include "zmod4410.hpp"
#include "config.hpp"
#include "sdk/task.hpp"
#include "sensors/environmental_i2c.hpp"
#include "sensors/gas_index.hpp"
#include <bit>
#include <cstdint>
#include <initializer_list>
#include <type_traits>
#include <utility>

using namespace std;
using namespace BLE;

namespace nevermore::sensors {

static_assert(I2C_BAUD_RATE_SENSOR_MAX <= 400'000,
        "`config.hpp`'s `I2C_BAUD_RATE_SENSOR_MAX` is too high for ZMOD4410 (max 400 kbit/s)");

namespace {

constexpr uint8_t ADDRESSES[]{0x32};

// Subset of known/discovered registers.
enum class Reg : uint8_t {
    product_id = 0x00,       // u2 BE
    device_config = 0x20,    // 6 octets
    general_purpose = 0x26,  // 7 to 9 octets response (??)
    tracking_number = 0x3A,  // 6 octets (serial #?)
    cmd_heater = 0x40,       // u2 BE, 8 max
    cmd_delay = 0x50,        // u2 BE, 8 max
    cmd_measure = 0x60,      // u1, 8 max
    cmd_sequencer = 0x68,    // struct size 2, indexes into heater/delay/measure + extra flags
    cmd_execute = 0x93,      // struct size 1
    status = 0x94,           // struct size 1
    result = 0x97,           // u2 BE, 16 max, populated by sequencer
    device_error = 0xB7,     // struct size 1
};

constexpr uint8_t HEATER_MAX = 8;

enum class Cmd : uint8_t {
    stop_sequencer = 0x00,
    start_sequencer = 0x80,
};

enum class ProductId : uint16_t {
    ZMOD4410 = 0x2310,
};

enum class Status : uint8_t {
    last_seq_step = 0b00011111,  // last executed sequencer step
    alarm = 0x20,                // ???
    sleep_timer_enabled = 0x40,
    sequencer_running = 0x80,
};

enum class DeviceErr : uint8_t {
    access_conflict = 0x40,  // ???
    power_on_reset_event = 0x80,
};

using Status_t = underlying_type_t<Status>;
using DeviceErr_t = underlying_type_t<DeviceErr>;

[[nodiscard]] bool operator&(Status_t x, Status y) {
    return x & to_underlying(y);
}

struct DeviceConfig {
    uint8_t mox_scaler;  // TODO: is `* 1000` in rmox calc part of constant or artifact of mox scale equation
    uint8_t unk1;
    uint16_t heater_scaler;
    uint8_t heater_m;
    uint8_t heater_b;

    [[nodiscard]] auto byteswapped() const {
        DeviceConfig self = *this;
        self.heater_scaler = byteswap(heater_scaler);
        return self;
    }

    [[nodiscard]] uint16_t heater_specific_power(uint16_t const x) const {
        // This expression is a pain in the ass and almost certainly depends on
        // implicit int promotion to avoid overflow.
        // NOLINTNEXTLINE(cppcoreguidelines-narrowing-conversions, bugprone-narrowing-conversions)
        float y = -(heater_scaler * ((heater_m + 640.f) * (heater_b + int16_t(x)) - 512000.f));
        y /= 12288000.f;
        assert(0 <= y && y <= 4096);
        return uint16_t(y);
    }

    void log(auto&& l) {
        l.log("DeviceConfig:\n"
              "\tmox_scaler:    %2u\n"
              "\tunk1:          0x%02X\n"
              "\theater_scaler: %5u\n"
              "\theater_m:      %2u\n"
              "\theater_b:      %2u",
                mox_scaler, unk1, heater_scaler, heater_m, heater_b);
    }
};
static_assert(sizeof(DeviceConfig) == 6);

template <typename A>
struct CmdConfig {
    static_assert(sizeof(A) <= 32);  // limited by reg map

    Cmd command = Cmd::start_sequencer;
    Reg result = Reg::result;
    initializer_list<uint16_t> heater;
    initializer_list<uint8_t> delay;
    initializer_list<uint8_t> measure;
    initializer_list<uint8_t> sequencer;
};

struct MeasureResult {
    uint16_t adc;

    [[nodiscard]] MeasureResult byteswapped() const {
        return {.adc = byteswap(adc)};
    }
};

// Using modified IAQ1 profile for now.
// FUTURE WORK: Customise/tailor to our use case.
constexpr CmdConfig<MeasureResult> zmod4410_measure{
        .result = Reg(to_underlying(Reg::result) + 2),  // skip first result
        .heater = {0xFDA8},
        .delay = {0x20, 0x04, 0x20, 0x04},
        .measure = {0x03},
        .sequencer =
                {
                        0x00, 0x00,  // 0 0 0
                        0x80, 0x08,  // 0 1 0 STOP
                },
};

constexpr uint32_t SETUP_RETRIES = 100;

constexpr auto SETUP_RETRY_DELAY = 200ms;
constexpr auto EXECUTE_POLL_DELAY = 50ms;

struct ZMOD4410 final : SensorPeriodicEnvI2C<Reg, "ZMOD4410"> {
    using SensorPeriodicEnvI2C::SensorPeriodicEnvI2C;

    GasIndex index;
    DeviceConfig device_config = {};

    bool setup() {
        // Quickly test if it's there.
        // This device is finicky as fuck and will sometimes stop talking back for a spell.
        if (!i2c.read<DeviceErr_t>(Reg::device_error)) return false;

        auto setup_retry = [&](auto&& attempt) {
            for (uint32_t i = 0; i <= SETUP_RETRIES; ++i) {
                if (attempt()) return true;

                task_delay<SETUP_RETRY_DELAY>();
            }

            i2c.log_error("setup failed - out of retries");
            return false;
        };

        auto revert_to_idle = [&]() {
            auto status = i2c.read<Status_t>(Reg::status);
            if (status && !(*status & Status::sequencer_running)) return true;  // idle/ready

            if (status)
                i2c.log_warn("status 0x%02x", unsigned(*status));
            else
                i2c.log_warn("didn't answer status");

            // don't care if write fails
            [[maybe_unused]] auto _ = i2c.write(Reg::cmd_execute, Cmd::stop_sequencer);
            return false;
        };
        if (!setup_retry(revert_to_idle)) return false;

        auto product_id = i2c.read<uint16_t>(Reg::product_id);
        if (!product_id) return false;
        product_id = byteswap(*product_id);

        i2c.log("product-id: %u", unsigned(*product_id));

        switch (ProductId(*product_id)) {
        default: {
            // each product might need its own cmd configs; don't be permissive
            i2c.log("unsupported product-id: %u", unsigned(*product_id));
            return false;
        }
        case ProductId::ZMOD4410: break;
        }

        auto tracking_num = i2c.read<array<uint8_t, 6>>(Reg::tracking_number);
        if (!tracking_num) return false;
        i2c.log("serial 0x%02x%02x%02x%02x%02x%02x", (*tracking_num)[0], (*tracking_num)[1],
                (*tracking_num)[2], (*tracking_num)[3], (*tracking_num)[4], (*tracking_num)[5]);

        auto config = i2c.read<DeviceConfig>(Reg::device_config);
        if (!config) return false;
        this->device_config = config->byteswapped();
        this->device_config.log(i2c);

        calibration_reset();
        index.restore(side.voc_calibration_blob(), i2c);

        return true;
    }

    void calibration_reset() override {
        // reinit also resets the checkpoint timeout
        index = {GasIndexAlgorithm_ALGORITHM_TYPE_VOC};
        // default VOC is 20'000, which is lower than how this sensor reacts
        index.gia.mSraw_Minimum = 0;
    }

    void calibration_force_checkpoint() override {
        index.checkpoint_clear();
    }

    void read() override {
        if (!prepare(zmod4410_measure)) return;

        auto result = execute(zmod4410_measure);
        if (!result) return;

        // TODO: temp/humidity compensation. this is done host side for this device

        result = result->byteswapped();
        index.process_and_checkpoint(side, i2c, result->adc);
    }

    template <typename A>
    bool prepare(CmdConfig<A> const& c) {
        uint16_t heater_corrected[HEATER_MAX];
        ranges::transform(c.heater, begin(heater_corrected),
                [&](auto x) { return byteswap(device_config.heater_specific_power(int16_t(x))); });

        if (!i2c.write(Reg::cmd_heater,
                    span{reinterpret_cast<uint8_t const*>(heater_corrected), c.heater.size() * 2}))
            return false;
        if (!i2c.write(Reg::cmd_delay, span{c.delay})) return false;
        if (!i2c.write(Reg::cmd_measure, span{c.measure})) return false;
        if (!i2c.write(Reg::cmd_sequencer, span{c.sequencer})) return false;

        return true;
    }

    // PRECONDITION: `c` has been prepared. you may execute a prepared command multiple times
    template <typename A>
    optional<A> execute(CmdConfig<A> const& c) {
        if (!i2c.write(Reg::cmd_execute, c.command)) return {};

        for (;;) {
            auto status = i2c.read<Status_t>(Reg::status);
            if (!status) return {};
            if (!(*status & Status::sequencer_running)) break;

            task_delay<EXECUTE_POLL_DELAY>();
        }

        auto result = i2c.read<A>(c.result);
        if (!result) return {};

        auto dev_status = i2c.read<DeviceErr_t>(Reg::device_error);
        if (!dev_status) return {};
        if (dev_status != 0) {
            i2c.log_error("device err: 0x%02x", *dev_status);
            // immediately ask it to stop, in case its a POR and the sequencer is running wild
            [[maybe_unused]] auto _ = i2c.write(Reg::cmd_execute, Cmd::stop_sequencer);
            return {};
        }

        return result;
    }
};

}  // namespace

unique_ptr<SensorPeriodic> zmod4410(I2C_Bus& bus, EnvironmentalFilter side) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<ZMOD4410>(bus, address, side); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensors
