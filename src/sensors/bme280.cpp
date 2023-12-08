#include "bme280.hpp"
#include "lib/bme280.h"
#include "sdk/ble_data_types.hpp"
#include "sensors/environmental_i2c.hpp"
#include <chrono>
#include <cstdint>
#include <utility>

using namespace std;

namespace nevermore::sensors {

// don't want to include the big library header via our own header, so just double check it matches here
static_assert(BME280_POWER_ON_DELAY == chrono::microseconds(BME280_STARTUP_DELAY));

namespace {

constexpr uint8_t ADDRESSES[] = {
        0b0111'0110,
        0b0111'0111,
};

constexpr bme280_settings BME280_SETTINGS{
        .osr_p = BME280_OVERSAMPLING_1X,
        .osr_t = BME280_OVERSAMPLING_1X,
        .osr_h = BME280_OVERSAMPLING_1X,
        .filter = BME280_FILTER_COEFF_2,
        // TODO: base this off of sampling period
        .standby_time = BME280_STANDBY_TIME_250_MS,
};

// unused. we want the helpers from `SensorPeriodicEnvI2C`, but the vendor's
// library is handling everything except I2C calls.
enum class Reg : uint8_t {};

// This could update more/less frequently, based on the update period (see `bme280_cal_meas_delay`).
// Current update period of 1s should be more than enough to compute results.
// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
struct BME280 final : SensorPeriodicEnvI2C<Reg, "BME280"> {
    using SensorPeriodicEnvI2C::SensorPeriodicEnvI2C;
    bme280_dev dev{
            .intf = BME280_I2C_INTF,
            .intf_ptr = this,
            .read = i2c_read_,
            .write = i2c_write_,
            .delay_us = [](uint32_t delay_us, void*) { busy_wait_us_32(delay_us); },
    };

    BME280(BME280 const&) = delete;
    BME280(BME280&&) = delete;

    bool setup() {  // NOLINT(readability-make-member-function-const)
        if (auto r = bme280_init(&dev); r != BME280_OK) {
            // suppress error msg & assume this just means there's no one on the bus
            if (r == BME280_E_COMM_FAIL) return false;
            // whatever we found wasn't a BME280 (maybe a BME68x?)
            if (r == BME280_E_DEV_NOT_FOUND) return false;

            i2c.log_error("failed to initialize the device (code %+d)", r);
            return false;
        }

        if (auto r = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &BME280_SETTINGS, &dev);
                r != BME280_OK) {
            i2c.log_error("failed to set device settings (code %+d)", r);
            return false;
        }

        if (auto r = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev); r < 0) {
            i2c.log_error("failed to set normal mode (code %+d)", r);
            return false;
        }

        return true;
    }

    void read() override {
        bme280_data comp_data{};
        if (auto r = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev); r < 0) {
            i2c.log_error("failed read (code %+d)", r);
            return;
        }

        side.set(BLE::Temperature(comp_data.temperature));
        side.set(BLE::Humidity(comp_data.humidity));
        side.set(BLE::Pressure(comp_data.pressure));
    }

    static BME280_INTF_RET_TYPE i2c_read_(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr) {
        auto* self = reinterpret_cast<BME280*>(intf_ptr);
        if (!i2c_write(self->name(), self->i2c.bus, self->i2c.address, reg_addr)) return BME280_E_COMM_FAIL;
        if (!i2c_read(self->name(), self->i2c.bus, self->i2c.address, reg_data, len))
            return BME280_E_COMM_FAIL;

        return BME280_OK;
    }

    static BME280_INTF_RET_TYPE i2c_write_(
            uint8_t reg_addr, uint8_t const* reg_data, uint32_t len, void* intf_ptr) {
        static_assert(BME280_MAX_LEN * 2 <= 32);
        assert(len <= BME280_MAX_LEN * 2);  // keep things reasonable
        auto* self = reinterpret_cast<BME280*>(intf_ptr);

        uint8_t buf[len + 1];
        buf[0] = reg_addr;
        memcpy(buf + 1, reg_data, len);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        if (!i2c_write(self->name(), self->i2c.bus, self->i2c.address, buf, sizeof(buf)))
            return BME280_E_COMM_FAIL;

        return BME280_OK;
    }
};

}  // namespace

unique_ptr<SensorPeriodic> bme280(i2c_inst_t& bus, EnvironmentalFilter side) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<BME280>(bus, address, side); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensors
