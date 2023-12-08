#include "lib/bme68x.h"
#include "bme68x.hpp"
#include "lib/bme68x_defs.h"
#include "sdk/ble_data_types.hpp"
#include "sensors/environmental_i2c.hpp"
#include <chrono>
#include <cstdint>
#include <utility>

using namespace std;

namespace nevermore::sensors {

// don't want to include the big library header via our own header, so just double check it matches here
static_assert(BME68x_POWER_ON_DELAY == chrono::microseconds(BME68X_PERIOD_RESET));

namespace {

constexpr uint8_t ADDRESSES[] = {
        0b0111'0110,
        0b0111'0111,
};

constexpr auto BME68x_MODE = BME68X_FORCED_MODE;

bme68x_conf BME68x_SETTINGS{
        .os_hum = BME68X_OS_1X,
        .os_temp = BME68X_OS_1X,
        .os_pres = BME68X_OS_1X,
        .filter = BME68X_FILTER_SIZE_1,
        // TODO: base this off of sampling period
        .odr = BME68X_ODR_250_MS,
};

// unused. we want the helpers from `SensorPeriodicEnvI2C`, but the vendor's
// library is handling everything except I2C calls.
enum class Reg : uint8_t {};

// This could update more/less frequently, based on the update period (see `bme280_cal_meas_delay`).
// Current update period of 1s should be more than enough to compute results.
// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
struct BME68x final : SensorPeriodicEnvI2C<Reg, "BME68x"> {
    using SensorPeriodicEnvI2C::SensorPeriodicEnvI2C;
    bme68x_dev dev{
            .intf_ptr = this,
            .intf = BME68X_I2C_INTF,
            .read = i2c_read_,
            .write = i2c_write_,
            .delay_us = [](uint32_t delay_us, void*) { busy_wait_us_32(delay_us); },
    };

    BME68x(BME68x const&) = delete;
    BME68x(BME68x&&) = delete;

    bool setup() {  // NOLINT(readability-make-member-function-const)
        if (auto r = bme68x_init(&dev); r != BME68X_OK) {
            // suppress error msg & assume this just means there's no one on the bus
            if (r == BME68X_E_COM_FAIL) return false;
            // whatever we found wasn't a BME68x (maybe a BME280?)
            if (r == BME68X_E_DEV_NOT_FOUND) return false;

            i2c.log_error("failed to initialize the device (code %+d)", r);
            return false;
        }

        if (auto r = bme68x_set_conf(&BME68x_SETTINGS, &dev); r != BME68X_OK) {
            i2c.log_error("failed to set device settings (code %+d)", r);
            return false;
        }

        // Disable the heater; we are not current using the device's gas sensor.
        // (It doesn't detect the VOCs we're interested in, and we don't want to
        //  override any attached SGP40s.)
        bme68x_heatr_conf heater_cfg{.enable = false};

        if (auto r = bme68x_set_heatr_conf(BME68x_MODE, &heater_cfg, &dev); r != BME68X_OK) {
            i2c.log_error("failed to set device heater settings (code %+d)", r);
            return false;
        }

        if (auto r = bme68x_set_op_mode(BME68x_MODE, &dev); r != BME68X_OK) {
            i2c.log_error("failed to set device mode (code %+d)", r);
            return false;
        }

        return true;
    }

    void read() override {
        bme68x_data comp_data{};
        uint8_t n_fields = 0;
        if (auto r = bme68x_get_data(BME68x_MODE, &comp_data, &n_fields, &dev); r < 0) {
            i2c.log_error("failed read (code %+d)", r);
            return;
        }
        if (n_fields == 0) return;

        side.set(BLE::Temperature(comp_data.temperature));
        side.set(BLE::Humidity(comp_data.humidity));
        side.set(BLE::Pressure(comp_data.pressure));
    }

    static BME68X_INTF_RET_TYPE i2c_read_(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr) {
        auto* self = reinterpret_cast<BME68x*>(intf_ptr);
        if (!i2c_write(self->name(), self->i2c.bus, self->i2c.address, reg_addr)) return BME68X_E_COM_FAIL;
        if (!i2c_read(self->name(), self->i2c.bus, self->i2c.address, reg_data, len))
            return BME68X_E_COM_FAIL;

        return BME68X_OK;
    }

    static BME68X_INTF_RET_TYPE i2c_write_(
            uint8_t reg_addr, uint8_t const* reg_data, uint32_t len, void* intf_ptr) {
        static_assert(BME68X_LEN_INTERLEAVE_BUFF <= 32);
        assert(len <= BME68X_LEN_INTERLEAVE_BUFF);  // keep things reasonable
        auto* self = reinterpret_cast<BME68x*>(intf_ptr);

        uint8_t buf[len + 1];
        buf[0] = reg_addr;
        memcpy(buf + 1, reg_data, len);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        if (!i2c_write(self->name(), self->i2c.bus, self->i2c.address, buf, sizeof(buf)))
            return BME68X_E_COM_FAIL;

        return BME68X_OK;
    }
};

}  // namespace

unique_ptr<SensorPeriodic> bme68x(i2c_inst_t& bus, EnvironmentalFilter side) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<BME68x>(bus, address, side); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensors
