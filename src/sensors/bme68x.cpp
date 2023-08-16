#include "lib/bme68x.h"
#include "bme68x.hpp"
#include "hardware/i2c.h"
#include "lib/bme68x_defs.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/i2c.hpp"
#include <chrono>
#include <cstdint>
#include <utility>

using namespace std;

namespace nevermore::sensors {

// don't want to include the big library header via our own header, so just double check it matches here
static_assert(BME68x_POWER_ON_DELAY == chrono::microseconds(BME68X_PERIOD_RESET));

namespace {

// LSB can be 0 or 1, depending on whether a pin is shorted on the SMD.
// Assume LSB of 0 for now.
constexpr uint8_t BME68x_ADDRESS = 0b0111'0110;

constexpr auto BME68x_MODE = BME68X_FORCED_MODE;

bme68x_conf BME68x_SETTINGS{
        .os_hum = BME68X_OS_1X,
        .os_temp = BME68X_OS_1X,
        .os_pres = BME68X_OS_1X,
        .filter = BME68X_FILTER_SIZE_1,
        // TODO: base this off of sampling period
        .odr = BME68X_ODR_250_MS,
};

BME68X_INTF_RET_TYPE i2c_read_(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr) {
    auto* bus = reinterpret_cast<i2c_inst_t*>(intf_ptr);
    if (!i2c_write("BME68x", *bus, BME68x_ADDRESS, reg_addr)) return BME68X_E_COM_FAIL;
    if (!i2c_read("BME68x", *bus, BME68x_ADDRESS, reg_data, len)) return BME68X_E_COM_FAIL;

    return BME68X_OK;
}

BME68X_INTF_RET_TYPE i2c_write_(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr) {
    static_assert(BME68X_LEN_INTERLEAVE_BUFF <= 32);
    assert(len <= BME68X_LEN_INTERLEAVE_BUFF);  // keep things reasonable
    auto* bus = reinterpret_cast<i2c_inst_t*>(intf_ptr);

    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(buf + 1, reg_data, len);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    if (!i2c_write("BME68x", *bus, BME68x_ADDRESS, buf, len + 1)) return BME68X_E_COM_FAIL;

    return BME68X_OK;
}

optional<bme68x_dev> init(i2c_inst_t& bus) {
    bme68x_dev dev{
            .intf_ptr = &bus,
            .intf = BME68X_I2C_INTF,
            .read = i2c_read_,
            .write = i2c_write_,
            .delay_us = [](uint32_t delay_us, void*) { busy_wait_us_32(delay_us); },
    };

    if (auto r = bme68x_init(&dev); r != BME68X_OK) {
        // suppress error msg & assume this just means there's no one on the bus
        if (r == BME68X_E_COM_FAIL) return {};
        if (r == BME68X_E_DEV_NOT_FOUND) return {};  // whatever we found wasn't a BME68x (maybe a BME280?)

        printf("ERR - BME68x - failed to initialize the device (code %+d).\n", r);
        return {};
    }

    if (auto r = bme68x_set_conf(&BME68x_SETTINGS, &dev); r != BME68X_OK) {
        printf("ERR - BME68x - failed to set device settings (code %+d).\n", r);
        return {};
    }

    // Disable the heater; we are not current using the device's gas sensor.
    // (It doesn't detect the VOCs we're interested in, and we don't want to override any attached SGP40s.)
    bme68x_heatr_conf heater_cfg{.enable = false};

    if (auto r = bme68x_set_heatr_conf(BME68x_MODE, &heater_cfg, &dev); r != BME68X_OK) {
        printf("ERR - BME68x - failed to set device heater settings (code %+d).\n", r);
        return {};
    }

    if (auto r = bme68x_set_op_mode(BME68x_MODE, &dev); r != BME68X_OK) {
        printf("ERR - BME68x - failed to set device mode (code %+d).\n", r);
        return {};
    }

    return dev;
}

// This could update more/less frequently, based on the update period (see `bme280_cal_meas_delay`).
// Current update period of 1s should be more than enough to compute results.
struct BME68x final : SensorPeriodic {
    EnvironmentalFilter side;
    bme68x_dev dev;

    BME68x(bme68x_dev dev, EnvironmentalFilter side) : side(side), dev(dev) {}

    [[nodiscard]] char const* name() const override {
        return "BME68x";
    }

    void read() override {
        bme68x_data comp_data{};
        uint8_t n_fields = 0;
        if (auto r = bme68x_get_data(BME68x_MODE, &comp_data, &n_fields, &dev); r < 0) {
            printf("ERR - BME68x - failed read: %d\n", r);
            return;
        }
        if (n_fields == 0) return;

        side.set(BLE::Temperature(comp_data.temperature));
        side.set(BLE::Humidity(comp_data.humidity));
        side.set(BLE::Pressure(comp_data.pressure));
    }
};

}  // namespace

unique_ptr<SensorPeriodic> bme68x(i2c_inst_t& bus, EnvironmentalFilter side) {
    auto dev = init(bus);
    if (!dev) return {};  // nothing found

    return make_unique<BME68x>(*dev, side);
}

}  // namespace nevermore::sensors
