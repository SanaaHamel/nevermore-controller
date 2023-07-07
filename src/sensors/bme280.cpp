#include "bme280.hpp"
#include "hardware/i2c.h"
#include "lib/bme280.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/i2c.hpp"
#include <chrono>
#include <cstdint>
#include <utility>

using namespace std;

// don't want to include the big library header via our own header, so just double check it matches here
static_assert(BME280_POWER_ON_DELAY == chrono::microseconds(BME280_STARTUP_DELAY));

namespace {

// LSB can be 0 or 1, depending on whether a pin is shorted on the SMD.
// Assume LSB of 0 for now.
constexpr uint8_t BME280_ADDRESS = 0b0111'0110;

constexpr bme280_settings BME280_SETTINGS{
        .osr_p = BME280_OVERSAMPLING_1X,
        .osr_t = BME280_OVERSAMPLING_1X,
        .osr_h = BME280_OVERSAMPLING_1X,
        .filter = BME280_FILTER_COEFF_2,
        // TODO: base this off of sampling period
        .standby_time = BME280_STANDBY_TIME_250_MS,
};

BME280_INTF_RET_TYPE i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr) {
    auto* bus = reinterpret_cast<i2c_inst_t*>(intf_ptr);
    if (1 != i2c_write_blocking(*bus, BME280_ADDRESS, reg_addr)) return BME280_E_COMM_FAIL;
    if (int(len) != i2c_read_blocking(bus, BME280_ADDRESS, reg_data, len, false)) return BME280_E_COMM_FAIL;

    return BME280_OK;
}

BME280_INTF_RET_TYPE i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr) {
    assert(len < 256);  // keep things reasonable
    auto* bus = reinterpret_cast<i2c_inst_t*>(intf_ptr);

    uint8_t buf[len + 1];
    buf[0] = reg_addr;
    memcpy(buf + 1, reg_data, len);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    if (int(sizeof(buf)) != i2c_write_blocking(bus, BME280_ADDRESS, buf, sizeof(buf), false))
        return BME280_E_COMM_FAIL;

    return BME280_OK;
}

optional<bme280_dev> init(i2c_inst_t& bus) {
    bme280_dev dev{
            .intf = BME280_I2C_INTF,
            .intf_ptr = &bus,
            .read = i2c_read,
            .write = i2c_write,
            .delay_us = [](uint32_t delay_us, void*) { busy_wait_us_32(delay_us); },
    };

    if (auto r = bme280_init(&dev); r != BME280_OK) {
        printf("BME280 - Failed to initialize the device (code %+d).\n", r);
        return {};
    }

    if (auto r = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &BME280_SETTINGS, &dev);
            r != BME280_OK) {
        printf("BME280 - Failed to set device settings (code %+d).\n", r);
        return {};
    }

    if (auto r = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev); r < 0) {
        printf("BME280 - Failed to set normal mode (code %+d).\n", r);
        return {};
    }

    return dev;
}

// This could update more/less frequently, based on the update period (see `bme280_cal_meas_delay`).
// Current update period of 1s should be more than enough to compute results.
struct BME280 final : SensorPeriodic {
    EnvironmentalSensorData data;  // tiny bit wasteful, but terser to manage
    bme280_dev dev;

    BME280(bme280_dev dev, EnvironmentalSensorData data) : data(std::move(data)), dev(dev) {}

    [[nodiscard]] char const* name() const override {
        return "BME280";
    }

    void read() override {
        bme280_data comp_data{};
        if (auto r = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev); r < 0) {
            printf("BME280 error whilst reading data: %d\n", r);
            return;
        }

        get<BLE::Temperature&>(data) = comp_data.temperature;
        get<BLE::Humidity&>(data) = comp_data.humidity;
        get<BLE::Pressure&>(data) = comp_data.pressure;
    }
};

}  // namespace

unique_ptr<SensorPeriodic> bme280(i2c_inst_t& bus, EnvironmentalSensorData state) {
    auto dev = init(bus);
    if (!dev) return {};  // nothing found

    return make_unique<BME280>(*dev, state);
}
