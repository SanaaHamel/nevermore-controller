#include "bmp280.hpp"
#include "lib/bmp280.h"
#include "lib/bmp280_defs.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/i2c.hpp"
#include "sensors/environmental_i2c.hpp"
#include <cstdint>

using namespace std;

namespace nevermore::sensors {

namespace {

constexpr uint8_t ADDRESSES[] = {
        0b0111'0110,
        0b0111'0111,
};

constexpr bmp280_config BMP280_SETTINGS{
        .os_temp = BMP280_OS_1X,
        .os_pres = BMP280_OS_1X,
        .odr = BMP280_ODR_250_MS,
        .filter = BMP280_FILTER_COEFF_2,
        .spi3w_en = BMP280_SPI3_WIRE_DISABLE,
};

// unused. we want the helpers from `SensorPeriodicEnvI2C`, but the vendor's
// library is handling everything except I2C calls.
enum class Reg : uint8_t {};

// This could update more/less frequently, based on the update period (see `bmp280_cal_meas_delay`).
// Current update period of 1s should be more than enough to compute results.
// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
struct BMP280 final : SensorPeriodicEnvI2C<Reg, "BMP280"> {
    using SensorPeriodicEnvI2C::SensorPeriodicEnvI2C;
    bmp280_dev dev{
            .intf = BMP280_I2C_INTF,
            .intf_ptr = this,
            .read = i2c_read_,
            .write = i2c_write_,
            .delay_ms = [](uint32_t delay_ms) { busy_wait_ms(delay_ms); },
    };

    BMP280(BMP280 const&) = delete;
    BMP280(BMP280&&) = delete;

    bool setup() {  // NOLINT(readability-make-member-function-const)
        if (auto r = bmp280_init(&dev); r != BMP280_OK) {
            // suppress error msg & assume this just means there's no one on the bus
            if (r == BMP280_E_COMM_FAIL) return false;
            // whatever we found wasn't a BMP280 (maybe a BMP68x?)
            if (r == BMP280_E_DEV_NOT_FOUND) return false;

            i2c.log_error("failed to initialize the device (code %+d)", r);
            return false;
        }

        if (auto r = bmp280_set_config(&BMP280_SETTINGS, &dev); r != BMP280_OK) {
            i2c.log_error("failed to set device settings (code %+d)", r);
            return false;
        }

        if (auto r = bmp280_set_power_mode(BMP280_NORMAL_MODE, &dev); r < 0) {
            i2c.log_error("failed to set normal mode (code %+d)", r);
            return false;
        }

        return true;
    }

    void read() override {
        bmp280_uncomp_data raw{};
        if (auto r = bmp280_get_uncomp_data(&raw, &dev); r < 0) {
            i2c.log_error("failed read (code %+d)", r);
            return;
        }

        int32_t t;
        if (auto r = bmp280_get_comp_temp_32bit(&t, raw.uncomp_temp, &dev); r == BMP280_OK)
            side.set(BLE::Temperature(t / 10.));
        else
            i2c.log_error("temperature error (code %+d)", r);

        uint32_t p;
        if (auto r = bmp280_get_comp_pres_32bit(&p, raw.uncomp_press, &dev); r == BMP280_OK)
            side.set(BLE::Pressure(p));
        else
            i2c.log_error("pressure error (code %+d)", r);
    }

    static int8_t i2c_read_(uint8_t reg_addr, uint8_t* reg_data, uint16_t len, void* intf_ptr) {
        auto* self = reinterpret_cast<BMP280*>(intf_ptr);
        if (!self->i2c.read(reg_addr, reg_data, len)) return BMP280_E_COMM_FAIL;

        return BMP280_OK;
    }

    static int8_t i2c_write_(uint8_t reg_addr, uint8_t* reg_data, uint16_t len, void* intf_ptr) {
        assert(len <= 32);  // keep things reasonable
        auto* self = reinterpret_cast<BMP280*>(intf_ptr);
        if (!self->i2c.write(reg_addr, reg_data, len)) return BMP280_E_COMM_FAIL;

        return BMP280_OK;
    }
};

}  // namespace

unique_ptr<SensorPeriodic> bmp280(I2C_Bus& bus, EnvironmentalFilter side) {
    for (auto address : ADDRESSES)
        if (auto p = make_unique<BMP280>(bus, address, side); p->setup()) return p;

    return {};
}

}  // namespace nevermore::sensors
