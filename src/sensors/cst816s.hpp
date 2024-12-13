#pragma once

#include "async_sensor.hpp"
#include "config/lib/lv_conf.h"
#include <chrono>
#include <cstdint>
#include <memory>

namespace nevermore {

struct I2C_Bus;

namespace sensors {

// Driver for the CST816S capacitive touch sensor.
// There doesn't seem to be an official SDK for this device, so this was cobbled
// from various sources. (Zephyr, WaveShare samples, etc..)

struct CST816S final : SensorPeriodic {
    struct ISR;
    static std::unique_ptr<CST816S> mk(I2C_Bus&);

    // Pulse the reset pin. (This device is somewhat finicky.)
    static void reset_all();
    // Register ISR with runtime. NOT IDEMPOTENT.
    static void register_isr();

    enum class Touch : uint8_t {
        Down = 0,
        Up = 1,
        Contact = 2,
    };

    enum class Gesture : uint8_t {
        None = 0x00,
        SlideDown = 0x01,
        SlideUp = 0x02,
        SlideLeft = 0x03,
        SlideRight = 0x04,
        ClickSingle = 0x05,
        ClickDouble = 0x0B,
        PressLong = 0x0C
    };

    struct State {
        uint16_t x = 0;
        uint16_t y = 0;
        Touch touch = Touch::Up;
        // FUTURE WORK: Gestures aren't read. LVGL can't make use of them anyways.
        // Gesture gesture = Gesture::None;
    };

    State state;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes)

    CST816S() = delete;
    CST816S(CST816S const&) = delete;
    CST816S& operator=(const CST816S&) = delete;
    ~CST816S() override;

    [[nodiscard]] char const* name() const override {
        return "CST816S";
    }

    [[nodiscard]] std::chrono::milliseconds update_period() const override {
        // no need to update more often than `LV_INDEV_DEF_READ_PERIOD`
        return std::chrono::milliseconds(LV_INDEV_DEF_READ_PERIOD - 1);
    }

protected:
    void read() override;

private:
    I2C_Bus* bus;

    CST816S(I2C_Bus&);
};

}  // namespace sensors
}  // namespace nevermore
