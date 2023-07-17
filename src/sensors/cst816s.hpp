#pragma once

#include "async_sensor.hpp"
#include "hardware/i2c.h"
#include <chrono>
#include <cstdint>
#include <memory>

namespace nevermore::sensors {

// Driver for the CST816S capacitive touch sensor.
// There doesn't seem to be an official SDK for this device, so this was cobbled
// from various sources. (Zephyr, WaveShare samples, etc..)

struct CST816S final : SensorPeriodic {
    static std::unique_ptr<CST816S> mk(i2c_inst_t&);

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

    State state;

    CST816S() = delete;
    CST816S(CST816S const&) = delete;
    CST816S& operator=(const CST816S&) = delete;
    ~CST816S() override;

    [[nodiscard]] char const* name() const override {
        return "CST816S";
    }

    void read() override;
    void register_(async_context_t&) override;
    [[nodiscard]] std::chrono::milliseconds update_period() const override;

    void interrupt();

private:
    i2c_inst_t* bus;
    async_context_t* ctx_async{};

    CST816S(i2c_inst_t&);
};

}  // namespace nevermore::sensors
