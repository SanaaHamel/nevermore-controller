#include "cst816s.hpp"
#include "config.hpp"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "lvgl.h"
#include "sdk/i2c.hpp"
#include "sdk/timer.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <initializer_list>
#include <utility>

using namespace std;

namespace nevermore::sensors {

static_assert(I2C_BAUD_RATE <= 400'000, "CST816S only supports up to 400 k baud/s");

namespace {

// HACK: GCC 12.2.1 doesn't seem to have an impl for `std::ranges::contains`.
template <typename A, typename B>
bool contains(A const& xs, B const& y) {
    return find(begin(xs), end(xs), y) != end(xs);
}

// Command/register constants taken from Zephyr kernel source.
//  `zephyr/drivers/kscan/kscan_cst816s.c`
//  Author: Qingsong Gou <gouqs@hotmail.com>

constexpr uint8_t ADDRESS = 0x15;

constexpr initializer_list<uint8_t> KNOWN_CHIP_IDS{
        0xB4,
        0xB5,
};

enum class IRQ : uint8_t {
    ONCE_WLP = 1u << 0,
    EN_MOTION = 1u << 4,
    EN_CHANGE = 1u << 5,
    EN_TOUCH = 1u << 6,
    EN_TEST = 1u << 7,
};

enum class Cmd : uint8_t {
    DATA = 0x00,
    GESTURE_ID = 0x01,
    FINGER_NUM = 0x02,
    XPOS_H = 0x03,
    XPOS_L = 0x04,
    YPOS_H = 0x05,
    YPOS_L = 0x06,
    BPC0H = 0xB0,
    BPC0L = 0xB1,
    BPC1H = 0xB2,
    BPC1L = 0xB3,
    POWER_MODE = 0xA5,
    CHIP_ID = 0xA7,
    PROJECT_ID = 0xA8,
    FIRMWARE_VERSION = 0xA9,
    MOTION_MASK = 0xEC,
    IRQ_PULSE_WIDTH = 0xED,
    NOR_SCAN_PER = 0xEE,
    MOTION_S1_ANGLE = 0xEF,
    LP_SCAN_RAW1H = 0xF0,
    LP_SCAN_RAW1L = 0xF1,
    LP_SCAN_RAW2H = 0xF2,
    LP_SCAN_RAW2L = 0xF3,
    LP_AUTO_WAKEUP_TIME = 0xF4,
    LP_SCAN_TH = 0xF5,
    LP_SCAN_WIN = 0xF6,
    LP_SCAN_FREQ = 0xF7,
    LP_SCAN_I_DAC = 0xF8,
    AUTOSLEEP_TIME = 0xF9,
    IRQ_CTL = 0xFA,
    DEBOUNCE_TIME = 0xFB,
    LONG_PRESS_TIME = 0xFC,
    IOCTL = 0xFD,
    DISABLE_AUTO_SLEEP = 0xFE,
};

[[maybe_unused]] uint8_t operator|(IRQ a, IRQ b) {
    return uint8_t(a) | uint8_t(b);
}

[[maybe_unused]] uint8_t operator|(uint8_t a, IRQ b) {
    return a | uint8_t(b);
}

template <typename A = uint8_t>
optional<A> reg_read(i2c_inst_t& bus, Cmd const cmd, bool nostop = false) {
    if (1 != i2c_write_blocking(bus, ADDRESS, cmd)) return {};

    A result;
    if (sizeof(A) != i2c_read_blocking(bus, ADDRESS, result)) return {};

    return result;
}

bool reg_write(i2c_inst_t& bus, Cmd const cmd, uint8_t value) {
    uint8_t data[]{uint8_t(cmd), value};
    return sizeof(data) == i2c_write_blocking(bus, ADDRESS, data);
}

optional<uint8_t> reg_write(i2c_inst_t& bus, Cmd const cmd, uint8_t value, uint8_t mask) {
    if (mask != 0xFF) {
        auto r = reg_read(bus, cmd, true);
        if (!r) {
            printf("failed to read current\n");
            return {};
        }

        value = (value & mask) | (*r & ~mask);
    }

    if (!reg_write(bus, cmd, value)) {
        printf("failed to write current\n");
        return {};
    }

    return value;
}

void reset() {
    gpio_put(PIN_TOUCH_RESET, false);  // trigger on low
    sleep(5ms);
    gpio_put(PIN_TOUCH_RESET, true);
    sleep(50ms);
}

// FIXME: HACK: This blows on so many levels:
// 1) The pico-SDK only tracks 1 callback per core.
// 2) The callback has no parameters.
// 3) The callback is on the wrong core (core 0) if we ever move the UI to core 1.
//
// The workaround to this BS is to track all CST816S instances created and poll
// all of them if any interrupt fires.
//
// Sensors are pinned in memory, and are never destroyed.
struct InstanceMetadata {
    InstanceMetadata() {
        lv_indev_drv_init(&driver);
        driver.type = LV_INDEV_TYPE_POINTER;
        driver.read_cb = read;
    }

    lv_indev_drv_t driver;  // `user_data` must point to CST816S instance
    lv_indev_t* device = nullptr;

private:
    static void read(lv_indev_drv_t* driver, lv_indev_data_t* data) {
        assert(driver);
        assert(data);
        if (!(driver && data)) return;

        auto* self = reinterpret_cast<CST816S*>(driver->user_data);
        assert(self);
        if (!self) return;

        data->point = {.x = lv_coord_t(self->state.x), .y = lv_coord_t(self->state.y)};
        data->state = self->state.touch == CST816S::Touch::Up ? LV_INDEV_STATE_REL : LV_INDEV_STATE_PR;
    }
};
array<InstanceMetadata, NUM_I2CS> g_instances;

struct RegisterInterruptCallback {
    RegisterInterruptCallback() {
        // NOT IDEMPOTENT. Will consume a shared interrupt handler each time.
        // This is a horrible foot-gun of an API.
        gpio_set_irq_enabled_with_callback(
                PIN_TOUCH_INTERRUPT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &go);
    }

    static void go(uint gpio, [[maybe_unused]] uint32_t event_mask) {
        assert(gpio == PIN_TOUCH_INTERRUPT);
        if (gpio != PIN_TOUCH_INTERRUPT) return;

        for (auto& instance : g_instances)
            if (auto* p = reinterpret_cast<CST816S*>(instance.driver.user_data)) p->interrupt();
    }
} g_register_interrupt_callback;

}  // namespace

CST816S::CST816S(i2c_inst_t& bus) : bus(&bus) {
    if (auto* it = ranges::find_if(g_instances, [](auto& x) { return x.driver.user_data == nullptr; });
            it != g_instances.end()) {
        assert(!it->device);
        it->driver.user_data = this;
        it->device = lv_indev_drv_register(&it->driver);
        assert(it->device && "failed to create LVGL input device");
    } else
        assert(false && "unable to register CST816S, too many exist");
}

// Ostensibly we'll never be destroyed, but hey, it's cheap to handle.
CST816S::~CST816S() {
    if (auto* it = ranges::find_if(g_instances, [&](auto& x) { return x.driver.user_data == this; });
            it != g_instances.end()) {
        if (it->device) lv_indev_delete(it->device);
        it->device = nullptr;
        it->driver.user_data = nullptr;
    }
}

void CST816S::read() {
    struct [[gnu::packed]] Batch {
        // for now we don't care/bother to populate these
        // uint8_t gesture;
        // uint8_t fingers;
        uint16_t x;
        uint16_t y;
    };

    auto read = reg_read<Batch>(*bus, Cmd::XPOS_H);
    if (!read) {
        printf("ERR - CST816S - failed to read state\n");
        return;
    }

    state.x = byteswap(read->x) & 0x0FFF;        // read in BE, need it in LE order
    state.y = byteswap(read->y) & 0x0FFF;        // read in BE, need it in LE order
    state.touch = Touch((read->x & 0xFF) >> 6);  // hi 2 bits in `x` are the event
    // state.gesture = Gesture(read->gesture);
}

void CST816S::register_(async_context_t& ctx) {
    SensorPeriodic::register_(ctx);
    ctx_async = &ctx;
}

void CST816S::interrupt() {
    // Can't safely read from the interrupt directly because we might be interrupting
    // an active I2C read from another sensor.
    // (They run on a different IRQ and don't preempt one another)
    if (ctx_async) {
        update_enqueue_immediate(*ctx_async);
    }
}

// We aren't really a periodic sensor, but we need to run as one so we don't
// interrupt others mid read-response.
chrono::milliseconds CST816S::update_period() const {
    return 1min;
}

unique_ptr<CST816S> CST816S::mk(i2c_inst_t& bus) {
    // This device is somewhat finicky.
    // Explicitly reset b/c we may be restarting the program w/o power cycling the device.
    reset();

    auto id = reg_read(bus, Cmd::CHIP_ID);
    if (!id) return {};  // nothing on the bus or error

    if (!contains(KNOWN_CHIP_IDS, *id)) {
        printf("ERR - CST816S - unrecognised chip ID 0x%02x\n", *id);
        return {};
    }

    if (auto rev = reg_read(bus, Cmd::FIRMWARE_VERSION))
        printf("CST816S - revision %u\n", *rev);
    else
        printf("WARN - CST816S - failed to read FW revision\n");

    if (!reg_write(bus, Cmd::IRQ_CTL, IRQ::EN_TOUCH | IRQ::EN_CHANGE, IRQ::EN_TOUCH | IRQ::EN_CHANGE)) {
        printf("ERR - CST816S - failed to change IRQ mode\n");
        return {};
    }

    return unique_ptr<CST816S>{new CST816S(bus)};
}

}  // namespace nevermore::sensors
