#include "btstack_run_loop.h"
#include "config.hpp"
#include "gatt.hpp"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico.h"  // IWYU pragma: keep for transitive includes (e.g. board)
#include "pico/stdio.h"
#include "picowota/reboot.h"
#include "sensors.hpp"
#include "utility/i2c_pins.hpp"
#include <cassert>
#include <chrono>
#include <cstdio>

#if NEVERMORE_PICO_W_BT || defined(CYW43_WL_GPIO_LED_PIN) || CYW43_USES_VSYS_PIN
#define CYW43_IN_USE 1
#else
#define CYW43_IN_USE 0
#endif

#if CYW43_IN_USE
#include "pico/cyw43_arch.h"
#endif

using namespace std;
using namespace nevermore;
using namespace nevermore::sensorium;

namespace {

constexpr auto WATCHDOG_TIMEOUT = 3000ms;

// pinned once started
struct Timer {
    Timer(chrono::milliseconds period, void (*task)(Timer&)) : task(task) {
        assert(task);
        period_set(period);
    }
    Timer(Timer const&) = delete;
    Timer& operator=(Timer) = delete;

    ~Timer() {
        btstack_run_loop_remove_timer(&ctx);
    }

    void schedule() {
        btstack_run_loop_set_timer(&ctx, period.count());
        btstack_run_loop_add_timer(&ctx);
    }

    // NOLINTNEXTLINE(cppcoreguidelines-non-private-member-variables-in-classes)
    void period_set(chrono::milliseconds period) {
        assert(0 < period.count());
        this->period = period;
    }

private:
    btstack_timer_source_t ctx = {
            .process =
                    [](btstack_timer_source_t* t) {
                        auto* self = reinterpret_cast<Timer*>(t->context);
                        self->task(*self);
                        self->schedule();
                    },
            .context = this,
    };
    void (*task)(Timer&);
    chrono::milliseconds period = {};
};

struct WatchdogSetupInfo {
    bool watchdog_caused_reboot;
};

WatchdogSetupInfo setup_watchdog_pre_stdio() {
    WatchdogSetupInfo info{
            .watchdog_caused_reboot = watchdog_enable_caused_reboot(),
    };

    picowota_watchdog_enable_bootloader(WATCHDOG_TIMEOUT / 1ms, true);

    return info;
}

void setup_watchdog_post_stdio(WatchdogSetupInfo const& info) {
    if (info.watchdog_caused_reboot) {
        printf("WARN - last reboot triggered by watchdog timeout\n");
    }
}

}  // namespace

int main() {
    auto const watchdog_info = setup_watchdog_pre_stdio();
    stdio_init_all();

#if CYW43_IN_USE
    // need the CYW43 up to access the LED, even if we don't have BT enabled
    if (auto err = cyw43_arch_init()) {
        panic("ERR - cyw43_arch_init failed = 0x%08x\n", err);
    }
#endif

    setup_watchdog_post_stdio(watchdog_info);

    for (auto const& bus : sensorium::i2c_pins()) {
        if (!bus) continue;

        gpio_set_function(bus.clock, GPIO_FUNC_I2C);
        gpio_set_function(bus.data, GPIO_FUNC_I2C);
        gpio_pull_up(bus.clock);
        gpio_pull_up(bus.data);
    }

    printf("I2C0 running at %u baud/s (requested %u baud/s)\n", i2c_init(i2c0, I2C_BAUD_RATE_SENSOR_MAX),
            unsigned(I2C_BAUD_RATE_SENSOR_MAX));
    printf("I2C1 running at %u baud/s (requested %u baud/s)\n", i2c_init(i2c1, I2C_BAUD_RATE_SENSOR_MAX),
            unsigned(I2C_BAUD_RATE_SENSOR_MAX));

    if (!gatt::init()) panic("ERR - GATT init failed\n");

    static Timer watchdog{WATCHDOG_TIMEOUT / 4, [](auto&) { watchdog_update(); }};
    watchdog.schedule();  // need to wait for btstack loop to be initialised

    if (!sensors::init()) panic("ERR - sensors init failed\n");

    auto poll = [](Timer& self) {
        static sensors::EnvState state;
        static bool issue;
        issue = !issue;

        if (issue) {
            sensors::poll_issue(state);
            self.period_set(max(chrono::duration_cast<chrono::milliseconds>(SENSOR_UPDATE_PERIOD),
                    sensors::poll_readback_delay()));
        } else {
            state = sensors::poll_readback();
            self.period_set(10ms);
        }
    };
    Timer sensors_poll{SENSOR_UPDATE_PERIOD, poll};
    sensors_poll.schedule();

    if constexpr (NEVERMORE_PICO_W_BT) {
        btstack_run_loop_execute();  // ! NO-RETURN
    } else {
        assert(false && "not impl");
    }

    return 0;
}
