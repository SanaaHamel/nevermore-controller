#include "utility/i2c.hpp"
#include "config.hpp"
#include "hardware/gpio.h"
#include "sdk/i2c.hpp"
#include "sdk/timer.hpp"
#include <cassert>
#include <chrono>

using namespace std::literals::chrono_literals;

namespace nevermore {

constexpr auto CLOCK_PULSE_DURATION = 10us;

// Derived from Apache Nuttx's implementation.
// https://github.com/apache/nuttx/blob/master/arch/arm/src/rp2040/rp2040_i2c.c#L623
// Consider this function under the Apache License.
bool i2c_bitbang_reset(GPIO const sda, GPIO const scl, unsigned const clock_cycles_timeout) {
    assert(i2c_gpio_bus_num(sda) == i2c_gpio_bus_num(scl) && "must be on same bus");
    assert(i2c_gpio_kind(sda) == I2C_Pin::SDA && "must be an SDA pin");
    assert(i2c_gpio_kind(scl) == I2C_Pin::SCL && "must be an SCL pin");

    gpio_set_function(sda, GPIO_FUNC_SIO);
    gpio_set_function(scl, GPIO_FUNC_SIO);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
    gpio_set_dir(sda, true);
    gpio_set_dir(scl, true);

    /* Let SDA go high */

    gpio_put(sda, true);
    gpio_put(scl, true);

    auto put_pause = [](GPIO gpio, bool value) {
        gpio_put(gpio, value);
        sleep(CLOCK_PULSE_DURATION);
    };

    // keep pulsing the clock until everyone lets go of the data line
    for (unsigned clock_cycles = 0; !gpio_get(sda); ++clock_cycles) {
        if (clock_cycles_timeout <= clock_cycles) return false;

        // wait until no one is holding down the clock line
        // (keep a separate timeout track here)
        for (unsigned clock_cycles = 0; !gpio_get(scl); ++clock_cycles) {
            if (clock_cycles_timeout <= clock_cycles) return false;

            sleep(CLOCK_PULSE_DURATION);
        }

        // pulse clock
        put_pause(scl, false);
        put_pause(scl, true);
    }

    // emit START followed by STOP -> reset devices' state machines
    put_pause(sda, false);
    put_pause(scl, false);
    put_pause(scl, true);
    put_pause(sda, true);

    return true;
}

}  // namespace nevermore
