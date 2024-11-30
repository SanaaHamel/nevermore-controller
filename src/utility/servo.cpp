#include "servo.hpp"
#include "config/pins.hpp"
#include "periodic_waves.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/pwm.hpp"
#include <cassert>

namespace nevermore {

// NOLINTNEXTLINE(performance-unnecessary-value-param) `pin` is tiny. pass by value.
void servo_set(GPIO const pin, ServoRange const& range, BLE::Percentage16_10 const perc) {
    assert(range.validate());
    if (!pin) return;

    if (perc == BLE::NOT_KNOWN) {
        pwm_set_gpio_duty(pin, 0);  // 0% is typically considered unset/release
        return;
    }

    auto value = periodic_waves::remap(
                         perc.value_or(0), 0., 100., range.start.value_or(0), range.end.value_or(0)) /
                 100;
    // bump 0 -> 1 so servo doesn't disengage
    auto duty = std::max<uint16_t>(1, uint16_t(UINT16_MAX * value));
    pwm_set_gpio_duty(pin, duty);
}

}  // namespace nevermore
