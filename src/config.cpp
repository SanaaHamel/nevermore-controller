#include "config.hpp"
#include "config/pins.hpp"
#include "sdk/gap.hpp"

using namespace std;

namespace nevermore {

// Spec recommends you give the BT controller some slack in picking an advert interval.
static_assert(ADVERTISE_INTERVAL_MIN < ADVERTISE_INTERVAL_MAX,
        "`config.hpp`'s `ADVERTISE_INTERVAL_MIN` must be < `ADVERTISE_INTERVAL_MAX`");
static_assert(BT_ADVERTISEMENT_INTERVAL_MIN <= ADVERTISE_INTERVAL_MIN,
        "`config.hpp`'s `ADVERTISE_INTERVAL_MIN` is set too low. Minimum is 100ms.");

static_assert(I2C_BAUD_RATE_SENSOR_MAX <= Pins::BusI2C::BAUD_RATE_GENERIC_MAX,
        "`config.hpp`'s `I2C_BAUD_RATE_SENSOR_MAX` exceeds system HW max.");

static_assert(1ms <= STDIO_USB_CONNECT_TIMEOUT || STDIO_USB_CONNECT_TIMEOUT.count() == 0,
        "`config.hpp`'s `STDIO_USB_CONNECT_TIMEOUT` must either be 0ms or a minimum of 1ms");

static_assert(PINS_DEFAULT.validate_or_throw_());

}  // namespace nevermore
