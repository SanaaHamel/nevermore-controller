// Torn screaming and kicking from the Pico SDK.

#include "stdio_usb.h"
#include "FreeRTOS.h"          // IWYU pragma: keep
#include "pico/binary_info.h"  // IWYU pragma: keep
#include "pico/error.h"
#include "pico/stdio/driver.h"  // IWYU pragma: keep [doesn't notice `stdio_driver_t`]
#include "pico/time.h"
#include "projdefs.h"
#include "sdk/i2c.hpp"
#include "semphr.h"  // IWYU pragma: keep [doesn't notice `SemaphoreHandle_t`]
#include "tusb.h"    // IWYU pragma: keep
#include "utility/scope_guard.hpp"

#if !((CFG_TUD_ENABLED | TUSB_OPT_DEVICE_ENABLED) && CFG_TUD_CDC)
#error USB not enabled.
#endif

#if PICO_STDIO_USB_SUPPORT_CHARS_AVAILABLE_CALLBACK
#error Not implemented
#endif

using namespace std;
using namespace nevermore;

namespace {

constexpr int STDIO_ITF = 0;

// Like xSemaphoreTake, but handles a suspended scheduler.
// Allows stdio when the scheduler is suspended.
bool semaphore_take_safe(SemaphoreHandle_t handle, TickType_t ticks_to_wait) {
    if (xSemaphoreTake(handle, 0)) return true;  // immediately got it
    // can't acquire it by waiting when suspended; handle it like a timeout
    if (xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) return false;

    return xSemaphoreTake(handle, ticks_to_wait);
}

SemaphoreHandle_t g_stdio_usb_mutex;

void stdio_usb_out_chars(const char* buf, int length) {
    if (!semaphore_take_safe(g_stdio_usb_mutex, pdMS_TO_TICKS(PICO_STDIO_DEADLOCK_TIMEOUT_MS))) return;
    SCOPE_GUARD {
        xSemaphoreGive(g_stdio_usb_mutex);
    };

    static uint64_t last_avail_time;
    if (!stdio_usb_connected()) {
        // reset our timeout
        last_avail_time = 0;
        return;
    }

    for (int i = 0; i < length;) {
        auto n = length - i;
        auto written = tud_cdc_n_write(STDIO_ITF, buf + i, (uint32_t)n);
        tud_cdc_n_write_flush(STDIO_ITF);
        i += written;

        if (0 == written) {
            if (!stdio_usb_connected()) break;
            if (!tud_cdc_n_write_available(STDIO_ITF) &&
                    last_avail_time + PICO_STDIO_USB_STDOUT_TIMEOUT_US <= time_us_64())
                break;
        } else {
            last_avail_time = time_us_64();
        }
    }
}

int stdio_usb_in_chars(char* buf, int length) {
    // note we perform this check outside the lock, to try and prevent possible deadlock conditions
    // with printf in IRQs (which we will escape through timeouts elsewhere, but that would be less graceful).
    //
    // these are just checks of state, so we can call them while not holding the lock.
    // they may be wrong, but only if we are in the middle of a tud_task call, in which case at worst
    // we will mistakenly think we have data available when we do not (we will check again), or
    // tud_task will complete running and we will check the right values the next time.
    //
    auto could_read = [] { return stdio_usb_connected() && tud_cdc_n_available(STDIO_ITF); };
    if (!could_read()) return PICO_ERROR_NO_DATA;

    if (!semaphore_take_safe(g_stdio_usb_mutex, pdMS_TO_TICKS(PICO_STDIO_DEADLOCK_TIMEOUT_MS)))
        return PICO_ERROR_NO_DATA;
    SCOPE_GUARD {
        xSemaphoreGive(g_stdio_usb_mutex);
    };

    // check again, might not be true by the time we get the lock
    if (!could_read()) return PICO_ERROR_NO_DATA;

    return (int)tud_cdc_n_read(STDIO_ITF, buf, (uint32_t)length);
}

}  // namespace

stdio_driver_t stdio_usb = {
        .out_chars = stdio_usb_out_chars,
        .in_chars = stdio_usb_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
        .crlf_enabled = PICO_STDIO_USB_DEFAULT_CRLF,
#endif
};

bool stdio_usb_init() {
#if !PICO_NO_BI_STDIO_USB
    bi_decl_if_func_used(bi_program_feature("USB stdin / stdout"));
#endif

    if (get_core_num() != alarm_pool_core_num(alarm_pool_get_default())) {
        // included an assertion here rather than just returning false, as this is likely
        // a coding bug, rather than anything else.
        assert(false);
        return false;
    }

    assert(tud_inited());  // we expect the caller to have initialized if they are using TinyUSB

    g_stdio_usb_mutex = xSemaphoreCreateMutex();

    stdio_set_driver_enabled(&stdio_usb, true);

#if PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS
#if PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS > 0
    absolute_time_t until = make_timeout_time_ms(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS);
#else
    absolute_time_t until = at_the_end_of_time;
#endif
    do {
        if (stdio_usb_connected()) {
#if PICO_STDIO_USB_POST_CONNECT_WAIT_DELAY_MS != 0
            task_delay(chorno::milliseconds(PICO_STDIO_USB_POST_CONNECT_WAIT_DELAY_MS));
#endif
            break;
        }
        sleep_ms(10);
    } while (!time_reached(until));
#endif

    return true;
}

bool stdio_usb_connected() {
#if PICO_STDIO_USB_CONNECTION_WITHOUT_DTR
    return tud_ready();
#else
    // this actually checks DTR
    return tud_cdc_n_connected(STDIO_ITF);
#endif
}
