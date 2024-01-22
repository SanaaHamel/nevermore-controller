#include "ws2812.hpp"
#include "config.hpp"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/sem.h"
#include "pico/time.h"
#include "ws2812.pio.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdio>

// Debugging helper for testing. Emits a simple animated pattern to the LEDs.
// Useful for determining if a problem lies with DMA, the PIO program, or data layout.
#define DEBUG_WS2812_PATTERN 0
#if DEBUG_WS2812_PATTERN
#include "utility/timer.hpp"
#include <cstring>
#include <numeric>
#endif

#define DEBUG_WS2812_UPDATE_DEFERRED_RATE 0
#if DEBUG_WS2812_UPDATE_DEFERRED_RATE
#include <bitset>
#endif

using namespace std;
using namespace std::literals::chrono_literals;

namespace nevermore::ws2812 {

namespace {

// Fixed sized simplifies memory & error handling.
// Must be a multiple of 4 for DMA transfer purposes.
constexpr size_t N_PIXEL_COMPONENTS_MAX = 64 * sizeof(int);

// WS2812 protocol ends a string of pixel data with a 'long' period of 0v
constexpr auto WS2812_TIME_RESET = 50us;
constexpr auto WS2812_TIME_PER_BIT = 1.25us;

static_assert(50us <= WS2812_TIME_RESET, "datasheet says quiet period must be >= 50us");

auto* const WS2812_PIO = pio0;  // NOLINT
auto const WS2812_SM = pio_claim_unused_sm(WS2812_PIO, true);

auto const g_dma_channel = dma_claim_unused_channel(true);

array<uint8_t, N_PIXEL_COMPONENTS_MAX> g_pixel_data;
size_t g_pixel_data_size = 0;  // INVARIANT(pixel_size_active <= g_pixel_data.size())

semaphore g_update_requested;
semaphore g_update_in_progress;
alarm_id_t g_update_delay_alarm_id = 0;

void DBG_update_deferred_rate_log([[maybe_unused]] bool deferred) {
#if DEBUG_WS2812_UPDATE_DEFERRED_RATE
    constexpr auto LOG_DELAY = 1s;

    static chrono::system_clock::time_point g_log_next;
    static bitset<256> g_entries;
    static size_t g_entry_next = 0;
    static size_t g_entry_count = 0;

    g_entries[g_entry_next++ % g_entries.size()] = deferred;
    g_entry_count = min(g_entry_count + 1, g_entries.size());

    auto now = chrono::system_clock::now();
    if (now < g_log_next) return;  // too soon
    g_log_next = now + LOG_DELAY;

    printf("DBG - WS2812 Deferred Update Rate: %.2f%% (%u of last %u)\n",
            (g_entries.count() / double(g_entry_count)) * 100, g_entries.count(), g_entry_count);
#endif
}

// PRECONDITION: Caller is holding `g_update_in_progress`.
void UNSAFE_update_launch_or_release() {
    if (sem_try_acquire(&g_update_requested)) {  // launch any pending update request
        dma_channel_set_read_addr(g_dma_channel, g_pixel_data.data(), true);
    } else {
        sem_release(&g_update_in_progress);
    }
}

int64_t update_complete_handler(alarm_id_t id, void* user_data) {
    g_update_delay_alarm_id = 0;
    UNSAFE_update_launch_or_release();

    return 0;  // 0 -> no repeat
}

void __isr dma_complete_handler() {
    if (!dma_channel_get_irq0_status(g_dma_channel)) return;
    dma_channel_acknowledge_irq0(g_dma_channel);

    if (g_update_delay_alarm_id) cancel_alarm(g_update_delay_alarm_id);
    g_update_delay_alarm_id =
            add_alarm_in_us(WS2812_TIME_RESET / 1us, update_complete_handler, nullptr, true);
}

void update_or_defer() {
    // raise update-requested; alarm could fire and handle everything before we take update-in-progress
    sem_release(&g_update_requested);
    auto acquired = sem_try_acquire(&g_update_in_progress);
    DBG_update_deferred_rate_log(!acquired);
    if (acquired) {
        UNSAFE_update_launch_or_release();
    }
}

#if DEBUG_WS2812_PATTERN
void dbg_animate(TimerHandle_t) {
    struct GRB {
        uint8_t g, r, b;
    };
    array<GRB, 10> px_data{};
    static_assert(sizeof(px_data) * 2 <= sizeof(g_pixel_data), "not enough space for animated area");

    for (unsigned i = 0; i < px_data.size(); ++i) {
        auto& x = px_data[i];  // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        x.g = 255 * (float(i) / float(px_data.size() - 1));
        x.r = 255;
    }

    static uint pixel_offset = 0;
    pixel_offset = (pixel_offset + 1) % (px_data.size() + 1);

    g_pixel_data = {};
    auto* p = g_pixel_data.begin() + pixel_offset * sizeof(*px_data.begin());
    for (auto x : px_data) {
        memcpy(p, &x, sizeof(x));
        p += sizeof(x);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }

#if 1  // dump buffer to PIO w/o DMA
    auto const* xs = (uint32_t const*)g_pixel_data.data();
    for (unsigned i = 0; i < g_pixel_data.size() / sizeof(int); ++i) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        pio_sm_put_blocking(WS2812_PIO, WS2812_SM, byteswap(xs[i]));
    }
#else  // dump buffer to PIO via DMA
    update_or_defer();
#endif
}
#endif

}  // namespace

void init() {
    sem_init(&g_update_requested, 0, 1);
    sem_init(&g_update_in_progress, 1, 1);

    irq_set_enabled(DMA_IRQ_0, true);
    irq_add_shared_handler(DMA_IRQ_0, dma_complete_handler, PICO_DEFAULT_IRQ_PRIORITY);
    dma_channel_set_irq0_enabled(g_dma_channel, true);

    uint offset = pio_add_program(WS2812_PIO, &ws2812_program);
    ws2812_program_init(WS2812_PIO, WS2812_SM, offset, PIN_NEOPIXEL_DATA_IN, 1s / WS2812_TIME_PER_BIT);

    // Initialise to max by default.
    // Upside: Clients can skip setup (useful if we lose power and reset)
    // Downside: We waste time writing to pixels that don't exist.
    setup(N_PIXEL_COMPONENTS_MAX);

#if DEBUG_WS2812_PATTERN
    mk_timer("dbg-ws2812-blink", 100ms)(dbg_animate);
#endif
}

size_t components_total() {
    return g_pixel_data_size;
}

bool setup(size_t num_components_total) {
    if (g_pixel_data_size == num_components_total) return true;  // no-op

    if (g_pixel_data.size() < num_components_total) {
        printf("ERR - ws2812_setup - n=%u exceeds compile-time specified max size\n", num_components_total);
        return false;  // not enough space in fixed buffer for this setup
    }

    // FUTURE WORK: can tighten timing. only need to wait for DMA to finish, not for full update (DMA + delay)
    sem_acquire_blocking(&g_update_in_progress);  // block until all transfers are done
    {
        g_pixel_data_size = num_components_total;
        g_pixel_data = {};  // reset to zero for consistency

        auto c = dma_channel_get_default_config(g_dma_channel);
        channel_config_set_dreq(&c, pio_get_dreq(WS2812_PIO, WS2812_SM, true));
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_bswap(&c, true);
        static_assert(CHAR_BIT == 8 && sizeof(int) == 4);
        static_assert(
                N_PIXEL_COMPONENTS_MAX % 4 == 0, "`N_MAX_BYTES` must be a multiple of `sizeof(int)` for DMA");
        // This'll write up to 3 extra components (/w value 0) to the end of the sequence.
        // This is benign, and the extra data should be ignored/forwarded by the pixel chain.
        dma_channel_configure(g_dma_channel, &c, &WS2812_PIO->txf[WS2812_SM], nullptr,
                (num_components_total + 3) / 4, false);
    }
    sem_release(&g_update_in_progress);

    return true;
}

bool update(size_t offset, span<uint8_t const> pixel_data) {
    size_t write_end;
    if (__builtin_add_overflow(offset, pixel_data.size(), &write_end) || g_pixel_data_size < write_end) {
        printf("ERR - ws2812_update - offset=%u len=%u is not within declared bounds max=%u\n", offset,
                pixel_data.size(), g_pixel_data_size);
        return false;  // out of bounds
    }

    if (g_pixel_data.empty()) return true;  // no-op

    // DATA RACE - Intentionally don't lock. We'll race with the DMA engine and accept spliced reads.
    copy_n(pixel_data.begin(), pixel_data.size(), g_pixel_data.begin() + offset);
    update_or_defer();
    return true;
}

}  // namespace nevermore::ws2812
