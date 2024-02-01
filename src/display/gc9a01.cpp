#include "gc9a01.hpp"
#include "config.hpp"
#include "display/GC9A01.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/regs/intctrl.h"
#include "hardware/spi.h"
#include "sdk/spi.hpp"
#include <cassert>
#include <cstdint>

using namespace std;

namespace nevermore::display {

namespace {

// ===================================
// This section is ripped from the lvgl-driver GC9A01 driver.
// Sadly they don't make it easy to have a customised flushing part...

#define GC9A01_XSTART 0
#define GC9A01_YSTART 0
#define GC9A01_CASET 0x2A
#define GC9A01_RASET 0x2B
#define GC9A01_RAMWR 0x2C
#define GC9A01_CMD_MODE 0
#define GC9A01_DATA_MODE 1

void GC9A01_command(uint8_t cmd) {
    LV_DRV_DISP_CMD_DATA(GC9A01_CMD_MODE);
    LV_DRV_DISP_SPI_WR_BYTE(cmd);
}

void GC9A01_data(uint8_t data) {
    LV_DRV_DISP_CMD_DATA(GC9A01_DATA_MODE);
    LV_DRV_DISP_SPI_WR_BYTE(data);
}

void GC9A01_set_addr_win(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint16_t x_start = x0 + GC9A01_XSTART;
    uint16_t x_end = x1 + GC9A01_XSTART;
    uint16_t y_start = y0 + GC9A01_YSTART;
    uint16_t y_end = y1 + GC9A01_YSTART;

    GC9A01_command(GC9A01_CASET);  // Column addr set
    GC9A01_data(x_start >> 8);
    GC9A01_data(x_start & 0xFF);  // XSTART
    GC9A01_data(x_end >> 8);
    GC9A01_data(x_end & 0xFF);  // XEND

    GC9A01_command(GC9A01_RASET);  // Row addr set
    GC9A01_data(y_start >> 8);
    GC9A01_data(y_start & 0xFF);  // YSTART
    GC9A01_data(y_end >> 8);
    GC9A01_data(y_end & 0xFF);  // YEND

    GC9A01_command(GC9A01_RAMWR);
}

// End ripped from lvgl-driver GC9A01 driver.
// ===================================

int g_dma_channel = -1;
lv_disp_drv_t* g_update_display_driver;

void __isr dma_complete() {
    if (!dma_channel_get_irq0_status(g_dma_channel)) return;
    dma_channel_acknowledge_irq0(g_dma_channel);

    // FIXME: HACK: `g_update_display_driver` can (rarely) be null.
    //  This precondition should be enforced by `gc9a01_flush_dma`, and no one
    //  else should be able to touch `g_dma_channel`, which we've claimed via
    //  `dma_claim_unused_channel`.
    //  Hypothesis 1: Spurious notifies from SDK/HW bug. (??!)
    //  Hypothesis 2: We're somehow getting someone else's DMA notifies??
    if (g_update_display_driver) {
        lv_disp_flush_ready(g_update_display_driver);
        g_update_display_driver = nullptr;
    }
}

// FUTURE WORK: use 2-data mode + PIO to double tx bandwidth
void gc9a01_flush_dma(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p) {
    assert(disp_drv);
    assert(!g_update_display_driver && "transfer already in progres (assume we have 1 display)");
    if (area->x2 < area->x1 || area->y2 < area->y1) return;  // zero area write

    g_update_display_driver = disp_drv;

    LV_DRV_DISP_SPI_CS(false);  // Listen to us

    GC9A01_set_addr_win(area->x1, area->y1, area->x2, area->y2);
    auto len = uint32_t(area->x2 - area->x1 + 1) * uint32_t(area->y2 - area->y1 + 1) * 2;

    LV_DRV_DISP_CMD_DATA(GC9A01_DATA_MODE);

    // SPI seems to require 8 bit chunks. Get corrupted writes w/ 16 bits.
    auto* spi = spi_gpio_bus(PINS_DISPLAY_SPI[0]);
    auto c = dma_channel_get_default_config(g_dma_channel);
    channel_config_set_dreq(&c, spi_get_dreq(spi, true));
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    dma_channel_configure(g_dma_channel, &c, &spi_get_hw(spi)->dr, color_p, len, true);
}

}  // namespace

optional<lv_disp_drv_t> gc9a01() {
    // FIXME: This should have locks to prevent races.
    //        For now display init is done from only one task, so we're safe.
    if (g_dma_channel == -1) {
        g_dma_channel = dma_claim_unused_channel(true);

        irq_set_enabled(DMA_IRQ_0, true);
        irq_add_shared_handler(DMA_IRQ_0, dma_complete, PICO_DEFAULT_IRQ_PRIORITY);
        dma_channel_set_irq0_enabled(g_dma_channel, true);
    }

    if (auto e = GC9A01_init(); e != 0) {
        // Current impl is not expected to ever fail.
        printf("ERR - ui_init - GC9A01_init failed = %d\n", e);
        return {};
    }

    lv_disp_drv_t driver{};
    lv_disp_drv_init(&driver);
    driver.flush_cb = gc9a01_flush_dma;
    return driver;
}

}  // namespace nevermore::display
