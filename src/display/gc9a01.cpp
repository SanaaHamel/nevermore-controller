#include "gc9a01.hpp"
#include "FreeRTOS.h"  // IWYU pragma: keep
#include "display.hpp"
#include "display/GC9A01.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "task.h"
#include <cassert>
#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <utility>

using namespace std;
using namespace std::literals::chrono_literals;

namespace nevermore::display {

namespace {

// ===================================
// This section is ripped from the lvgl-driver GC9A01 driver.
// Sadly they don't make it easy to have a customised flushing part...
// Nor do they offer an option to skip the dumb slow in-fill.

#define GC9A01_XSTART 0
#define GC9A01_YSTART 0
#define GC9A01_CMD_MODE 0
#define GC9A01_DATA_MODE 1

/* GC9A01 Commands that we know of.  Limited documentation */
// Documentation on op codes for GC9A01 are very hard to find.
// Will document should they be found.
//
// NB: Parameters are to be passed in BE order
enum class GC9A01 : uint8_t {
    sleep_out_mode = 0x11,         // []
    display_inversion_off = 0x20,  // ?? assume []
    display_inversion_on = 0x21,   // []
    display_on = 0x29,             // []
    column_addr_set = 0x2A,        // u16 y start, u16 y end
    row_addr_set = 0x2B,           // u16 y start, u16 y end
    ram_wr = 0x2C,                 // []
    colour_mode = 0x3A,            // 1 octet
    mad_ctl = 0x36,                // 1 octet
    mad_ctl_rgb = 0x00,
    mad_ctl_mv = 0x20,
    mad_ctl_mx = 0x40,
    mad_ctl_my = 0x80,
    tearing_effect_line_on = 0x35,  // []
    display_fn_ctrl = 0xB6,         // 2 octets

    power_ctl_2 = 0xC3,
    power_ctl_3 = 0xC4,
    power_ctl_4 = 0xC9,

    set_gamma_1 = 0xF0,  // 6 octets
    set_gamma_2 = 0xF1,  // 6 octets
    set_gamma_3 = 0xF2,  // 6 octets
    set_gamma_4 = 0xF3,  // 6 octets

    unk_62 = 0x62,  // 12 octets
    unk_63 = 0x63,  // 12 octets
    unk_64 = 0x64,  // 7 octets
    unk_66 = 0x66,  // 10 ocetets
    unk_67 = 0x67,  // 10 octets
    unk_70 = 0x70,  // 9 octets
    unk_74 = 0x74,  // 7 ocetets
    unk_84 = 0x84,  // 1 octet
    unk_85 = 0x85,  // 1 octet
    unk_86 = 0x86,  // 1 octet
    unk_87 = 0x87,  // 1 octet
    unk_88 = 0x88,  // 1 octet
    unk_89 = 0x89,  // 1 octet
    unk_8A = 0x8A,  // 1 octet
    unk_8B = 0x8B,  // 1 octet
    unk_8C = 0x8C,  // 1 octet
    unk_8D = 0x8D,  // 1 octet
    unk_8E = 0x8E,  // 1 octet
    unk_8F = 0x8F,  // 1 octet
    unk_90 = 0x90,  // 4 octets
    unk_98 = 0x98,  // 2 octets
    unk_AE = 0xAE,  // 1 octet
    unk_BD = 0xBD,  // 1 octet
    unk_BC = 0xBC,  // 1 octet
    unk_BE = 0xBE,  // 1 octet
    unk_CD = 0xCD,  // 1 octet
    unk_DF = 0xDF,  // 3 octets
    unk_E1 = 0xE1,  // 2 octets
    unk_E8 = 0xE8,  // 1 octets
    unk_EB = 0xEB,  // 1 octet
    unk_ED = 0xED,  // 2 octet
    unk_FF = 0xFF,  // 3 octet

    inter_register_enable_1 = 0xFE,  // []
    inter_register_enable_2 = 0xEF,  // []
};

void GC9A01_command(GC9A01 cmd) {
    LV_DRV_DISP_CMD_DATA(GC9A01_CMD_MODE);
    LV_DRV_DISP_SPI_WR_BYTE(to_underlying(cmd));
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

    GC9A01_command(GC9A01::column_addr_set);
    GC9A01_data(x_start >> 8);
    GC9A01_data(x_start & 0xFF);  // XSTART
    GC9A01_data(x_end >> 8);
    GC9A01_data(x_end & 0xFF);  // XEND

    GC9A01_command(GC9A01::row_addr_set);
    GC9A01_data(y_start >> 8);
    GC9A01_data(y_start & 0xFF);  // YSTART
    GC9A01_data(y_end >> 8);
    GC9A01_data(y_end & 0xFF);  // YEND

    GC9A01_command(GC9A01::ram_wr);
}

void GC9A01_hard_reset() {
    LV_DRV_DISP_SPI_CS(false);  // Low to listen to us

    LV_DRV_DISP_RST(true);
    LV_DRV_DELAY_MS(50);
    LV_DRV_DISP_RST(false);
    LV_DRV_DELAY_MS(50);
    LV_DRV_DISP_RST(true);
    LV_DRV_DELAY_MS(50);
}

struct ScriptCommand {
    enum class Op : uint8_t { cmd, data, delay };

    constexpr ScriptCommand(GC9A01 cmd) : op(Op::cmd), data(to_underlying(cmd)) {}
    constexpr ScriptCommand(uint8_t data) : op(Op::data), data(data) {}
    constexpr ScriptCommand(chrono::milliseconds delay) : op(Op::delay), data(delay.count()) {
        assert(0 <= delay.count() && delay.count() <= UINT16_MAX);
    }

    Op op;
    uint16_t data;
};

constexpr initializer_list<ScriptCommand> INIT_SCRIPT = {
        GC9A01::inter_register_enable_2,  // Inter Register Enable2 (??? why here again?)

        GC9A01::unk_EB,
        0x14,

        GC9A01::inter_register_enable_1,  // Inter Register Enable1
        GC9A01::inter_register_enable_2,  // Inter Register Enable2

        GC9A01::unk_EB,
        0x14,

        GC9A01::unk_84,
        0x40,

        GC9A01::unk_85,
        0xFF,

        GC9A01::unk_86,
        0xFF,

        GC9A01::unk_87,
        0xFF,

        GC9A01::unk_88,
        0x0A,

        GC9A01::unk_89,
        0x21,

        GC9A01::unk_8A,
        0x00,

        GC9A01::unk_8B,
        0x80,

        GC9A01::unk_8C,
        0x01,

        GC9A01::unk_8D,
        0x01,

        GC9A01::unk_8E,
        0xFF,

        GC9A01::unk_8F,
        0xFF,

        GC9A01::display_fn_ctrl,
        0x00,
        0x00,

        GC9A01::mad_ctl,  // Memory Access Control
        0x48,             // Set the display direction 0,1,2,3	four directions

        GC9A01::colour_mode,  // COLMOD: Pixel Format Set
        0x05,                 // 16 Bits per pixel

        GC9A01::unk_90,
        0x08,
        0x08,
        0x08,
        0x08,

        GC9A01::unk_BD,
        0x06,

        GC9A01::unk_BC,
        0x00,

        GC9A01::unk_FF,
        0x60,
        0x01,
        0x04,

        GC9A01::power_ctl_2,  // Power Control 2
        0x13,
        GC9A01::power_ctl_3,  // Power Control 3
        0x13,
        GC9A01::power_ctl_4,  // Power Control 4
        0x22,

        GC9A01::unk_BE,
        0x11,

        GC9A01::unk_E1,
        0x10,
        0x0E,

        GC9A01::unk_DF,
        0x21,
        0x0C,
        0x02,

        GC9A01::set_gamma_1,
        0x45,
        0x09,
        0x08,
        0x08,
        0x26,
        0x2A,

        GC9A01::set_gamma_2,
        0x43,
        0x70,
        0x72,
        0x36,
        0x37,
        0x6F,

        GC9A01::set_gamma_3,
        0x45,
        0x09,
        0x08,
        0x08,
        0x26,
        0x2A,

        GC9A01::set_gamma_4,
        0x43,
        0x70,
        0x72,
        0x36,
        0x37,
        0x6F,

        GC9A01::unk_ED,
        0x1B,
        0x0B,

        GC9A01::unk_AE,
        0x77,

        GC9A01::unk_CD,
        0x63,

        GC9A01::unk_70,
        0x07,
        0x07,
        0x04,
        0x0E,
        0x0F,
        0x09,
        0x07,
        0x08,
        0x03,

        GC9A01::unk_E8,
        0x34,

        GC9A01::unk_62,
        0x18,
        0x0D,
        0x71,
        0xED,
        0x70,
        0x70,
        0x18,
        0x0F,
        0x71,
        0xEF,
        0x70,
        0x70,

        GC9A01::unk_63,
        0x18,
        0x11,
        0x71,
        0xF1,
        0x70,
        0x70,
        0x18,
        0x13,
        0x71,
        0xF3,
        0x70,
        0x70,

        GC9A01::unk_64,
        0x28,
        0x29,
        0xF1,
        0x01,
        0xF1,
        0x00,
        0x07,

        GC9A01::unk_66,
        0x3C,
        0x00,
        0xCD,
        0x67,
        0x45,
        0x45,
        0x10,
        0x00,
        0x00,
        0x00,

        GC9A01::unk_67,
        0x00,
        0x3C,
        0x00,
        0x00,
        0x00,
        0x01,
        0x54,
        0x10,
        0x32,
        0x98,

        GC9A01::unk_74,
        0x10,
        0x85,
        0x80,
        0x00,
        0x00,
        0x4E,
        0x00,

        GC9A01::unk_98,
        0x3E,
        0x07,

        GC9A01::tearing_effect_line_on,  // Tearing Effect Line ON
        GC9A01::display_inversion_on,    // Display Inversion ON

        GC9A01::sleep_out_mode,  // Sleep Out Mode
        120ms,
        GC9A01::display_on,  // Display ON
        255ms,
};

void GC9A01_run_script(initializer_list<ScriptCommand> const& script) {
    for (auto&& [op, arg] : script) {
        using enum ScriptCommand::Op;
        switch (op) {
        case cmd: GC9A01_command(GC9A01(arg & 0xFF)); break;
        case data: GC9A01_data(arg & 0xFF); break;
        case delay: LV_DRV_DELAY_MS(arg); break;
        }
    }
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

    auto* spi = display::active_spi();
    if (!spi) return;

    g_update_display_driver = disp_drv;

    LV_DRV_DISP_SPI_CS(false);  // Listen to us

    GC9A01_set_addr_win(area->x1, area->y1, area->x2, area->y2);
    auto len = uint32_t(area->x2 - area->x1 + 1) * uint32_t(area->y2 - area->y1 + 1) * 2;

    LV_DRV_DISP_CMD_DATA(GC9A01_DATA_MODE);

    // SPI seems to require 8 bit chunks. Get corrupted writes w/ 16 bits.
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

    GC9A01_hard_reset();
    GC9A01_run_script(INIT_SCRIPT);

    lv_disp_drv_t driver{};
    lv_disp_drv_init(&driver);
    driver.flush_cb = gc9a01_flush_dma;
    driver.wait_cb = [](lv_disp_drv_t*) { taskYIELD(); };
    return driver;
}

}  // namespace nevermore::display
