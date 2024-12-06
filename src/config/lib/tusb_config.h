#pragma once

#if CFG_TUSB_OS != OPT_OS_FREERTOS
#if CFG_TUSB_OS != OPT_OS_PICO
#error "Expected `CFG_TUSB_OS=OPT_OS_PICO` from Pico SDK's build system."
#endif

#undef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_FREERTOS
#endif

#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE)

// Need 2 CDCs, 1 for stdio, 1 for GATT-over-serial protocol
#define CFG_TUD_CDC 2
// bigger buffers for higher throughput
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 512
#define CFG_TUD_CDC_EP_BUFSIZE 512
