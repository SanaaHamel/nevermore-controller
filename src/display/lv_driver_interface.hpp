#pragma once

// LV DRIVER INTERFACE FUNCTIONS
// Declared here to minimise changes from removing LV driver dependency.

extern "C" {
#include <stdint.h>

void LV_DRV_DISP_CMD_DATA(bool value);
void LV_DRV_DISP_RST(bool value);
void LV_DRV_DISP_SPI_CS(bool);
void LV_DRV_DISP_SPI_WR_ARRAY(char const* src, unsigned len);
void LV_DRV_DISP_SPI_WR_BYTE(uint8_t x);
void LV_DRV_DELAY_MS(uint32_t ms);
}
