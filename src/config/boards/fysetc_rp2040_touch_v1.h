#pragma once

// For board detection
#define FYSETC_RP2040_TOUCH_V1

// --- UART ---
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 0
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 1
#endif

// --- LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 20
#endif
// no PICO_DEFAULT_WS2812_PIN

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 1
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 6
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 7
#endif

// --- SPI ---
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 1
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 10
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 11
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 12
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN 9
#endif

// --- FLASH ---

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

// Drive high to force power supply into PWM mode (lower ripple on 3V3 at light loads)
// #define PICO_SMPS_MODE_PIN 23

#ifndef PICO_RP2040_B0_SUPPORTED
#define PICO_RP2040_B0_SUPPORTED 1
#endif

// The GPIO Pin used to read VBUS to determine if the device is battery powered.
#ifndef PICO_VBUS_PIN
// Does not have a VBUS pin
// #define PICO_VBUS_PIN 24
#endif

// The GPIO Pin used to monitor VSYS. Typically you would use this with ADC.
// There is an example in adc/read_vsys in pico-examples.
#ifndef PICO_VSYS_PIN
// Does not have a VSYS pin
// #define PICO_VSYS_PIN 29
#endif

// Does have a ADC pin for the fan voltage
#define FYSETC_RP2040_TOUCH_V1_VFAN_PIN 29

#ifndef FYSETC_RP2040_TOUCH_V1_LCD_SPI
#define FYSETC_RP2040_TOUCH_V1_LCD_SPI 1
#endif
#ifndef FYSETC_RP2040_TOUCH_V1_LCD_DC_PIN
#define FYSETC_RP2040_TOUCH_V1_LCD_DC_PIN 8
#endif
#ifndef FYSETC_RP2040_TOUCH_V1_LCD_CS_PIN
#define FYSETC_RP2040_TOUCH_V1_LCD_CS_PIN PICO_DEFAULT_SPI_CSN_PIN
#endif
#ifndef FYSETC_RP2040_TOUCH_V1_LCD_SCLK_PIN
#define FYSETC_RP2040_TOUCH_V1_LCD_SCLK_PIN PICO_DEFAULT_SPI_SCK_PIN
#endif
#ifndef FYSETC_RP2040_TOUCH_V1_LCD_TX_PIN
#define FYSETC_RP2040_TOUCH_V1_LCD_TX_PIN PICO_DEFAULT_SPI_TX_PIN
#endif
#ifndef FYSETC_RP2040_TOUCH_V1_LCD_RST_PIN
#define FYSETC_RP2040_TOUCH_V1_LCD_RST_PIN 13
#endif
#ifndef FYSETC_RP2040_TOUCH_V1_LCD_BL_PIN
#define FYSETC_RP2040_TOUCH_V1_LCD_BL_PIN 25
#endif

#ifndef FYSETC_RP2040_TOUCH_V1_TOUCH_I2C
#define FYSETC_RP2040_TOUCH_V1_TOUCH_I2C 1
#endif
#ifndef FYSETC_RP2040_TOUCH_V1_TOUCH_INTERRUPT_PIN
#define FYSETC_RP2040_TOUCH_V1_TOUCH_INTERRUPT_PIN 21
#endif
#ifndef FYSETC_RP2040_TOUCH_V1_TOUCH_RST_PIN
#define FYSETC_RP2040_TOUCH_V1_TOUCH_RST_PIN 22
#endif
