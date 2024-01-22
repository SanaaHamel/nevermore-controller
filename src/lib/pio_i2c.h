/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Modified by:
//  Sanaa Hamel   (2023-2024)

#ifndef _PIO_I2C_H
#define _PIO_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pico/time.h"
#include "pico/types.h"
#include "pio_i2c.pio.h"

// ----------------------------------------------------------------------------
// Transaction-level functions

int pio_i2c_write_blocking(PIO pio, uint sm, uint8_t addr, uint8_t const* txbuf, size_t len);
int pio_i2c_read_blocking(PIO pio, uint sm, uint8_t addr, uint8_t* rxbuf, size_t len);
int pio_i2c_write_blocking_until(
        PIO pio, uint sm, uint8_t addr, uint8_t const* txbuf, size_t len, absolute_time_t until);
int pio_i2c_read_blocking_until(
        PIO pio, uint sm, uint8_t addr, uint8_t* rxbuf, size_t len, absolute_time_t until);

static inline int pio_i2c_write_timeout_us(
        PIO pio, uint sm, uint8_t addr, const uint8_t* txbuf, size_t len, uint timeout_us) {
    absolute_time_t t = make_timeout_time_us(timeout_us);
    return pio_i2c_write_blocking_until(pio, sm, addr, txbuf, len, t);
}

static inline int pio_i2c_read_timeout_us(
        PIO pio, uint sm, uint8_t addr, uint8_t* rxbuf, size_t len, uint timeout_us) {
    absolute_time_t t = make_timeout_time_us(timeout_us);
    return pio_i2c_read_blocking_until(pio, sm, addr, rxbuf, len, t);
}

#ifdef __cplusplus
}
#endif

#endif
