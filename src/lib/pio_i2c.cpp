/**
 * Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Modified by:
//  Sanaa Hamel (2023-2024)

#include "pio_i2c.h"
#include "pico/error.h"
#include "pico/timeout_helper.h"
#include <climits>
#include <cstdio>

namespace {

const int PIO_I2C_ICOUNT_LSB = 10;
const int PIO_I2C_FINAL_LSB = 9;
const int PIO_I2C_DATA_LSB = 1;
const int PIO_I2C_NAK_LSB = 0;

////

bool pio_i2c_check_error(PIO pio, uint sm) {
    return pio_interrupt_get(pio, sm);
}

void pio_i2c_resume_after_error(PIO pio, uint sm) {
    pio_sm_drain_tx_fifo(pio, sm);
    pio_sm_exec(pio, sm,
            (pio->sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
    pio_interrupt_clear(pio, sm);
}

void pio_i2c_rx_enable(PIO pio, uint sm, bool en) {
    if (en)
        hw_set_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    else
        hw_clear_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
}

static inline void pio_i2c_put16(PIO pio, uint sm, uint16_t data) {
    while (pio_sm_is_tx_fifo_full(pio, sm))
        ;
        // some versions of GCC dislike this
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
    *(io_rw_16*)&pio->txf[sm] = data;
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
}

// If I2C is ok, block and push data. Otherwise fall straight through.
void pio_i2c_put_or_err(PIO pio, uint sm, uint16_t data) {
    while (pio_sm_is_tx_fifo_full(pio, sm))
        if (pio_i2c_check_error(pio, sm)) return;
    if (pio_i2c_check_error(pio, sm)) return;
        // some versions of GCC dislike this
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
    *(io_rw_16*)&pio->txf[sm] = data;
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
}

uint8_t pio_i2c_get(PIO pio, uint sm) {
    return (uint8_t)pio_sm_get(pio, sm);
}

void pio_i2c_start(PIO pio, uint sm) {
    pio_i2c_put_or_err(pio, sm, 1u << PIO_I2C_ICOUNT_LSB);  // Escape code for 2 instruction sequence
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]);  // We are already in idle
                                                                                 // state, just pull SDA low
    pio_i2c_put_or_err(pio, sm,
            set_scl_sda_program_instructions[I2C_SC0_SD0]);  // Also pull clock low so we can present data
}

void pio_i2c_stop(PIO pio, uint sm) {
    pio_i2c_put_or_err(pio, sm, 2u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(
            pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]);  // SDA is unknown; pull it down
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]);  // Release clock
    pio_i2c_put_or_err(
            pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1]);  // Release SDA to return to idle state
};

[[maybe_unused]] void pio_i2c_repstart(PIO pio, uint sm) {
    pio_i2c_put_or_err(pio, sm, 3u << PIO_I2C_ICOUNT_LSB);
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD1]);
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD1]);
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC1_SD0]);
    pio_i2c_put_or_err(pio, sm, set_scl_sda_program_instructions[I2C_SC0_SD0]);
}

static void pio_i2c_wait_idle(PIO pio, uint sm) {
    // Finished when TX runs dry or SM hits an IRQ
    pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    while (!(pio->fdebug & 1u << (PIO_FDEBUG_TXSTALL_LSB + sm) || pio_i2c_check_error(pio, sm)))
        tight_loop_contents();
}

[[maybe_unused]] int pio_i2c_write_blocking2(PIO pio, uint sm, uint8_t addr, uint8_t const* txbuf, uint len) {
    int err = 0;
    pio_i2c_start(pio, sm);
    pio_i2c_rx_enable(pio, sm, false);
    pio_i2c_put16(pio, sm, (addr << 2) | 1u);
    while (len && !pio_i2c_check_error(pio, sm)) {
        if (!pio_sm_is_tx_fifo_full(pio, sm)) {
            --len;
            pio_i2c_put_or_err(
                    pio, sm, (*txbuf++ << PIO_I2C_DATA_LSB) | ((len == 0) << PIO_I2C_FINAL_LSB) | 1u);
        }
    }
    pio_i2c_stop(pio, sm);
    pio_i2c_wait_idle(pio, sm);
    if (pio_i2c_check_error(pio, sm)) {
        err = -1;
        pio_i2c_resume_after_error(pio, sm);
        pio_i2c_stop(pio, sm);
    }
    return err;
}

[[maybe_unused]] int pio_i2c_read_blocking2(PIO pio, uint sm, uint8_t addr, uint8_t* rxbuf, uint len) {
    int err = 0;
    pio_i2c_start(pio, sm);
    pio_i2c_rx_enable(pio, sm, true);
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
        (void)pio_i2c_get(pio, sm);
    pio_i2c_put16(pio, sm, (addr << 2) | 3u);
    uint32_t tx_remain = len;  // Need to stuff 0xff bytes in to get clocks

    bool first = true;

    while ((tx_remain || len) && !pio_i2c_check_error(pio, sm)) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(pio, sm)) {
            --tx_remain;
            pio_i2c_put16(pio, sm,
                    (0xffu << 1) | (tx_remain ? 0 : (1u << PIO_I2C_FINAL_LSB) | (1u << PIO_I2C_NAK_LSB)));
        }
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            if (first) {
                // Ignore returned address byte
                (void)pio_i2c_get(pio, sm);
                first = false;
            } else {
                --len;
                *rxbuf++ = pio_i2c_get(pio, sm);
            }
        }
    }
    pio_i2c_stop(pio, sm);
    pio_i2c_wait_idle(pio, sm);
    if (pio_i2c_check_error(pio, sm)) {
        err = -1;
        pio_i2c_resume_after_error(pio, sm);
        pio_i2c_stop(pio, sm);
    }
    return err;
}

////

struct Context {
    PIO pio;
    uint sm;
    check_timeout_fn timeout_check;
    timeout_state_t* ts;

    [[nodiscard]] int check_error() const {
        if (pio_interrupt_get(pio, sm)) {
            // printf("interrupt\n");
            return PICO_ERROR_GENERIC;
        }
        if (timeout_check && timeout_check(ts)) {
            // printf("timeout\n");
            return PICO_ERROR_TIMEOUT;
        }

        return 0;
    }

    void resume_after_error() const {
        pio_sm_drain_tx_fifo(pio, sm);
        pio_sm_exec(pio, sm,
                (pio->sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >>
                        PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB);
        pio_interrupt_clear(pio, sm);
    }

    void rx_enable(bool en) {
        if (en)
            hw_set_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
        else
            hw_clear_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS);
    }

    [[nodiscard]] bool tx_full() const {
        return pio_sm_is_tx_fifo_full(pio, sm);
    }

    [[nodiscard]] bool rx_empty() const {
        return pio_sm_is_rx_fifo_empty(pio, sm);
    }

    void put(uint16_t data) {
        while (tx_full())
            ;
            // some versions of GCC dislike this
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
        *(io_rw_16*)&pio->txf[sm] = data;
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
    }

    // If I2C is ok, block and push data. Otherwise fall straight through.
    void put_or_err(uint16_t data) {
        while (tx_full())
            if (check_error()) return;
        if (check_error()) return;
            // some versions of GCC dislike this
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif
        *(io_rw_16*)&pio->txf[sm] = data;
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
    }

    uint8_t get() {
        return pio_sm_get(pio, sm) & 0xFF;
    }

    void start() {
        // Escape code for 2 instruction sequence
        put_or_err(1u << PIO_I2C_ICOUNT_LSB);
        // We are already in idle state, just pull SDA low
        put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);
        // Also pull clock low so we can present data
        put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);
    }

    void stop() {
        put_or_err(2u << PIO_I2C_ICOUNT_LSB);
        // SDA is unknown; pull it down
        put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);
        // Release clock
        put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);
        // Release SDA to return to idle state
        put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD1]);
    }

    void repstart() {
        put_or_err(3u << PIO_I2C_ICOUNT_LSB);
        put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD1]);
        put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD1]);
        put_or_err(set_scl_sda_program_instructions[I2C_SC1_SD0]);
        put_or_err(set_scl_sda_program_instructions[I2C_SC0_SD0]);
    }

    void wait_idle() {
        // Finished when TX runs dry or SM hits an IRQ
        pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
        while (!(pio->fdebug & 1u << (PIO_FDEBUG_TXSTALL_LSB + sm) || check_error()))
            tight_loop_contents();
    }
};

int pio_i2c_write_blocking_internal(PIO const pio, uint const sm, uint8_t const addr, uint8_t const* txbuf,
        size_t const len, check_timeout_fn const timeout_check, timeout_state_t* ts) {
    assert(len <= INT_MAX);
    // {
    //     auto err = pio_i2c_write_blocking2(pio, sm, addr, txbuf, len);
    //     if (err) return err;
    //     return len;
    // }

    Context ctx{pio, sm, timeout_check, ts};

    ctx.start();
    ctx.rx_enable(false);
    ctx.put_or_err((addr << 2) | 1u);

    for (size_t n = len; n && !ctx.check_error();) {
        if (!ctx.tx_full()) {
            --n;
            ctx.put_or_err((*txbuf++ << PIO_I2C_DATA_LSB) | ((n == 0) << PIO_I2C_FINAL_LSB) | 1u);
        }
    }

    ctx.stop();
    ctx.wait_idle();

    if (auto err = ctx.check_error()) {
        ctx.timeout_check = nullptr;  // disable/clear timeout checks
        ctx.resume_after_error();
        ctx.stop();
        return err;
    }

    return len;
}

int pio_i2c_read_blocking_internal(PIO const pio, uint const sm, uint8_t const addr, uint8_t* rxbuf,
        size_t const len, check_timeout_fn const timeout_check, timeout_state_t* ts) {
    assert(len <= INT_MAX);
    Context ctx{pio, sm, timeout_check, ts};

    ctx.start();
    ctx.rx_enable(true);
    while (!ctx.rx_empty() && !ctx.check_error())
        (void)ctx.get();

    ctx.put_or_err((addr << 2) | 3u);
    size_t tx_remain = len;  // Need to stuff 0xff bytes in to get clocks
    bool first = true;

    for (size_t n = len; (tx_remain || n) && !ctx.check_error();) {
        if (tx_remain && !ctx.tx_full()) {
            --tx_remain;
            ctx.put((0xffu << 1) | (tx_remain ? 0 : (1u << PIO_I2C_FINAL_LSB) | (1u << PIO_I2C_NAK_LSB)));
        }

        if (!ctx.rx_empty()) {
            if (first) {
                // Ignore returned address byte
                (void)ctx.get();
                first = false;
            } else {
                --n;
                *rxbuf++ = ctx.get();
            }
        }
    }

    ctx.stop();
    ctx.wait_idle();

    if (auto err = ctx.check_error()) {
        ctx.timeout_check = nullptr;  // disable/clear timeout checks
        ctx.resume_after_error();
        ctx.stop();
        return err;
    }

    return len;
}

}  // namespace

int pio_i2c_write_blocking_until(
        PIO pio, uint sm, uint8_t addr, uint8_t const* txbuf, size_t len, absolute_time_t until) {
    timeout_state_t ts;
    return pio_i2c_write_blocking_internal(
            pio, sm, addr, txbuf, len, init_single_timeout_until(&ts, until), &ts);
}

int pio_i2c_read_blocking_until(
        PIO pio, uint sm, uint8_t addr, uint8_t* rxbuf, size_t len, absolute_time_t until) {
    timeout_state_t ts;
    return pio_i2c_read_blocking_internal(
            pio, sm, addr, rxbuf, len, init_single_timeout_until(&ts, until), &ts);
}

int pio_i2c_write_blocking(PIO pio, uint sm, uint8_t addr, uint8_t const* txbuf, size_t len) {
    return pio_i2c_write_blocking_internal(pio, sm, addr, txbuf, len, nullptr, nullptr);
}

int pio_i2c_read_blocking(PIO pio, uint sm, uint8_t addr, uint8_t* rxbuf, size_t len) {
    return pio_i2c_read_blocking_internal(pio, sm, addr, rxbuf, len, nullptr, nullptr);
}
