#pragma once

#include "FreeRTOS.h"  // IWYU pragma: keep
#include "semphr.h"    // IWYU pragma: keep [doesn't notice `SemaphoreHandle_t`]
#include "utility/crc.hpp"
#include "utility/scope_guard.hpp"
#include <cassert>
#include <climits>
#include <cstdarg>
#include <cstdio>
#include <optional>
#include <type_traits>
#include <utility>

// NB: These variants are lock guarded to prevent both cores from using the same bus.

namespace nevermore {

constexpr uint32_t I2C_TIMEOUT_US = 1000000;

enum class I2C_Pin : uint8_t { SDA = 0, SCL = 1 };

constexpr uint8_t i2c_gpio_bus_num(uint8_t pin) {
    return (pin / 2) & 1u;
}

constexpr I2C_Pin i2c_gpio_kind(uint8_t pin) {
    return I2C_Pin(pin % 2);
}

// I2C reserves some addresses for special purposes.
// These are any addresses of the form: 000 0xxx, 111 1xxx
constexpr bool i2c_address_reserved(uint8_t addr) {
    constexpr uint8_t MASK = 0b1111000;
    auto const masked = addr & MASK;
    return masked == 0 || masked == MASK;
}

struct I2C_Bus {  // NOLINT(cppcoreguidelines-special-member-functions)
    I2C_Bus(SemaphoreHandle_t lock = xSemaphoreCreateMutex()) : lock(lock) {};
    I2C_Bus(I2C_Bus const&) = delete;
    virtual ~I2C_Bus() {
        vSemaphoreDelete(lock);
    }

    [[nodiscard]] auto guard() {  // NOLINT(readability-make-member-function-const)
        xSemaphoreTake(lock, portMAX_DELAY);
        return ScopeGuard{[&] { xSemaphoreGive(lock); }};
    }

    [[nodiscard]] bool write(char const* name, uint8_t addr, uint8_t const* src, size_t len) {
        assert(len <= INT_MAX && "success unrepresentable");
        auto _ = guard();
#if NEVERMORE_I2C_DEBUG
        log_debug(name, addr, "writing len=%d", len);
#endif
        int r = write(addr, src, len);
        if (r < 0 || size_t(r) != len) {
            log_error(name, addr, "write failed; len=%d result=%d", len, r);
            return false;
        }

        return true;
    }

    [[nodiscard]] bool read(char const* name, uint8_t addr, uint8_t* dst, size_t len) {
        assert(len <= INT_MAX && "success unrepresentable");
        auto _ = guard();
#if NEVERMORE_I2C_DEBUG
        log_debug(name, addr, "reading len=%d", len);
#endif
        int r = read(addr, dst, len);
        if (r < 0 || size_t(r) != len) {
            log_error(name, addr, "read failed; len=%d result=%d", len, r);
            return false;
        }

        return true;
    }

    template <typename A>
    [[nodiscard]] bool write(char const* name, uint8_t addr, A const& blob)
        requires(!std::is_pointer_v<A>)
    {
        return write(name, addr, reinterpret_cast<uint8_t const*>(&blob), sizeof(A));
    }

    template <typename A>
    [[nodiscard]] bool read(char const* name, uint8_t addr, A& blob)
        requires(!std::is_pointer_v<A>)
    {
        return read(name, addr, reinterpret_cast<uint8_t*>(&blob), sizeof(A));
    }

    template <typename A>
    [[nodiscard]] std::optional<A> read(char const* name, uint8_t addr)
        requires(!std::is_pointer_v<A>)
    {
        A result;
        if (!read(name, addr, result)) return {};
        return {std::move(result)};
    }

    template <CRC8_t CRC_INIT, typename A>
    [[nodiscard]] std::optional<A> read_crc(char const* name, uint8_t addr) {
        ResponseCRC<A, CRC_INIT> response;
        if (!read(name, addr, response)) return {};

        if (!response.verify()) {
            // really should show up in a log if they've noise in their wiring
            log_error(name, addr, "read failed CRC; crc-reported=0x%02x crc-computed=0x%02x", response.crc,
                    response.data_crc());
            return {};
        }

        // HACK: explicit copy to work around packed value ref issue
        return A(response.data);
    }

#define DEFINE_I2C_LOG(fn_name, prefix)                                                  \
    [[gnu::format(printf, 4, 5)]]                                                        \
    void fn_name(char const* name, uint8_t addr, const char* format, ...) const {        \
        va_list arglist;                                                                 \
        va_start(arglist, format);                                                       \
        fn_name(name, addr, format, arglist);                                            \
        va_end(arglist);                                                                 \
    }                                                                                    \
    void fn_name(char const* name, uint8_t addr, const char* format, va_list xs) const { \
        /* HACK: race-y, but whatever, stdio is race-y... */                             \
        printf(prefix "[%s 0x%02x] %s - ", this->name(), addr, (char const*)name);       \
        vprintf(format, xs);                                                             \
        puts(""); /* `puts` implicitly emits a newline after argument */                 \
    }

    DEFINE_I2C_LOG(log, "")
    DEFINE_I2C_LOG(log_warn, "WARN - ")
    DEFINE_I2C_LOG(log_error, "ERR - ")
    DEFINE_I2C_LOG(log_debug, "DBG - ")
#undef DEFINE_I2C_LOG

    [[nodiscard]] virtual const char* name() const = 0;

protected:
    // return # of bytes written, < 0 if error
    [[nodiscard]] virtual int write(uint8_t addr, uint8_t const* src, size_t len) = 0;
    // return # of bytes read, < 0 if error
    [[nodiscard]] virtual int read(uint8_t addr, uint8_t* dst, size_t len) = 0;

private:
    SemaphoreHandle_t lock;
};

}  // namespace nevermore
