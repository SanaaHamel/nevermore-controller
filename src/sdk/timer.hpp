#pragma once

#include "hardware/timer.h"
#include <chrono>

using namespace std::literals::chrono_literals;

inline std::chrono::microseconds time_64u() {
    return std::chrono::microseconds{time_us_64()};  // no worries about unsigned
}

template <typename T, typename U>
void busy_wait(std::chrono::duration<T, U> dur) {
    busy_wait_us(dur / 1us);
}

// lower power than `busy_wait`, but you can't use it during interrupts/exceptions
template <typename T, typename U>
void sleep(std::chrono::duration<T, U> dur) {
    sleep_us(dur / 1us);
}
