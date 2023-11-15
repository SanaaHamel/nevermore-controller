#pragma once

#include <chrono>
#include <cmath>
#include <numeric>

// Toy helpers for debugging/testing. Not intended for general use.

namespace nevermore {

template <typename A>
constexpr A remap(A const& x, A const& min, A const& max, A const& lo, A const& hi) {
    auto p = (x - min) / (max - min);
    return std::lerp(lo, hi, p);
}

double time_sin(std::chrono::duration<float> period) {
    auto t = std::chrono::steady_clock::now().time_since_epoch() / period;
    return remap<double>(std::sin(t * 2 * M_PI), -1, 1, 0, 1);
}

double time_saw(std::chrono::duration<float> period) {
    return std::fmod(std::chrono::steady_clock::now().time_since_epoch() / std::chrono::seconds(1),
                   period.count()) /
           period.count();
}

}  // namespace nevermore
