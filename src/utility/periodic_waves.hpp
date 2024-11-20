#pragma once

#include <chrono>
#include <cmath>
#include <numbers>

namespace nevermore::periodic_waves {

template <typename A>
constexpr A remap(A const& x, A const& min, A const& max, A const& lo, A const& hi) {
    auto p = (x - min) / (max - min);
    return std::lerp(lo, hi, p);
}

inline float sin(std::chrono::duration<float> period,
        std::chrono::steady_clock::duration now = std::chrono::steady_clock::now().time_since_epoch()) {
    return remap<float>(std::sin(float(now / period) * 2 * std::numbers::pi_v<float>), -1, 1, 0, 1);
}

inline float saw(std::chrono::duration<float> period,
        std::chrono::steady_clock::duration now = std::chrono::steady_clock::now().time_since_epoch()) {
    return std::fmod(std::chrono::duration_cast<std::chrono::duration<float>>(now).count(), period.count()) /
           period.count();
}

inline float square(std::chrono::duration<float> period,
        std::chrono::steady_clock::duration now = std::chrono::steady_clock::now().time_since_epoch()) {
    return std::floor(saw(period, now) * 2);
}

}  // namespace nevermore::periodic_waves
