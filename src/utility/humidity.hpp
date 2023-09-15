#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

namespace nevermore::humidity {

// pressure saturation in Pa
// drawn from DOI:10.1063/1.1461829
template <typename N>
constexpr N h2o_vapor_saturation(N const temperature_k) {
    constexpr N P_c = 22.064 * 1'000'000;  // ~~MPa~~ Pa, critical pressure for H2O
    constexpr N T_c = 647.096;             // K, critical temperature for H2O
    constexpr std::pair<N, N> xs[]{
            // empirical constants
            {-7.85951783, 1},
            {1.84408259, 1.5},
            {-11.7866497, 3},
            {22.6807411, 3.5},
            {-15.9618719, 4},
            {1.80122502, 7.5},
    };

    N tau = 1 - (temperature_k / T_c);
    N sum = 0;
    for (auto [a, e] : xs)
        sum += a * std::pow(tau, e);

    sum *= (T_c / temperature_k);
    return P_c * std::exp(sum);
}

// in in g/m^3
template <typename N>
constexpr N absolute(N const humidity_relative, N const temperature_c) {
    constexpr N R_w = 461.5 / 1000;  // J / (g K), specific gas constant for H2O vapor

    N RH = std::clamp<N>(humidity_relative, 0, 100);
    N T = std::max<N>(temperature_c + 273.15, 0);
    N P_s = h2o_vapor_saturation(T);    // Pa
    return RH * P_s / (R_w * T * 100);  // g / m^3
}

// Taken from SGP30's driver implementation reference.
// Looks to be within ~1% error relative to `absolute`.
// in in g/m^3
template <typename N>
constexpr N absolute_fast(N const humidity_relative, N const temperature_c) {
    N ah = humidity_relative / 100;
    N K = N(273.15) + temperature_c;
    return N(216.7 * 6.112) * ah * exp((N(17.62) * temperature_c) / (N(243.12) + temperature_c)) / K;
}

}  // namespace nevermore::humidity
