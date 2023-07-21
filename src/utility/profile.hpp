#pragma once

#include "scope_guard.hpp"
#include <chrono>
#include <cstdio>

namespace nevermore {

#define DBG_PROFILE_EXPR(desc, unit, expr) \
    DBG_profile_time(desc " %d " #unit "\n", 1##unit, [&]() { return expr; })

template <typename F, typename Dur>
auto DBG_profile_time(char const* msg, Dur const& unit, F&& go) {
    auto bgn = std::chrono::steady_clock::now();
    SCOPE_GUARD {
        auto end = std::chrono::steady_clock::now();
        printf(msg, int((end - bgn) / unit));
    };

    return go();
}

}  // namespace nevermore
