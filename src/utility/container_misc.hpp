#pragma once

#include <algorithm>

namespace nevermore {

template <typename A>
constexpr auto contains(A const& xs) {
    return [=](auto const& x) { return std::find(begin(xs), end(xs), x) != end(xs); };
}

}  // namespace nevermore
