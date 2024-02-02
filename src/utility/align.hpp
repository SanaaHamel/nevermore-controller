#pragma once

#include <type_traits>

namespace nevermore {

template <typename A>
constexpr A align(A x, A a)
    requires std::is_unsigned_v<A>
{
    return (x + (a - 1)) & ~(a - 1);
}

}  // namespace nevermore
