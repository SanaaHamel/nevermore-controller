#pragma once

#include <algorithm>
#include <cstring>
#include <initializer_list>
#include <utility>

namespace nevermore {

template <typename... A>
    requires(std::is_trivially_copyable_v<A> && ...)
class [[gnu::packed]] PackedTuple final {
    template <typename T>
    constexpr static size_t partial_sum_exclusive(size_t N, std::initializer_list<T> xs) {
        auto it = xs.begin();
        N = std::min<size_t>(N, xs.end() - it);
        T total = 0;
        for (size_t i = 0; i < N; ++i)
            total += *it++;
        return total;
    }

    template <size_t I>
    using type = typename std::tuple_element_t<I, PackedTuple>;

    template <size_t I>
    constexpr static size_t offset = partial_sum_exclusive(I, {sizeof(A)...});

    template <size_t I, typename X>
    void initialise(X x) {
        set<I>(std::forward<X>(x));
    }

    template <size_t I, typename X, typename... XS>
    void initialise(X&& x, XS&&... xs) {
        set<I>(std::forward<X>(x));
        initialise<I + 1>(std::forward<XS>(xs)...);
    }

    uint8_t data[(sizeof(A) + ...)];

public:
    // spurious static analysis error: this does initialise all of `data`
    PackedTuple() : PackedTuple(A{}...) {}  // NOLINT(cppcoreguidelines-pro-type-member-init)

    // GCC fucks this up beautify and starts thinking `initialise` is a static member func invoke.
    // IDK why. Whatever, the less general version works.
    // template <typename... XS>
    // PackedTuple(XS&&... xs) {
    //     initialise<0>(std::forward<XS>(xs)...);
    //     initialise<sizeof...(xs)
    // }

    // spurious static analysis error: this does initialise all of `data`
    PackedTuple(A const&... xs) {  // NOLINT(cppcoreguidelines-pro-type-member-init)
        initialise<0>(xs...);
    }

    template <size_t I>
    auto& get() const noexcept {
        return *reinterpret_cast<type<I> const*>(data + offset<I>);
    }

    template <size_t I>
    auto& get() noexcept {
        return *reinterpret_cast<type<I>*>(data + offset<I>);
    }

    template <size_t I>
    void set(type<I> const& x) noexcept {
        std::memcpy(data + offset<I>, &x, sizeof(x));
    }
};

template <size_t I, typename... A>
auto& get(PackedTuple<A...>& xs) noexcept {
    return xs.template get<I>();
}

template <size_t I, typename... A>
auto& get(PackedTuple<A...> const& xs) noexcept {
    return xs.template get<I>();
}

}  // namespace nevermore

template <typename... A>
struct std::tuple_size<nevermore::PackedTuple<A...>> : std::integral_constant<std::size_t, sizeof...(A)> {};

template <size_t I, typename A, typename... B>
struct std::tuple_element<I, nevermore::PackedTuple<A, B...>>
        : std::tuple_element<I - 1, nevermore::PackedTuple<B...>> {};

template <typename A, typename... B>
struct std::tuple_element<0, nevermore::PackedTuple<A, B...>> {
    using type = A;
};
