#pragma once

#include <utility>

namespace nevermore {

template <typename F>
class ScopeGuard {
    F f;
    bool live;

public:
    ScopeGuard() = delete;
    ScopeGuard(const ScopeGuard&) = delete;
    ScopeGuard& operator=(const ScopeGuard&) = delete;

    ScopeGuard(ScopeGuard&& rhs) noexcept : f(std::move(rhs.f)), live(rhs.live) {
        rhs.live = false;
    }

    ScopeGuard(F f) : f(std::move(f)), live(true) {}

    ~ScopeGuard() {
        if (live) f();
    }
};

enum class ScopeGuardBuilder {};
template <typename F>
auto operator+(ScopeGuardBuilder, F&& f) {
    return ScopeGuard{std::forward<F>(f)};
}

#define SCOPE_GUARD_ANON_VAR_CONCAT_(x, y) x##y
#define SCOPE_GUARD_ANON_VAR_CONCAT(x, y) SCOPE_GUARD_ANON_VAR_CONCAT_(x, y)
#define SCOPE_GUARD auto SCOPE_GUARD_ANON_VAR_CONCAT(scope_guard__, __COUNTER__) = ScopeGuardBuilder{} + [&]()

}  // namespace nevermore
