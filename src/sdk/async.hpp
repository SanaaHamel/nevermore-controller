#pragma once

#include "pico/async_context.h"
#include "utility/scope_guard.hpp"

namespace nevermore {

template <typename F>
auto async_using(async_context_t& context, F&& go) {
    async_context_acquire_lock_blocking(&context);
    SCOPE_GUARD {
        async_context_release_lock(&context);
    };

    return go();
}

}  // namespace nevermore
