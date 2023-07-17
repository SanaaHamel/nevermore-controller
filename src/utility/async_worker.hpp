#pragma once

#include "pico/async_context.h"
#include "sdk/async.hpp"
#include <cassert>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <ratio>
#include <utility>

namespace async_worker {

template <typename F, typename... Args>
concept Invocable = requires { std::is_nothrow_invocable_v<F, Args...>; };

using Delay = std::chrono::duration<uint32_t, std::milli>;

template <Invocable F>
struct Worker

{
    Delay delay;

    Worker(Worker const&) = delete;
    Worker& operator=(Worker const&) = delete;

    Worker(F closure, Delay delay) : delay(delay), closure(std::move(closure)) {
        worker.user_data = reinterpret_cast<void*>(this);
        worker.do_work = [](async_context_t* context, async_at_time_worker_t* worker) {
            auto& self = *reinterpret_cast<Worker*>(worker->user_data);
            self.context = nullptr;  // we've completed
            self.closure();
            self.register_(*context, self.delay);
        };
    }

    ~Worker() {
        unregister();
    }

    void register_(async_context_t& context) {
        register_(context, delay);
    }

    void register_(async_context_t& context, Delay delay) {
        assert(!this->context && "already registered");
        if (this->context) return;

        async_using(context, [&]() {
            bool ok = async_context_add_at_time_worker_in_ms(&context, &worker, delay.count());
            assert(ok && "failed to register worker");
            if (ok) {
                this->context = &context;
            }
        });
    }

    // return `true` if unregistered (`false` if it wasn't registered)
    bool unregister() {
        if (!context) return false;

        async_using(*context, [&]() {
            [[maybe_unused]] bool ok = async_context_remove_at_time_worker(context, &worker);
            context = nullptr;
            assert(ok && "INVARIANT: `context != nullptr` <=> registered");
        });
        return true;
    }

private:
    F closure;
    async_context_t* context{};  // INVARIANT: non-null <=> registered
    async_at_time_worker_t worker{};
};

}  // namespace async_worker

template <typename A, typename Ratio>
consteval auto mk_async_worker(std::chrono::duration<A, Ratio> delay) {
    using namespace async_worker;

    auto delay_ms = std::chrono::duration_cast<std::chrono::duration<int64_t, Delay::period>>(delay);
    if (delay_ms.count() <= 0) throw "invalid delay value";
    if (std::numeric_limits<Delay::rep>::max() < delay_ms.count()) throw "invalid delay value";

    return [=]<Invocable F>(F go) { return Worker(std::move(go), delay_ms); };
}
