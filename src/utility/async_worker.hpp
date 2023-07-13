#pragma once

#include "pico/async_context.h"
#include "sdk/async.hpp"
#include <cassert>
#include <chrono>
#include <cstdint>
#include <ratio>

template <typename A>
struct AsyncWorker {
    using Delay = std::chrono::duration<uint32_t, std::milli>;
    using FnPtr = A (*)();

    Delay delay;

    AsyncWorker(FnPtr go, Delay delay) : delay(delay), go(go) {
        worker.user_data = reinterpret_cast<void*>(this);
        worker.do_work = [](async_context_t* context, async_at_time_worker_t* worker) {
            auto& self = *reinterpret_cast<AsyncWorker*>(worker->user_data);
            self.go();
            register_(*context, *worker, self.delay);
        };
    }

    AsyncWorker(AsyncWorker const&) = delete;
    AsyncWorker& operator=(AsyncWorker const&) = delete;
    ~AsyncWorker() {
        assert(false && "Cannot be destroyed.");
    }

    void trigger(async_context_t& context) {
        worker.do_work(&context, &worker);
    }

    void register_(async_context_t& context) {
        register_(context, delay);
    }

    void register_(async_context_t& context, Delay initial_delay) {
        register_(context, worker, initial_delay);
    }

private:
    FnPtr go;
    async_at_time_worker_t worker{};

    static void register_(async_context_t& context, async_at_time_worker_t& worker, Delay ms) {
        async_using(
                context, [&]() { async_context_add_at_time_worker_in_ms(&context, &worker, ms.count()); });
    }
};

template <uint32_t DELAY_MS, typename A>
auto mk_async_worker(A (*go)()) {
    return AsyncWorker<decltype(A())>(go, std::chrono::milliseconds(DELAY_MS));
}

template <uint32_t DELAY_MS>
auto mk_async_worker(void (*go)()) {
    return AsyncWorker<void>(go, std::chrono::milliseconds(DELAY_MS));
}
