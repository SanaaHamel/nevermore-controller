#pragma once

#include "pico/async_context.h"
#include "sdk/async.hpp"
#include <cassert>
#include <chrono>
#include <cstdint>

template <uint32_t DELAY_MS, typename A>
struct AsyncWorker {
    using FnPtr = A (*)();

    AsyncWorker(FnPtr go) {
        worker.user_data = reinterpret_cast<void*>(go);
        worker.do_work = [](async_context_t* context, async_at_time_worker_t* worker) {
            reinterpret_cast<FnPtr>(worker->user_data)();
            register_(*context, *worker);
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

    void register_(async_context_t& context,
            std::chrono::milliseconds initial_delay = std::chrono::milliseconds(DELAY_MS)) {
        register_(context, worker, initial_delay / std::chrono::milliseconds(1));
    }

private:
    async_at_time_worker_t worker{};

    static void register_(async_context_t& context, async_at_time_worker_t& worker, uint32_t ms = DELAY_MS) {
        async_using(context, [&]() { async_context_add_at_time_worker_in_ms(&context, &worker, ms); });
    }
};

template <uint32_t DELAY_MS, typename A>
auto mk_sync_worker(A (*go)()) {
    return AsyncWorker<DELAY_MS, decltype(A())>(go);
}

template <uint32_t DELAY_MS>
auto mk_async_worker(void (*go)()) {
    return AsyncWorker<DELAY_MS, void>(go);
}
