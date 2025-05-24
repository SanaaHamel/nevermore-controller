#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <type_traits>

namespace nevermore {

template <typename N>
    requires(std::is_arithmetic_v<N>)
struct PID {
    using Sec = std::chrono::duration<N>;

    N k_p;
    N k_i;
    N k_d;

    // helper for typical standard form
    static constexpr PID mk(N k_p, Sec t_i, Sec t_d) {
        return {
                .k_p = k_p,
                .k_i = k_p / t_i.count(),
                .k_d = k_p * t_d.count(),
        };
    }

    template <N min_ = N(0), N max_ = N(1)>
    struct StateMinimal {
        static constexpr N min = min_;
        static constexpr N max = max_;

        constexpr N update(PID const& pid, N const e, N const ie, N const de) {
            N const curr_ie = std::clamp(_prev_ie + (pid.k_i * ie), min, max);
            N const o = (pid.k_p * e) + curr_ie + (pid.k_d * de);

            _prev_ie = curr_ie;
            return std::clamp(o, min, max);
        }

    private:
        N _prev_ie = 0;
    };

    template <N min_ = N(0), N max_ = N(1)>
    struct State : StateMinimal<min_, max_> {
        constexpr N update(PID const& pid, Sec const dt, N const current, N const target) {
            if (!_init) {
                _init = true;
                _prev = current;
            }

            N const e = target - current;
            N const ie = e * dt.count();
            N const de = (current - _prev) / dt.count();
            _prev = current;
            return StateMinimal<min_, max_>::update(pid, e, ie, de);
        }

    private:
        N _prev = 0;
        bool _init = false;
    };
};

}  // namespace nevermore
