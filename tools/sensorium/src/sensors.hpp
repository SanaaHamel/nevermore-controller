#pragma once

#include <cassert>
#include <chrono>
#include <optional>

namespace nevermore::sensorium::sensors {

struct EnvState {
    std::optional<float> _temperature;
    std::optional<float> _humidity;

    [[nodiscard]] float temperature() const {
        return _temperature.value_or(25.f);
    }

    // range: [0, 100]
    [[nodiscard]] float humidity() const {
        auto value = _humidity.value_or(50.f);
        assert(0 <= value && value <= 100);
        return value;
    }

    [[nodiscard]] EnvState or_else(EnvState const& x) const {
        return {
                ._temperature = _temperature.or_else([&]() { return x._temperature; }),
                ._humidity = _humidity.or_else([&]() { return x._humidity; }),
        };
    }
};

bool init();
void poll_issue(EnvState const&);
EnvState poll_readback();
std::chrono::milliseconds poll_readback_delay();

}  // namespace nevermore::sensorium::sensors
