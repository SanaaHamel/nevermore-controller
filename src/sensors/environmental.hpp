#pragma once

#include "sdk/ble_data_types.hpp"
#include "sensors.hpp"
#include <tuple>
#include <type_traits>
#include <utility>

namespace nevermore::sensors {

struct EnvironmentalFilter {
    enum class Kind { Intake, Exhaust };
    Kind kind;

    // The Right Thing(TM) would be to have refs to config/service-data.
    // For now, just use `EnvironmentalService::g_sensors` and `EnvironmentalService::g_config`.

    template <typename A>
        requires(!std::is_reference_v<A>)
    [[nodiscard]] A get(Sensors const& sensors = g_sensors, Config const& config = g_config) const {
        return get_<A>(sensors, config);
    }

    template <typename A>
    void set(A x, Sensors& sensors = g_sensors) {
        auto [main, _] = pick(sensors);
        std::get<A&>(main) = x;
    }

    [[nodiscard]] double compensation_temperature(
            Sensors const& sensors = g_sensors, Config const& config = g_config) const;
    [[nodiscard]] double compensation_humidity(
            Sensors const& sensors = g_sensors, Config const& config = g_config) const;

private:
    using Side = std::tuple<BLE::Temperature&, BLE::Humidity&, BLE::Pressure&, VOCIndex&, VOCRaw&>;

    template <typename A>
        requires(!std::is_reference_v<A>)
    [[nodiscard]] A get_(Sensors const& sensors = g_sensors, Config const& config = g_config) const {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        auto [main, other] = pick(const_cast<Sensors&>(sensors));
        if (auto value = std::get<A&>(main); value != BLE::NOT_KNOWN) return value;
        if (auto value = std::get<A&>(other); config.fallback) return value;
        return BLE::NOT_KNOWN;
    }

    [[nodiscard]] std::tuple<Side, Side> pick(Sensors& sensors = g_sensors) const {
        Side intake{sensors.temperature_intake, sensors.humidity_intake, sensors.pressure_intake,
                sensors.voc_index_intake, sensors.voc_raw_intake};
        Side exhaust{sensors.temperature_exhaust, sensors.humidity_exhaust, sensors.pressure_exhaust,
                sensors.voc_index_exhaust, sensors.voc_raw_exhaust};

        switch (kind) {
        case Kind::Intake: return {intake, exhaust};
        case Kind::Exhaust: return {exhaust, intake};
        }

        std::unreachable();  // stupid lack of case analysis
    }
};

// special: exhaust can prefer to fall back too the MCU temperature (always known) instead of intake
template <>
inline BLE::Temperature EnvironmentalFilter::get<BLE::Temperature>(
        Sensors const& sensors, Config const& config) const {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    auto [main, other] = pick(const_cast<Sensors&>(sensors));
    if (auto value = std::get<BLE::Temperature&>(main); value != BLE::NOT_KNOWN) return value;
    // Exhaust falls back to MCU first, if enabled
    if (g_config.fallback_exhaust_mcu && kind == Kind::Exhaust) return sensors.temperature_mcu;
    // No other fallbacks allowed
    if (!g_config.fallback) return BLE::NOT_KNOWN;
    // Fall back to other side
    if (auto value = std::get<BLE::Temperature&>(other); value != BLE::NOT_KNOWN) return value;
    // we're intake, have no value, and neither does exhaust -> double fallback to MCU
    if (g_config.fallback_exhaust_mcu) return sensors.temperature_mcu;
    return BLE::NOT_KNOWN;
}

inline double EnvironmentalFilter::compensation_temperature(
        Sensors const& sensors, Config const& config) const {
    return get<BLE::Temperature>(sensors, config).value_or(sensors.temperature_mcu.value_or(20));
}

inline double EnvironmentalFilter::compensation_humidity(Sensors const& sensors, Config const& config) const {
    // assume 25% humidity, which is not unreasonable in a hot printer
    // TODO: make fallback value based off temperature? is it worth the trouble?
    return get<BLE::Humidity>(sensors, config).value_or(25);
}

}  // namespace nevermore::sensors
