#pragma once

#include "sdk/ble_data_types.hpp"
#include "sensors.hpp"
#include "sensors/gas_index_ble.hpp"
#include <tuple>
#include <type_traits>
#include <utility>

#if DBG_MEASURE_VOC_TEMPERATURE_HUMIDITY_EFFECT
#include <cstdio>
#endif

namespace nevermore::sensors {

struct EnvironmentalFilter {
    enum class Kind { Intake, Exhaust };
    Kind kind;

    EnvironmentalFilter(Kind kind) : kind(kind) {}

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

#if DBG_MEASURE_VOC_TEMPERATURE_HUMIDITY_EFFECT
        if constexpr (std::is_same_v<A, VOCRaw>) {
            dbg_voc_breakdown_state = (dbg_voc_breakdown_state + 1) % 4;
            switch (dbg_voc_breakdown_state) {
            case 0: {
                std::get<VOCRawBreakdown&>(main).uncompensated = x;
                dbg_print_voc_breakdown(main);
                return;
            }
            case 1: break;  // update standard VOC-raw value
            case 2: std::get<VOCRawBreakdown&>(main).humidity = x; return;
            case 3: std::get<VOCRawBreakdown&>(main).temperature = x; return;
            }
        }
#endif
        std::get<A&>(main) = x;
    }

    [[nodiscard]] double compensation_temperature(
            Sensors const& sensors = g_sensors, Config const& config = g_config) const;
    [[nodiscard]] double compensation_humidity(
            Sensors const& sensors = g_sensors, Config const& config = g_config) const;

    // PRECONDITION: called immediately after `set<VOCRaw>`
    [[nodiscard]] bool was_voc_breakdown_measurement() const {
#if DBG_MEASURE_VOC_TEMPERATURE_HUMIDITY_EFFECT
        return dbg_voc_breakdown_state != 1;
#else
        return false;
#endif
    }

private:
#if DBG_MEASURE_VOC_TEMPERATURE_HUMIDITY_EFFECT
    using Side = std::tuple<BLE::Temperature&, BLE::Humidity&, BLE::Pressure&, VOCIndex&, VOCRaw&, GIAState&,
            VOCRawBreakdown&>;
    uint8_t dbg_voc_breakdown_state = 0;  // 0 => measuring full compensation, 3 => no comp

    static void dbg_print_voc_breakdown(Side const& side) {
        int f = std::get<VOCRaw&>(side).raw_value;
        int h = std::get<VOCRawBreakdown&>(side).humidity.raw_value;
        int t = std::get<VOCRawBreakdown&>(side).temperature.raw_value;
        int n = std::get<VOCRawBreakdown&>(side).uncompensated.raw_value;
        printf("VOC Breakdown - both=% 7d  humid=% 7d  temp=% 7d  none=% 7d\n", f, h, t, n);
        printf("                              humid=% 7d  temp=% 7d  none=% 7d\n", h - f, t - f, n - f);
    }
#else
    using Side = std::tuple<BLE::Temperature&, BLE::Humidity&, BLE::Pressure&, VOCIndex&, VOCRaw&, GIAState&>;
#endif

    template <typename A>
        requires(!std::is_reference_v<A>)
    [[nodiscard]] A get_(Sensors const& sensors = g_sensors, Config const& config = g_config) const {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        auto [main, other] = pick(const_cast<Sensors&>(sensors));
        if constexpr (BLE::has_not_known<A>) {
            if (auto value = std::get<A&>(main); value != BLE::NOT_KNOWN) return value;
            if (auto value = std::get<A&>(other); config.fallback) return value;
            return BLE::NOT_KNOWN;
        } else {
            return std::get<A&>(main);
        }
    }

    [[nodiscard]] std::tuple<Side, Side> pick(Sensors& sensors = g_sensors) const {
#if DBG_MEASURE_VOC_TEMPERATURE_HUMIDITY_EFFECT
        Side intake{sensors.temperature_intake, sensors.humidity_intake, sensors.pressure_intake,
                sensors.voc_index_intake, sensors.voc_raw_intake, sensors.gia_intake,
                sensors.voc_raw_breakdown_intake};
        Side exhaust{sensors.temperature_exhaust, sensors.humidity_exhaust, sensors.pressure_exhaust,
                sensors.voc_index_exhaust, sensors.voc_raw_exhaust, sensors.gia_exhaust,
                sensors.voc_raw_breakdown_exhaust};
#else
        Side intake{sensors.temperature_intake, sensors.humidity_intake, sensors.pressure_intake,
                sensors.voc_index_intake, sensors.voc_raw_intake, sensors.gia_intake};
        Side exhaust{sensors.temperature_exhaust, sensors.humidity_exhaust, sensors.pressure_exhaust,
                sensors.voc_index_exhaust, sensors.voc_raw_exhaust, sensors.gia_exhaust};
#endif

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
    auto fallback = [&]() { return 20.; };

#if DBG_MEASURE_VOC_TEMPERATURE_HUMIDITY_EFFECT
    switch (dbg_voc_breakdown_state) {
    case 1:                     // FALL-THRU: humidity
    case 3: return fallback();  // none
    default: break;
    }
#endif

    return get<BLE::Temperature>(sensors, config).value_or(sensors.temperature_mcu.value_or(fallback()));
}

inline double EnvironmentalFilter::compensation_humidity(Sensors const& sensors, Config const& config) const {
    // assume 25% humidity, which is not unreasonable in a hot printer
    // TODO: make fallback value based off temperature? is it worth the trouble?
    auto fallback = [&]() { return 25.; };

#if DBG_MEASURE_VOC_TEMPERATURE_HUMIDITY_EFFECT
    switch (dbg_voc_breakdown_state) {
    case 2:                     // FALL-THRU: temperature
    case 3: return fallback();  // none
    default: break;
    }
#endif

    return get<BLE::Humidity>(sensors, config).value_or(fallback());
}

}  // namespace nevermore::sensors
