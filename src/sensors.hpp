#pragma once

#include "sdk/ble_data_types.hpp"
#include "sensors/gas_index_ble.hpp"
#include <cstdint>

#define DBG_MEASURE_VOC_TEMPERATURE_HUMIDITY_EFFECT 0

namespace nevermore::sensors {

BLE_DECL_SCALAR_OPTIONAL(VOCIndex, uint16_t, 1, 0, 0, 0);      // range [0, 500], 0 = not-known;
BLE_DECL_SCALAR_OPTIONAL(VOCRaw, uint16_t, 1, 0, 0, 0xFFFFu);  // range [0, 2^16-2], 0xFFFF = not-known;

#if DBG_MEASURE_VOC_TEMPERATURE_HUMIDITY_EFFECT

struct [[gnu::packed]] VOCRawBreakdown {
    VOCRaw humidity;
    VOCRaw temperature;
    VOCRaw uncompensated;
    auto operator<=>(VOCRawBreakdown const&) const = default;
};

#undef NEWTYPE_VOC_RAW

#endif

struct Config {
    // If a sensor in a `FilterSide` is missing, then try to fall back to the other side's sensor.
    bool fallback = false;
    // StealthMax MCU is positioned inside the exhaust airflow.
    // Disabled by default because not all Nevermores are StealthMaxes.
    bool fallback_exhaust_mcu = false;
};

extern Config g_config;

// Requirements:
// * Must match declared order of characteristics in environmental service
// * Must be packed.
struct [[gnu::packed]] Sensors {
    BLE::Temperature temperature_intake;
    BLE::Temperature temperature_exhaust;
    BLE::Temperature temperature_mcu;
    BLE::Humidity humidity_intake;
    BLE::Humidity humidity_exhaust;
    // Officially there's no not-known constant for pressure. We define an unofficial one in BLE data types.
    BLE::Pressure pressure_intake;
    BLE::Pressure pressure_exhaust;
    VOCIndex voc_index_intake;
    VOCIndex voc_index_exhaust;
    VOCRaw voc_raw_intake;
    VOCRaw voc_raw_exhaust;
    GIAState gia_intake;
    GIAState gia_exhaust;

#if DBG_MEASURE_VOC_TEMPERATURE_HUMIDITY_EFFECT
    VOCRawBreakdown voc_raw_breakdown_intake;
    VOCRawBreakdown voc_raw_breakdown_exhaust;
#endif

    [[nodiscard]] Sensors with_fallbacks(Config const& config = g_config) const;

    auto operator<=>(Sensors const&) const = default;
};

extern Sensors g_sensors;

// Sensors are registered as periodic workers for the context.
bool init();
void reset_calibrations();

}  // namespace nevermore::sensors
