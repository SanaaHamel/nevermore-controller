#pragma once

#include "pico/async_context.h"
#include "sdk/ble_data_types.hpp"
#include <cstdint>

namespace nevermore::sensors {

BLE_DECL_SCALAR_OPTIONAL(VOCIndex, uint16_t, 1, 0, 0, 0);  // range [0, 500], 0 = not-known;

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

    auto operator<=>(Sensors const&) const = default;
};

extern Sensors g_sensors;

// Sensors are registered as periodic workers for the context.
bool init(async_context_t&);

}  // namespace nevermore::sensors
