#pragma once

#include "bluetooth.h"
#include "sdk/ble_data_types.hpp"
#include <cstdint>
#include <optional>

namespace EnvironmentService {

BLE_DECL_SCALAR_OPTIONAL(VOCIndex, uint16_t, 1, 0, 0, 0);  // range [0, 500], 0 = not-known;

// must match declared order of characteristics in environmental service b/c this
// is copied/presented in the advertised Service Data
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

// returns none if not handled
std::optional<uint16_t> attr_read(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size);

std::optional<int> attr_write(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t const* buffer, uint16_t buffer_size);

void init();
void disconnected(hci_con_handle_t);

}  // namespace EnvironmentService
