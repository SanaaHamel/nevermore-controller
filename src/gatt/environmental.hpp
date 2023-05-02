#pragma once

#include "bluetooth.h"
#include "sdk/ble_data_types.hpp"
#include <cstdint>
#include <optional>

namespace EnvironmentService {

BLE_DECLARE_SCALAR_TYPE(VOCIndex, uint16_t, 1, 0, 0);  // range [0, 500], 0 = not-known;

constexpr VOCIndex VOC_INDEX_NOT_KNOWN = VOCIndex::from_raw(0);

// must match declared order of characteristics in environmental service b/c this
// is copied/presented in the advertised Service Data
struct [[gnu::packed]] ServiceData {
    BLE::Temperature temperature_intake{BLE::NOT_KNOWN};
    BLE::Temperature temperature_exhaust{BLE::NOT_KNOWN};
    BLE::Temperature temperature_mcu{BLE::NOT_KNOWN};
    BLE::Humidity humidity_intake{BLE::NOT_KNOWN};
    BLE::Humidity humidity_exhaust{BLE::NOT_KNOWN};
    // sadly there is no official way of reporting not-a-valid-value
    BLE::Pressure pressure_intake = BLE::PRESSURE_1_ATMOSPHERE;
    BLE::Pressure pressure_exhaust = BLE::PRESSURE_1_ATMOSPHERE;
    VOCIndex voc_index_intake = VOC_INDEX_NOT_KNOWN;
    VOCIndex voc_index_exhaust = VOC_INDEX_NOT_KNOWN;
};

// returns none if not handled
std::optional<uint16_t> attr_read(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size);

std::optional<int> attr_write(
        hci_con_handle_t, uint16_t att_handle, uint16_t offset, uint8_t const* buffer, uint16_t buffer_size);

}  // namespace EnvironmentService
