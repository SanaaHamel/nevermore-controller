#pragma once

#include "gatt/environmental.hpp"
#include "sdk/ble_data_types.hpp"
#include <tuple>

using EnvironmentalSensorData =
        std::tuple<BLE::Temperature&, BLE::Humidity&, BLE::Pressure&, EnvironmentService::VOCIndex&>;
