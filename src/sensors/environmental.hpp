#pragma once

#include "sensors.hpp"

namespace nevermore::sensors {

using EnvironmentalSensorData =
        std::tuple<BLE::Temperature&, BLE::Humidity&, BLE::Pressure&, nevermore::sensors::VOCIndex&>;

}  // namespace nevermore::sensors
