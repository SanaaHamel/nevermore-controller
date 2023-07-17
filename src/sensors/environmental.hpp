#pragma once

#include "sensors.hpp"

using EnvironmentalSensorData =
        std::tuple<BLE::Temperature&, BLE::Humidity&, BLE::Pressure&, nevermore::sensors::VOCIndex&>;
