#pragma once

#include <cstdint>

// A horrible hack to keep nevermore-controller stolen files happy

#define configTICK_RATE_HZ 1000
#define portMAX_DELAY 0xFFFF'FFFF
#define pdMS_TO_TICKS(ms) 0

using TickType_t = uint32_t;
