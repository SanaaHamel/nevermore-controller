#pragma once

#include "FreeRTOS.h"
#include <cassert>

// A horrible hack to keep nevermore-controller stolen files happy

inline void vTaskDelay(TickType_t) {
    assert(false);
}
