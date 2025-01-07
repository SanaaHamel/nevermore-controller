#pragma once

#include <cstdint>

// A horrible hack to keep nevermore-controller stolen files happy
struct SemaphoreHandle_t {};

inline SemaphoreHandle_t xSemaphoreCreateMutex() {
    return {};
}

inline void vSemaphoreDelete(SemaphoreHandle_t) {}

inline bool xSemaphoreTake(SemaphoreHandle_t, uint32_t) {
    return true;
}

inline void xSemaphoreGive(SemaphoreHandle_t) {}
