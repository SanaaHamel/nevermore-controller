#pragma once

#include "sdk/ble_data_types.hpp"
#include <cstdint>

extern "C" {
struct GasIndexAlgorithmParams;
}

namespace nevermore::sensors {

// FUTURE WORK: promote to fix16_t to have decimal components
BLE_DECL_SCALAR_OPTIONAL(GIA_Mean, uint16_t, 1, 0, 0, 0xFFFFu);      // range [0, 2^16-2], 0xFFFF = not-known;
BLE_DECL_SCALAR_OPTIONAL(GIA_Variance, uint16_t, 1, 0, 0, 0xFFFFu);  // range [0, 2^16-2], 0xFFFF = not-known;
BLE_DECL_SCALAR_OPTIONAL(GIA_Threshold, uint16_t, 1, 0, 0,
        0xFFFFu);  // range [0, 2**16-2], 0xFFFF = not-known; basically a redef of VOC Index
// we're wasting 1 bit of space (1.0 => 0x8000), but that simplifies the code a lot for handling
// the edge case of 1.0
BLE_DECL_SCALAR_OPTIONAL(GIA_Sigmoid, uint16_t, 1, 0, -15,
        0xFFFFu);  // range [0, 1], 0xFFFF = not-known; the low 16 of fix16_t

struct [[gnu::packed]] GIAState {
    GIAState() = default;
    GIAState(GasIndexAlgorithmParams const&);

    GIA_Mean mean;
    GIA_Variance var;
    GIA_Sigmoid gamma_mean;
    GIA_Sigmoid gamma_var;
    GIA_Sigmoid gating_mean;
    GIA_Sigmoid gating_var;
    GIA_Threshold threshold_gating_mean;
    GIA_Threshold threshold_gating_var;

    auto operator<=>(GIAState const&) const = default;
};

}  // namespace nevermore::sensors
