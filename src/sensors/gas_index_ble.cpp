#include "gas_index_ble.hpp"
#include "lib/sensirion_gas_index_algorithm.h"

namespace nevermore::sensors {

namespace {

double fix16_frac(fix16_t x) {
    return x / double(1 << 16);
}

}  // namespace

GIAState::GIAState(GasIndexAlgorithmParams const& x) {
    if (x.mGas_Index == 0) return;  // wait until we have some state

    mean = GasIndexAlgorithm_sraw_mean(&x);
    var = GasIndexAlgorithm_sraw_std(&x);
    gamma_mean = fix16_frac(x._sigmoid_gamma_mean);
    gamma_var = fix16_frac(x._sigmoid_gamma_variance);
    gating_mean = fix16_frac(x._sigmoid_gating_mean);
    gating_var = fix16_frac(x._sigmoid_gating_variance);
    threshold_gating_mean = fix16_cast_to_int(x._gating_threshold_mean);
    threshold_gating_var = fix16_cast_to_int(x._gating_threshold_variance);
}

}  // namespace nevermore::sensors
