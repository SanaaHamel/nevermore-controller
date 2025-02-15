#pragma once

#include "lib/sensirion_gas_index_algorithm.h"
#include "sensors.hpp"
#include "sensors/environmental.hpp"
#include "settings.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstring>

namespace nevermore::sensors {

struct GasIndex {
    using Clock = std::chrono::steady_clock;

    static constexpr auto CHECKPOINT_PERIOD = 24h;

    GasIndexAlgorithmParams gia{};
    Clock::time_point next_checkpoint = Clock::now() + CHECKPOINT_PERIOD;

    GasIndex(int32_t type = GasIndexAlgorithm_ALGORITHM_TYPE_VOC) {
        assert(type == GasIndexAlgorithm_ALGORITHM_TYPE_VOC || type == GasIndexAlgorithm_ALGORITHM_TYPE_NOX);
        GasIndexAlgorithm_init(&gia, type);
    }

    void process_and_checkpoint(EnvironmentalFilter side, auto& log, int32_t raw,
            settings::Settings const& settings = settings::g_active) {
        auto _ = side.guard();
        side.set(VOCRaw(raw));
        if (side.was_voc_breakdown_measurement()) return;

        if (!settings.flags(settings::Flags::sensors_voc_expected_variance_independent)) {
            auto _ = side.other().guard();
            GasIndexAlgorithm_sraw_std_fix16_set(
                    &gia, std::max(expected_variance(side.other()), GasIndexAlgorithm_sraw_std_fix16(&gia)));
        }

        side.set(process(raw, settings));
        side.set(GIAState(gia));
        checkpoint(side.voc_calibration_blob(), log);

        if (!settings.flags(settings::Flags::sensors_voc_expected_variance_independent)) {
            expected_variance(side) = GasIndexAlgorithm_sraw_std_fix16(&gia);
        }
    }

    // ~330 us during steady-state, ~30 us during startup blackout
    VOCIndex process(int32_t raw, settings::Settings const& settings = settings::g_active) {
        if (auto threshold = settings.voc_gating_threshold_override.or_(settings.voc_gating_threshold);
                threshold != BLE::NOT_KNOWN) {
            assert(1 <= threshold && threshold <= 500);
            gia.mGating_Threshold = F16(threshold.value_or(0));
        }

        gia._gating_force = !settings.voc_calibration_enabled;

        int32_t voc_index{};
        GasIndexAlgorithm_process(&gia, raw, &voc_index);
        assert(0 <= voc_index && voc_index <= 500);
        return voc_index;
    }

    // returns false IFF `src` doesn't contain a saved state
    bool restore(settings::SensorCalibrationBlob const& src) {
        Blob blob;
        static_assert(sizeof(Blob) <= sizeof(src));
        memcpy(&blob, &src, sizeof(Blob));
        if (blob[0] == 0 && blob[1] == 0) return false;

        GasIndexAlgorithm_set_states(&gia, blob[0], blob[1]);
        next_checkpoint = Clock::now() + CHECKPOINT_PERIOD;
        return true;
    }

    void save(settings::SensorCalibrationBlob& dst) {
        Blob blob;
        GasIndexAlgorithm_get_states(&gia, &blob[0], &blob[1]);
        static_assert(sizeof(Blob) <= sizeof(dst));
        memcpy(&dst, &blob, sizeof(Blob));
    }

    // Clear checkpoint timeout. Next call to `checkpoint` will always issue a checkpoint.
    void checkpoint_clear() {
        next_checkpoint = Clock::now();
    }

    bool checkpoint(settings::SensorCalibrationBlob& blob) {
        auto const now = Clock::now();
        if (now < next_checkpoint) return false;

        next_checkpoint = now + CHECKPOINT_PERIOD;
        save(blob);
        return true;
    }

    template <typename A>
    bool restore(settings::SensorCalibrationBlob const& blob, A& log) {
        if (!restore(blob)) return false;

        Blob blob2;
        memcpy(&blob2, &blob, sizeof(blob2));
        log.log("restored {0x%08x, 0x%08x}", (int)blob2[0], (int)blob2[1]);
        return true;
    }

    template <typename A>
    bool checkpoint(settings::SensorCalibrationBlob& blob, A& log) {
        if (!checkpoint(blob)) return false;

        Blob blob2;
        memcpy(&blob2, &blob, sizeof(blob2));
        log.log("checkpointed {0x%08x, 0x%08x}", (int)blob2[0], (int)blob2[1]);
        return true;
    }

private:
    using Blob = int32_t[2];

    static std::array<fix16_t, 2> s_expected_variance;
    static fix16_t& expected_variance(EnvironmentalFilter side) {
        return s_expected_variance.at(side.kind == EnvironmentalFilter::Kind::Intake ? 0 : 1);
    }
};

}  // namespace nevermore::sensors
