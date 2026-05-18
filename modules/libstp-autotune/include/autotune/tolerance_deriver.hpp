#pragma once

#include <map>
#include <string>

#include "autotune/types.hpp"
#include "motion/motion_config.hpp"

namespace libstp::autotune
{
    /**
     * @brief Derive realistic motion tolerances from MotionTuner results.
     *
     * Pure computation — no hardware interaction. Takes the per-parameter
     * tuning scores, recovers a representative error magnitude per parameter,
     * scales by `cfg.margin_factor`, clamps into the configured limits, and
     * writes the result back to @p motion_config.
     *
     * The recovery is:
     *
     *     error_estimate = final_score / (score_error_weight + epsilon)
     *
     * which inverts the dominant `score_error_weight * error` term that the
     * MotionTuner uses when computing the composite score.
     *
     * @param tune_results   Map of MotionTuner outputs keyed by param name.
     *                       "distance" / "lateral" feed `distance_tolerance_m`;
     *                       "heading" feeds `angle_tolerance_rad`.
     * @param tune_config    The MotionTuneConfig used to produce
     *                       @p tune_results (needed for score_error_weight).
     * @param motion_config  Motion PID config to update in place.
     * @param cfg            Tolerance derivation configuration.
     * @return The derived tolerances and which fields were actually updated.
     */
    ToleranceResult deriveTolerances(
        const std::map<std::string, MotionTuneResult>& tune_results,
        const MotionTuneConfig&                        tune_config,
        motion::UnifiedMotionPidConfig&                motion_config,
        const ToleranceConfig&                         cfg = {});

} // namespace libstp::autotune
