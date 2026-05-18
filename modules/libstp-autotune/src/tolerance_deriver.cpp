#include "autotune/tolerance_deriver.hpp"

#include <algorithm>
#include <cmath>

#include "foundation/logging.hpp"

namespace libstp::autotune
{

ToleranceResult deriveTolerances(
    const std::map<std::string, MotionTuneResult>& tune_results,
    const MotionTuneConfig&                        tune_config,
    motion::UnifiedMotionPidConfig&                motion_config,
    const ToleranceConfig&                         cfg)
{
    ToleranceResult out;

    // Seed with the current values so a partial update doesn't surprise the
    // caller.
    out.derived_distance_tolerance_m = motion_config.distance_tolerance_m;
    out.derived_angle_tolerance_rad  = motion_config.angle_tolerance_rad;

    const double w_err  = tune_config.score_error_weight;
    const double w_safe = w_err + 1e-9;

    auto pickDistanceError = [&](double& out_err) -> bool
    {
        bool found = false;
        out_err = 0.0;
        for (const char* key : {"distance", "lateral"})
        {
            auto it = tune_results.find(key);
            if (it == tune_results.end())
                continue;
            const double est = it->second.final_score / w_safe;
            if (est > out_err)
                out_err = est;
            found = true;
        }
        return found;
    };

    double dist_err = 0.0;
    if (pickDistanceError(dist_err))
    {
        const double scaled = dist_err * cfg.margin_factor;
        const double clamped = std::clamp(scaled,
                                          cfg.min_distance_tolerance_m,
                                          cfg.max_distance_tolerance_m);
        out.derived_distance_tolerance_m = clamped;
        motion_config.distance_tolerance_m = clamped;
        out.distance_updated = true;

        LIBSTP_LOG_INFO("[ToleranceDeriver] distance: error_est={:.4f}m -> "
                        "tolerance={:.4f}m (margin x{:.2f}, clamped to [{:.4f}, {:.4f}])",
                        dist_err, clamped, cfg.margin_factor,
                        cfg.min_distance_tolerance_m,
                        cfg.max_distance_tolerance_m);
    }

    auto h_it = tune_results.find("heading");
    if (h_it != tune_results.end())
    {
        const double heading_err = h_it->second.final_score / w_safe;
        const double scaled  = heading_err * cfg.margin_factor;
        const double clamped = std::clamp(scaled,
                                          cfg.min_angle_tolerance_rad,
                                          cfg.max_angle_tolerance_rad);
        out.derived_angle_tolerance_rad = clamped;
        motion_config.angle_tolerance_rad = clamped;
        out.angle_updated = true;

        LIBSTP_LOG_INFO("[ToleranceDeriver] heading: error_est={:.4f}rad -> "
                        "tolerance={:.4f}rad (margin x{:.2f}, clamped to [{:.4f}, {:.4f}])",
                        heading_err, clamped, cfg.margin_factor,
                        cfg.min_angle_tolerance_rad,
                        cfg.max_angle_tolerance_rad);
    }

    if (!out.distance_updated && !out.angle_updated)
    {
        LIBSTP_LOG_WARN("[ToleranceDeriver] No usable tune_results — config unchanged");
    }

    return out;
}

} // namespace libstp::autotune
