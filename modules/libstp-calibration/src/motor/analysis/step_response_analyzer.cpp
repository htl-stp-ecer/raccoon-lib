#include "calibration/motor/analysis/step_response_analyzer.hpp"
#include "calibration/motor/utils/math_utils.hpp"
#include "foundation/logging.hpp"
#include <cmath>

namespace libstp::calibration::analysis
{
    StepResponseParams fitStepResponse(
        const data::VelocityProfile& profile,
        double initial_velocity)
    {
        StepResponseParams params{0.0, 0.0, 0.0, 0.0};

        if (profile.data.size() < 10) {
            return params;
        }

        // Find steady-state velocity (last 20% of data)
        size_t start_ss = profile.data.size() * 4 / 5;
        std::vector<double> ss_velocities;
        for (size_t i = start_ss; i < profile.data.size(); ++i) {
            ss_velocities.push_back(profile.data[i].velocity);
        }
        double v_ss = utils::getMeanValue(ss_velocities);

        // Steady-state gain K = Δvelocity / Δcommand
        double v_change = v_ss;
        double command = std::abs(profile.data.front().command);

        if (command < 0.1) {
            LIBSTP_LOG_WARN("Step response command too small: {}", command);
            return params;
        }

        params.K = v_change / command;

        // Find time when velocity reaches 63.2% of final value (tau)
        double target_velocity = 0.632 * v_ss;
        double t0 = profile.data.front().time;

        for (const auto& point : profile.data) {
            if (point.velocity >= target_velocity) {
                params.tau = point.time - t0;
                break;
            }
        }

        // Estimate delay (when response starts to rise above 10% of final)
        double threshold = 0.1 * v_ss;
        for (const auto& point : profile.data) {
            if (point.velocity >= threshold) {
                params.delay = point.time - t0;
                break;
            }
        }

        // Default values if not found
        if (params.tau == 0.0) params.tau = 0.5;
        if (params.delay == 0.0) params.delay = 0.05;
        if (std::abs(params.K) < 0.001) params.K = 1.0;

        return params;
    }
}
