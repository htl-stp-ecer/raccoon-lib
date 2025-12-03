#pragma once

#include <string>

#include "foundation/motor.hpp"

namespace libstp::calibration
{
    struct CalibrationResult
    {
        foundation::PidGains pid;
        foundation::Feedforward ff;
        bool success{false};
        std::string error_message;
        double duration_seconds{0.0};

        // Detailed metrics for debugging
        struct Metrics {
            double static_friction_forward{0.0};
            double static_friction_backward{0.0};
            double velocity_constant_r_squared{0.0}; // Linear regression fit quality
            int velocity_samples{0};
            double acceleration_mean{0.0};
            double acceleration_std_dev{0.0};

            // PID tuning metrics
            double time_constant_tau{0.0};
            double steady_state_gain{0.0};
            double delay{0.0};

            // Relay feedback metrics (if used)
            double ultimate_gain_ku{0.0};
            double ultimate_period_tu{0.0};
            int oscillation_count{0};

            // Validation metrics
            double validation_mean_error{0.0};
            double validation_max_error{0.0};
            bool validation_passed{false};
        } metrics;
    };
}