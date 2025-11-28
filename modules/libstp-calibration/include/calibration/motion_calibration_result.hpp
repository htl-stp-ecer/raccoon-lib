#pragma once

#include <vector>
#include <string>
#include <map>

namespace libstp::calibration
{
    enum class MotionType
    {
        TURN,
        DRIVE_STRAIGHT,
        STRAFE
    };

    struct MotionCalibrationResult
    {
        struct GainSet
        {
            double kp{0.0};
            double ki{0.0};
            double kd{0.0};
            MotionType motion_type;
            std::string controller_name;  // "angle", "distance", "heading", "lateral"
        };

        std::vector<GainSet> gains;
        bool success{false};
        std::string error_message;
        double duration_seconds{0.0};

        struct Metrics
        {
            struct MotionMetrics
            {
                double avg_settling_time{0.0};
                double max_overshoot{0.0};
                double avg_steady_state_error{0.0};
                double avg_completion_time{0.0};
                int tests_passed{0};
                int tests_failed{0};
            };

            std::map<MotionType, MotionMetrics> per_motion;
            bool validated_at_low_speed{false};
            bool validated_at_high_speed{false};
            bool validated_short_distance{false};
            bool validated_long_distance{false};
        } metrics;
    };
}
