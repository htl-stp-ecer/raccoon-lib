#pragma once

#include <vector>

namespace libstp::calibration::motion
{
    struct TrajectorySnapshot
    {
        double time;
        double error;
        double command;
    };

    struct TrajectoryMetrics
    {
        double settling_time{0.0};
        double overshoot{0.0};
        double steady_state_error{0.0};
        double itae{0.0};
        double iae{0.0};
        int oscillation_count{0};
        double max_jerk{0.0};
        double total_jerk{0.0};
    };

    struct TestResult
    {
        std::vector<TrajectorySnapshot> trajectory;
        double settling_time{0.0};
        double overshoot{0.0};
        double steady_state_error{0.0};
        double completion_time{0.0};
        bool success{false};
    };
}
