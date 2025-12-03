#pragma once

#include "trajectory_data.hpp"
#include "calibration/motion_calibration_config.hpp"

namespace libstp::calibration::motion
{
    TrajectoryMetrics analyzeTrajectory(
        const std::vector<TrajectorySnapshot>& trajectory,
        double target
    );

    void analyzeLegacyStepResponse(
        const std::vector<TrajectorySnapshot>& trajectory,
        double target,
        double& settling_time,
        double& overshoot,
        double& steady_state_error
    );

    double computeMotionCost(
        const TrajectoryMetrics& metrics,
        const MotionCalibrationConfig& config,
        double max_drift = 0.0
    );
}
