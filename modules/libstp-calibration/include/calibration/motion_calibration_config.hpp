#pragma once

#include <vector>
#include <Eigen/Core>

namespace libstp::calibration
{
    struct MotionCalibrationConfig
    {
        // Test sequences for TurnMotion
        struct TurnTest
        {
            std::vector<double> angles_deg{45.0, 90.0, 180.0};
            std::vector<double> max_rates{0.5, 1.0, 2.0};  // rad/s
        };

        // Test sequences for DriveStraightMotion
        struct DriveTest
        {
            std::vector<double> distances_m{0.3, 0.5, 1.0};
            std::vector<double> max_speeds{0.2, 0.4, 0.6};  // m/s
        };

        // Test sequences for StrafeMotion
        struct StrafeTest
        {
            std::vector<double> distances_m{0.2, 0.4, 0.6};
            std::vector<double> max_speeds{0.15, 0.3, 0.45};  // m/s
        };

        TurnTest turn_tests;
        DriveTest drive_tests;
        StrafeTest strafe_tests;

        // Tuning parameters
        double initial_kp{2.0};
        double step_size_multiplier{0.8};
        int max_iterations{10};

        // Performance targets (competition-optimized)
        double target_settling_time{1.5};
        double max_overshoot{0.15};           // 15% acceptable for speed
        double target_steady_state_error{0.02};  // 2%

        // Safety
        double max_test_duration{120.0};
        double max_single_test_time{10.0};
        Eigen::Vector2d safe_space{2.0, 2.0};
    };
}
