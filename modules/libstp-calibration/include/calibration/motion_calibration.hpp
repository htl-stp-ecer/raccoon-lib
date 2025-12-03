#pragma once

#include "motion_calibration_config.hpp"
#include "motion_calibration_result.hpp"

namespace libstp::drive
{
    class Drive;
}

namespace libstp::odometry
{
    class IOdometry;
}

namespace libstp::calibration
{
    /**
     * @brief Motion calibration orchestrator
     *
     * Coordinates turn, drive, and strafe motion calibration.
     * Uses the existing motion_calibration.cpp implementation which contains
     * test runners, optimizers, and tuners inline.
     *
     * @note Future refactoring will extract these components:
     *   - motion/motion_test_runner
     *   - motion/simplex_optimizer
     *   - motion/angle_controller_tuner
     *   - motion/distance_controller_tuner
     *   - motion/heading_controller_tuner
     *   - motion/lateral_controller_tuner
     */
    class MotionCalibrator
    {
    public:
        MotionCalibrator(
            drive::Drive& drive,
            odometry::IOdometry& odometry,
            MotionCalibrationConfig config = {}
        );
        ~MotionCalibrator() = default;

        // Main calibration entry point
        MotionCalibrationResult calibrate();

        // Individual motion calibration
        MotionCalibrationResult calibrateTurnMotion();
        MotionCalibrationResult calibrateDriveStraightMotion();
        MotionCalibrationResult calibrateStrafeMotion();

    private:
        struct PidGainsMotion
        {
            double kp{0.0};
            double ki{0.0};
            double kd{0.0};
        };

        // Tuning methods
        PidGainsMotion tuneAngleController();
        PidGainsMotion tuneDistanceController();
        PidGainsMotion tuneHeadingController();
        PidGainsMotion tuneLateralController();

        // Validation
        bool validateGains(const std::vector<PidGainsMotion>& gains, MotionType type);
        bool crossValidate(const MotionCalibrationResult& result);

        // Safety
        bool checkSafeSpace() const;
        bool checkTimeout(double start_time) const;

        drive::Drive& drive_;
        odometry::IOdometry& odometry_;
        MotionCalibrationConfig config_;
        MotionCalibrationResult result_;
        double calibration_start_time_{0.0};
    };
}
