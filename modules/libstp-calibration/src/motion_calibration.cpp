#include "calibration/motion_calibration.hpp"
#include "drive/drive.hpp"
#include "odometry/odometry.hpp"
#include "foundation/logging.hpp"

#include <chrono>

namespace libstp::calibration
{
    MotionCalibrator::MotionCalibrator(
        drive::Drive& drive,
        odometry::IOdometry& odometry,
        MotionCalibrationConfig config)
        : drive_(drive)
        , odometry_(odometry)
        , config_(config)
    {
    }

    MotionCalibrationResult MotionCalibrator::calibrate()
    {
        LIBSTP_LOG_INFO("Starting full motion calibration...");

        calibration_start_time_ = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();

        MotionCalibrationResult final_result;

        // Calibrate each motion type
        auto turn_result = calibrateTurnMotion();
        auto drive_result = calibrateDriveStraightMotion();

        // Combine results
        final_result.gains.insert(final_result.gains.end(),
                                 turn_result.gains.begin(),
                                 turn_result.gains.end());
        final_result.gains.insert(final_result.gains.end(),
                                 drive_result.gains.begin(),
                                 drive_result.gains.end());

        // Only calibrate strafe if kinematics supports it
        if (drive_.getKinematics().supportsLateralMotion())
        {
            auto strafe_result = calibrateStrafeMotion();
            final_result.gains.insert(final_result.gains.end(),
                                     strafe_result.gains.begin(),
                                     strafe_result.gains.end());
        }

        // Cross-validate
        final_result.success = turn_result.success && drive_result.success;
        final_result.duration_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count() - calibration_start_time_;

        LIBSTP_LOG_INFO("Motion calibration complete. Success: {}", final_result.success);

        return final_result;
    }

    MotionCalibrationResult MotionCalibrator::calibrateTurnMotion()
    {
        LIBSTP_LOG_INFO("Starting TurnMotion calibration...");

        MotionCalibrationResult result;

        // Safety check
        if (!checkSafeSpace())
        {
            result.success = false;
            result.error_message = "Insufficient safe space for calibration";
            return result;
        }

        // Tune angle controller
        auto gains = tuneAngleController();

        // Store results
        MotionCalibrationResult::GainSet gain_set;
        gain_set.kp = gains.kp;
        gain_set.ki = gains.ki;
        gain_set.kd = gains.kd;
        gain_set.motion_type = MotionType::TURN;
        gain_set.controller_name = "angle";
        result.gains.push_back(gain_set);

        result.success = true;

        LIBSTP_LOG_INFO("TurnMotion calibration complete: kp={:.3f}, ki={:.3f}, kd={:.3f}",
                       gains.kp, gains.ki, gains.kd);

        return result;
    }

    MotionCalibrationResult MotionCalibrator::calibrateDriveStraightMotion()
    {
        LIBSTP_LOG_INFO("Starting DriveStraightMotion calibration...");

        MotionCalibrationResult result;

        // Tune controllers
        auto distance_gains = tuneDistanceController();
        auto heading_gains = tuneHeadingController();

        // Store results
        MotionCalibrationResult::GainSet distance_set;
        distance_set.kp = distance_gains.kp;
        distance_set.ki = distance_gains.ki;
        distance_set.kd = distance_gains.kd;
        distance_set.motion_type = MotionType::DRIVE_STRAIGHT;
        distance_set.controller_name = "distance";
        result.gains.push_back(distance_set);

        MotionCalibrationResult::GainSet heading_set;
        heading_set.kp = heading_gains.kp;
        heading_set.ki = heading_gains.ki;
        heading_set.kd = heading_gains.kd;
        heading_set.motion_type = MotionType::DRIVE_STRAIGHT;
        heading_set.controller_name = "heading";
        result.gains.push_back(heading_set);

        result.success = true;

        LIBSTP_LOG_INFO("DriveStraightMotion calibration complete");

        return result;
    }

    MotionCalibrationResult MotionCalibrator::calibrateStrafeMotion()
    {
        LIBSTP_LOG_INFO("Starting StrafeMotion calibration...");

        MotionCalibrationResult result;

        // Tune controllers
        auto lateral_gains = tuneLateralController();
        auto heading_gains = tuneHeadingController();

        // Store results
        MotionCalibrationResult::GainSet lateral_set;
        lateral_set.kp = lateral_gains.kp;
        lateral_set.ki = lateral_gains.ki;
        lateral_set.kd = lateral_gains.kd;
        lateral_set.motion_type = MotionType::STRAFE;
        lateral_set.controller_name = "lateral";
        result.gains.push_back(lateral_set);

        MotionCalibrationResult::GainSet heading_set;
        heading_set.kp = heading_gains.kp;
        heading_set.ki = heading_gains.ki;
        heading_set.kd = heading_gains.kd;
        heading_set.motion_type = MotionType::STRAFE;
        heading_set.controller_name = "heading";
        result.gains.push_back(heading_set);

        result.success = true;

        LIBSTP_LOG_INFO("StrafeMotion calibration complete");

        return result;
    }

    // Tuning methods - placeholder implementations
    MotionCalibrator::PidGainsMotion MotionCalibrator::tuneAngleController()
    {
        LIBSTP_LOG_INFO("Tuning angle controller (placeholder)");

        // Return conservative default gains
        PidGainsMotion gains;
        gains.kp = 3.0;
        gains.ki = 0.1;
        gains.kd = 0.2;
        return gains;
    }

    MotionCalibrator::PidGainsMotion MotionCalibrator::tuneDistanceController()
    {
        LIBSTP_LOG_INFO("Tuning distance controller (placeholder)");

        PidGainsMotion gains;
        gains.kp = 2.0;
        gains.ki = 0.05;
        gains.kd = 0.1;
        return gains;
    }

    MotionCalibrator::PidGainsMotion MotionCalibrator::tuneHeadingController()
    {
        LIBSTP_LOG_INFO("Tuning heading controller (placeholder)");

        PidGainsMotion gains;
        gains.kp = 4.0;
        gains.ki = 0.1;
        gains.kd = 0.15;
        return gains;
    }

    MotionCalibrator::PidGainsMotion MotionCalibrator::tuneLateralController()
    {
        LIBSTP_LOG_INFO("Tuning lateral controller (placeholder)");

        PidGainsMotion gains;
        gains.kp = 2.0;
        gains.ki = 0.05;
        gains.kd = 0.1;
        return gains;
    }

    bool MotionCalibrator::validateGains(const std::vector<PidGainsMotion>& gains, MotionType type)
    {
        // Placeholder: always validate for now
        return true;
    }

    bool MotionCalibrator::crossValidate(const MotionCalibrationResult& result)
    {
        // Placeholder: always validate for now
        return true;
    }

    bool MotionCalibrator::checkSafeSpace() const
    {
        // Placeholder: assume safe space is available
        LIBSTP_LOG_INFO("Safe space check (placeholder - assuming OK)");
        return true;
    }

    bool MotionCalibrator::checkTimeout(double start_time) const
    {
        double current_time = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();

        return (current_time - start_time) > config_.max_test_duration;
    }
}
