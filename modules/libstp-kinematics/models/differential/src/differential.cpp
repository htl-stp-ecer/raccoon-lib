//
// Created by tobias on 9/8/25.
//

#include "kinematics/differential/differential.hpp"
#include "calibration/motor/calibration.hpp"
#include "foundation/types.hpp"
#include "foundation/config.hpp"
#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <cmath>
#include <thread>

namespace libstp::kinematics::differential
{
    DifferentialKinematics::DifferentialKinematics(hal::motor::Motor* left_motor,
                                                   hal::motor::Motor* right_motor,
                                                   double wheelbase,
                                                   double wheelRadius,
                                                   double max_velocity,
                                                   double max_acceleration)
        : m_wheelbase(wheelbase)
          , m_wheelRadius(wheelRadius)
          , left_motor_{.adapter = drive::MotorAdapter(left_motor)}
          , right_motor_{.adapter = drive::MotorAdapter(right_motor)}
    {
        if (!left_motor) throw std::invalid_argument("left_motor cannot be null");
        if (!right_motor) throw std::invalid_argument("right_motor cannot be null");
        if (wheelbase <= 0.0) throw std::invalid_argument("wheelbase must be positive");
        if (wheelRadius <= 0.0) throw std::invalid_argument("wheelRadius must be positive");
        setWheelLimits(max_velocity, max_acceleration);
        LIBSTP_LOG_TRACE(
            "DifferentialKinematics::ctor wheelbase={} wheelRadius={} max_vel={} max_accel={}",
            wheelbase,
            wheelRadius,
            max_velocity,
            max_acceleration);
    }

    void DifferentialKinematics::setWheelLimits(const double max_velocity, const double max_acceleration)
    {
        max_wheel_velocity_ = std::max(0.0, max_velocity);
        max_wheel_acceleration_ = std::max(0.0, max_acceleration);

        left_motor_.limiter.setMaxRate(max_wheel_acceleration_);
        right_motor_.limiter.setMaxRate(max_wheel_acceleration_);

        LIBSTP_LOG_TRACE(
            "DifferentialKinematics::setWheelLimits max_velocity={} max_acceleration={}",
            max_wheel_velocity_,
            max_wheel_acceleration_);
    }

    std::size_t DifferentialKinematics::wheelCount() const
    {
        return 2;
    }

    MotorCommands DifferentialKinematics::applyCommand(const foundation::ChassisVelocity& cmd, double dt)
    {
        LIBSTP_LOG_TRACE(
            "DifferentialKinematics::applyCommand dt={} cmd vx={} wz={}",
            dt,
            cmd.vx,
            cmd.wz);

        const double v_left = (cmd.vx - cmd.wz * m_wheelbase / 2.0) / m_wheelRadius;
        const double v_right = (cmd.vx + cmd.wz * m_wheelbase / 2.0) / m_wheelRadius;

        LIBSTP_LOG_TRACE(
            "DifferentialKinematics wheel speeds raw left={} right={}",
            v_left,
            v_right);

        const double left_clamped = std::clamp(v_left, -max_wheel_velocity_, max_wheel_velocity_);
        const double right_clamped = std::clamp(v_right, -max_wheel_velocity_, max_wheel_velocity_);

        LIBSTP_LOG_TRACE(
            "DifferentialKinematics wheel speeds clamped left={} right={} (max={})",
            left_clamped,
            right_clamped,
            max_wheel_velocity_);

        double left_accel = 0.0;
        double right_accel = 0.0;
        const double left_limited = left_motor_.limiter.step(left_clamped, left_motor_.target_w, dt, left_accel);
        const double right_limited = right_motor_.limiter.step(right_clamped, right_motor_.target_w, dt, right_accel);

        LIBSTP_LOG_TRACE(
            "DifferentialKinematics wheel speeds limited left={} right={} accel_left={} accel_right={}",
            left_limited,
            right_limited,
            left_accel,
            right_accel);

        left_motor_.target_w = left_limited;
        right_motor_.target_w = right_limited;

        bool left_saturated = false;
        bool right_saturated = false;
        left_motor_.adapter.setVelocityWithAccel(left_limited, left_accel, dt, &left_saturated);
        right_motor_.adapter.setVelocityWithAccel(right_limited, right_accel, dt, &right_saturated);

        LIBSTP_LOG_TRACE(
            "DifferentialKinematics command applied left={} right={} sat_left={} sat_right={} dt={}",
            left_limited,
            right_limited,
            left_saturated,
            right_saturated,
            dt);

        MotorCommands result;
        result.wheel_velocities = {left_limited, right_limited};
        result.saturated_any = left_saturated || right_saturated;
        result.saturation_mask = (left_saturated ? 0x1 : 0x0) | (right_saturated ? 0x2 : 0x0);

        return result;
    }

    foundation::ChassisVelocity DifferentialKinematics::estimateState() const
    {
        const double w_left = left_motor_.adapter.getVelocity();
        const double w_right = right_motor_.adapter.getVelocity();

        const double vx = (w_left + w_right) * m_wheelRadius / 2.0;
        const double wz = (w_right - w_left) * m_wheelRadius / m_wheelbase;

        LIBSTP_LOG_TRACE(
            "DifferentialKinematics::estimateState wheel_left={} wheel_right={} vx={} wz={}",
            w_left,
            w_right,
            vx,
            wz);

        return foundation::ChassisVelocity{vx, 0.0, wz};
    }

    void DifferentialKinematics::hardStop()
    {
        LIBSTP_LOG_TRACE("DifferentialKinematics::hardStop invoked");
        left_motor_.target_w = 0.0;
        right_motor_.target_w = 0.0;
        left_motor_.adapter.resetController();
        right_motor_.adapter.resetController();
        left_motor_.adapter.brake();
        right_motor_.adapter.brake();
    }

    bool DifferentialKinematics::supportsLateralMotion() const
    {
        return false; // Differential drive cannot strafe
    }

    void DifferentialKinematics::resetEncoders()
    {
        left_motor_.adapter.resetEncoderTracking();
        right_motor_.adapter.resetEncoderTracking();
        LIBSTP_LOG_TRACE("DifferentialKinematics::resetEncoders - reset all motor encoder tracking");
    }

    std::vector<calibration::CalibrationResult> DifferentialKinematics::calibrateMotors(
        const calibration::CalibrationConfig& config)
    {
        LIBSTP_LOG_INFO("=== Starting DifferentialKinematics motor calibration ===");

        std::vector<calibration::CalibrationResult> results;

        // Calibrate left motor
        LIBSTP_LOG_INFO("Calibrating left motor...");
        calibration::CalibrationResult left_result = left_motor_.adapter.calibrate(config);
        results.push_back(left_result);

        if (left_result.success) {
            LIBSTP_LOG_INFO("Left motor calibration successful");
        } else {
            LIBSTP_LOG_ERROR("Left motor calibration failed: {}", left_result.error_message);
        }

        // Wait a bit between calibrations
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Calibrate right motor
        LIBSTP_LOG_INFO("Calibrating right motor...");
        calibration::CalibrationResult right_result = right_motor_.adapter.calibrate(config);
        results.push_back(right_result);

        if (right_result.success) {
            LIBSTP_LOG_INFO("Right motor calibration successful");
        } else {
            LIBSTP_LOG_ERROR("Right motor calibration failed: {}", right_result.error_message);
        }

        LIBSTP_LOG_INFO("=== DifferentialKinematics motor calibration completed ===");
        LIBSTP_LOG_INFO("Success rate: {}/2 motors",
                    (left_result.success ? 1 : 0) + (right_result.success ? 1 : 0));

        return results;
    }
}
