//
// Created by tobias on 9/8/25.
//

#include "kinematics/differential/differential.hpp"
#include "calibration/motor/calibration.hpp"
#include "foundation/types.hpp"
#include "foundation/config.hpp"
#include <chrono>
#include <stdexcept>
#include <cmath>
#include <thread>

namespace libstp::kinematics::differential
{
    DifferentialKinematics::DifferentialKinematics(hal::motor::IMotor* left_motor,
                                                   hal::motor::IMotor* right_motor,
                                                   double wheelbase,
                                                   double wheelRadius)
        : m_wheelbase(wheelbase)
          , m_wheelRadius(wheelRadius)
          , left_motor_(left_motor)
          , right_motor_(right_motor)
    {
        if (!left_motor) throw std::invalid_argument("left_motor cannot be null");
        if (!right_motor) throw std::invalid_argument("right_motor cannot be null");
        if (wheelbase <= 0.0) throw std::invalid_argument("wheelbase must be positive");
        if (wheelRadius <= 0.0) throw std::invalid_argument("wheelRadius must be positive");
        LIBSTP_LOG_TRACE(
            "DifferentialKinematics::ctor wheelbase={} wheelRadius={}",
            wheelbase,
            wheelRadius);
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

        double v_left = (cmd.vx - cmd.wz * m_wheelbase / 2.0) / m_wheelRadius;
        double v_right = (cmd.vx + cmd.wz * m_wheelbase / 2.0) / m_wheelRadius;

        // Desaturate: if either wheel exceeds max, scale BOTH proportionally to preserve vx/wz ratio
        double max_abs = std::max(std::abs(v_left), std::abs(v_right));
        bool saturated = false;
        if (m_maxWheelSpeed > 0.0 && max_abs > m_maxWheelSpeed)
        {
            const double scale = m_maxWheelSpeed / max_abs;
            v_left *= scale;
            v_right *= scale;
            saturated = true;
            LIBSTP_LOG_DEBUG(
                "DifferentialKinematics desaturated: scale={:.3f} (max_abs={:.2f}, limit={:.2f})",
                scale, max_abs, m_maxWheelSpeed);
        }

        LIBSTP_LOG_TRACE(
            "DifferentialKinematics wheel speeds left={} right={}{}",
            v_left,
            v_right,
            saturated ? " [desaturated]" : "");

        left_motor_.setVelocity(v_left, dt);
        right_motor_.setVelocity(v_right, dt);

        LIBSTP_LOG_TRACE(
            "DifferentialKinematics command applied left={} right={} dt={}",
            v_left,
            v_right,
            dt);

        MotorCommands result;
        result.wheel_velocities = {v_left, v_right};
        result.saturated_any = saturated;

        return result;
    }

    foundation::ChassisVelocity DifferentialKinematics::estimateState() const
    {
        const double w_left = left_motor_.getVelocity();
        const double w_right = right_motor_.getVelocity();

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
        left_motor_.brake();
        right_motor_.brake();
    }

    bool DifferentialKinematics::supportsLateralMotion() const
    {
        return false; // Differential drive cannot strafe
    }

    void DifferentialKinematics::resetEncoders()
    {
        left_motor_.resetEncoderTracking();
        right_motor_.resetEncoderTracking();
        LIBSTP_LOG_TRACE("DifferentialKinematics::resetEncoders - reset all motor encoder tracking");
    }

    std::vector<calibration::CalibrationResult> DifferentialKinematics::calibrateMotors(
        const calibration::CalibrationConfig& config)
    {
        LIBSTP_LOG_INFO("=== Starting DifferentialKinematics motor calibration ===");

        std::vector<calibration::CalibrationResult> results;

        // Calibrate left motor
        LIBSTP_LOG_INFO("Calibrating left motor...");
        calibration::CalibrationResult left_result = left_motor_.calibrate(config);
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
        calibration::CalibrationResult right_result = right_motor_.calibrate(config);
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

    std::vector<hal::motor::IMotor*> DifferentialKinematics::getMotors() const
    {
        return {
            &const_cast<hal::motor::IMotor&>(left_motor_.motor()),
            &const_cast<hal::motor::IMotor&>(right_motor_.motor())
        };
    }

    void DifferentialKinematics::applyPowerCommand(const foundation::ChassisVelocity& direction,
                                                    int power_percent)
    {
        // Same IK as applyCommand but only ratios matter.
        double w_left  = direction.vx - direction.wz * m_wheelbase / 2.0;
        double w_right = direction.vx + direction.wz * m_wheelbase / 2.0;

        const double max_abs = std::max(std::abs(w_left), std::abs(w_right));
        if (max_abs < 1e-9) return;

        const double scale = static_cast<double>(power_percent) / max_abs;
        const int p_left  = static_cast<int>(std::round(w_left * scale));
        const int p_right = static_cast<int>(std::round(w_right * scale));

        LIBSTP_LOG_DEBUG(
            "DifferentialKinematics::applyPowerCommand power={}% -> left={} right={}",
            power_percent, p_left, p_right);

        left_motor_.motor().setSpeed(p_left);
        right_motor_.motor().setSpeed(p_right);
    }
}
