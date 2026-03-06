//
// Created by tobias on 9/8/25.
//

#include "kinematics/mecanum/mecanum.hpp"
#include "calibration/motor/calibration.hpp"
#include "foundation/types.hpp"
#include "foundation/config.hpp"
#include <chrono>
#include <stdexcept>
#include <cmath>
#include <thread>

namespace libstp::kinematics::mecanum
{
    MecanumKinematics::MecanumKinematics(hal::motor::IMotor* front_left_motor,
                                         hal::motor::IMotor* front_right_motor,
                                         hal::motor::IMotor* back_left_motor,
                                         hal::motor::IMotor* back_right_motor,
                                         const double wheelbase,
                                         const double trackWidth,
                                         const double wheelRadius)
        : m_wheelbase(wheelbase)
          , m_trackWidth(trackWidth)
          , m_wheelRadius(wheelRadius)
          , front_left_motor_(front_left_motor)
          , front_right_motor_(front_right_motor)
          , back_left_motor_(back_left_motor)
          , back_right_motor_(back_right_motor)
    {
        if (!front_left_motor) throw std::invalid_argument("front_left_motor cannot be null");
        if (!front_right_motor) throw std::invalid_argument("front_right_motor cannot be null");
        if (!back_left_motor) throw std::invalid_argument("back_left_motor cannot be null");
        if (!back_right_motor) throw std::invalid_argument("back_right_motor cannot be null");
        if (wheelbase <= 0.0) throw std::invalid_argument("wheelbase must be positive");
        if (trackWidth <= 0.0) throw std::invalid_argument("trackWidth must be positive");
        if (wheelRadius <= 0.0) throw std::invalid_argument("wheelRadius must be positive");
        LIBSTP_LOG_TRACE(
            "MecanumKinematics::ctor wheelbase={} trackWidth={} wheelRadius={}",
            wheelbase,
            trackWidth,
            wheelRadius);
    }

    std::size_t MecanumKinematics::wheelCount() const
    {
        return 4;
    }

    MotorCommands MecanumKinematics::applyCommand(const foundation::ChassisVelocity& cmd, double dt)
    {
        LIBSTP_LOG_TRACE(
            "MecanumKinematics::applyCommand dt={} cmd vx={} vy={} wz={}",
            dt,
            cmd.vx,
            cmd.vy,
            cmd.wz);

        const double L = (m_wheelbase + m_trackWidth) / 2.0;

        // Body frame convention: +x forward, +y to the right (matches odometry/motion modules)
        double w_fl = (cmd.vx + cmd.vy - L * cmd.wz) / m_wheelRadius;
        double w_fr = (cmd.vx - cmd.vy + L * cmd.wz) / m_wheelRadius;
        double w_bl = (cmd.vx - cmd.vy - L * cmd.wz) / m_wheelRadius;
        double w_br = (cmd.vx + cmd.vy + L * cmd.wz) / m_wheelRadius;

        // Desaturate: if any wheel exceeds max, scale ALL proportionally to preserve ratio
        double max_abs = std::max({std::abs(w_fl), std::abs(w_fr), std::abs(w_bl), std::abs(w_br)});
        bool saturated = false;
        if (m_maxWheelSpeed > 0.0 && max_abs > m_maxWheelSpeed)
        {
            const double scale = m_maxWheelSpeed / max_abs;
            w_fl *= scale;
            w_fr *= scale;
            w_bl *= scale;
            w_br *= scale;
            saturated = true;
            LIBSTP_LOG_DEBUG(
                "MecanumKinematics desaturated: scale={:.3f} (max_abs={:.2f}, limit={:.2f})",
                scale, max_abs, m_maxWheelSpeed);
        }

        LIBSTP_LOG_TRACE(
            "MecanumKinematics wheel speeds fl={} fr={} bl={} br={}{}",
            w_fl, w_fr, w_bl, w_br,
            saturated ? " [desaturated]" : "");

        front_left_motor_.setVelocity(w_fl, dt);
        front_right_motor_.setVelocity(w_fr, dt);
        back_left_motor_.setVelocity(w_bl, dt);
        back_right_motor_.setVelocity(w_br, dt);

        MotorCommands result;
        result.wheel_velocities = {w_fl, w_fr, w_bl, w_br};
        result.saturated_any = saturated;

        return result;
    }

    foundation::ChassisVelocity MecanumKinematics::estimateState() const
    {
        const double w_fl = front_left_motor_.getVelocity();
        const double w_fr = front_right_motor_.getVelocity();
        const double w_bl = back_left_motor_.getVelocity();
        const double w_br = back_right_motor_.getVelocity();

        const double L = (m_wheelbase + m_trackWidth) / 2.0;

        // Body frame convention: +x forward, +y to the right
        const double vx = (w_fl + w_fr + w_bl + w_br) * m_wheelRadius / 4.0;
        const double vy = (w_fl - w_fr - w_bl + w_br) * m_wheelRadius / 4.0;
        const double wz = (-w_fl + w_fr - w_bl + w_br) * m_wheelRadius / (4.0 * L);

        LIBSTP_LOG_TRACE(
            "MecanumKinematics::estimateState wheels fl={} fr={} bl={} br={} -> vx={} vy={} wz={}",
            w_fl,
            w_fr,
            w_bl,
            w_br,
            vx,
            vy,
            wz);

        return foundation::ChassisVelocity{vx, vy, wz};
    }

    void MecanumKinematics::hardStop()
    {
        LIBSTP_LOG_TRACE("MecanumKinematics::hardStop invoked");
        front_left_motor_.brake();
        front_right_motor_.brake();
        back_left_motor_.brake();
        back_right_motor_.brake();
    }

    bool MecanumKinematics::supportsLateralMotion() const
    {
        return true; // Mecanum drive can strafe
    }

    void MecanumKinematics::resetEncoders()
    {
        front_left_motor_.resetEncoderTracking();
        front_right_motor_.resetEncoderTracking();
        back_left_motor_.resetEncoderTracking();
        back_right_motor_.resetEncoderTracking();
        LIBSTP_LOG_TRACE("MecanumKinematics::resetEncoders - reset all motor encoder tracking");
    }

    std::vector<calibration::CalibrationResult> MecanumKinematics::calibrateMotors(
        const calibration::CalibrationConfig& config)
    {
        LIBSTP_LOG_INFO("=== Starting MecanumKinematics motor calibration ===");

        std::vector<calibration::CalibrationResult> results;

        // Calibrate front left motor
        LIBSTP_LOG_INFO("Calibrating front left motor...");
        calibration::CalibrationResult fl_result = front_left_motor_.calibrate(config);
        results.push_back(fl_result);

        if (fl_result.success) {
            LIBSTP_LOG_INFO("Front left motor calibration successful");
        } else {
            LIBSTP_LOG_ERROR("Front left motor calibration failed: {}", fl_result.error_message);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Calibrate front right motor
        LIBSTP_LOG_INFO("Calibrating front right motor...");
        calibration::CalibrationResult fr_result = front_right_motor_.calibrate(config);
        results.push_back(fr_result);

        if (fr_result.success) {
            LIBSTP_LOG_INFO("Front right motor calibration successful");
        } else {
            LIBSTP_LOG_ERROR("Front right motor calibration failed: {}", fr_result.error_message);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Calibrate back left motor
        LIBSTP_LOG_INFO("Calibrating back left motor...");
        calibration::CalibrationResult bl_result = back_left_motor_.calibrate(config);
        results.push_back(bl_result);

        if (bl_result.success) {
            LIBSTP_LOG_INFO("Back left motor calibration successful");
        } else {
            LIBSTP_LOG_ERROR("Back left motor calibration failed: {}", bl_result.error_message);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Calibrate back right motor
        LIBSTP_LOG_INFO("Calibrating back right motor...");
        calibration::CalibrationResult br_result = back_right_motor_.calibrate(config);
        results.push_back(br_result);

        if (br_result.success) {
            LIBSTP_LOG_INFO("Back right motor calibration successful");
        } else {
            LIBSTP_LOG_ERROR("Back right motor calibration failed: {}", br_result.error_message);
        }

        LIBSTP_LOG_INFO("=== MecanumKinematics motor calibration completed ===");
        int success_count = (fl_result.success ? 1 : 0) + (fr_result.success ? 1 : 0) +
                           (bl_result.success ? 1 : 0) + (br_result.success ? 1 : 0);
        LIBSTP_LOG_INFO("Success rate: {}/4 motors", success_count);

        return results;
    }

    std::vector<hal::motor::IMotor*> MecanumKinematics::getMotors() const
    {
        return {
            &const_cast<hal::motor::IMotor&>(front_left_motor_.motor()),
            &const_cast<hal::motor::IMotor&>(front_right_motor_.motor()),
            &const_cast<hal::motor::IMotor&>(back_left_motor_.motor()),
            &const_cast<hal::motor::IMotor&>(back_right_motor_.motor())
        };
    }
}
