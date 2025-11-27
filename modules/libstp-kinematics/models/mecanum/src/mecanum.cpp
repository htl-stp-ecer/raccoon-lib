//
// Created by tobias on 9/8/25.
//

#include "kinematics/mecanum/mecanum.hpp"
#include "calibration/calibration.hpp"
#include "foundation/types.hpp"
#include "foundation/config.hpp"
#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <cmath>
#include <thread>

namespace libstp::kinematics::mecanum
{
    MecanumKinematics::MecanumKinematics(hal::motor::Motor* front_left_motor,
                                         hal::motor::Motor* front_right_motor,
                                         hal::motor::Motor* back_left_motor,
                                         hal::motor::Motor* back_right_motor,
                                         const double wheelbase,
                                         const double trackWidth,
                                         const double wheelRadius,
                                         double max_velocity,
                                         double max_acceleration)
        : m_wheelbase(wheelbase)
          , m_trackWidth(trackWidth)
          , m_wheelRadius(wheelRadius)
          , front_left_motor_{.adapter = drive::MotorAdapter(front_left_motor)}
          , front_right_motor_{.adapter = drive::MotorAdapter(front_right_motor)}
          , back_left_motor_{.adapter = drive::MotorAdapter(back_left_motor)}
          , back_right_motor_{.adapter = drive::MotorAdapter(back_right_motor)}
    {
        if (!front_left_motor) throw std::invalid_argument("front_left_motor cannot be null");
        if (!front_right_motor) throw std::invalid_argument("front_right_motor cannot be null");
        if (!back_left_motor) throw std::invalid_argument("back_left_motor cannot be null");
        if (!back_right_motor) throw std::invalid_argument("back_right_motor cannot be null");
        if (wheelbase <= 0.0) throw std::invalid_argument("wheelbase must be positive");
        if (trackWidth <= 0.0) throw std::invalid_argument("trackWidth must be positive");
        if (wheelRadius <= 0.0) throw std::invalid_argument("wheelRadius must be positive");
        setWheelLimits(max_velocity, max_acceleration);
        LIBSTP_LOG_INFO(
            "MecanumKinematics::ctor wheelbase={} trackWidth={} wheelRadius={} max_vel={} max_accel={}",
            wheelbase,
            trackWidth,
            wheelRadius,
            max_velocity,
            max_acceleration);
    }

    void MecanumKinematics::setWheelLimits(const double max_velocity, const double max_acceleration)
    {
        max_wheel_velocity_ = std::max(0.0, max_velocity);
        max_wheel_acceleration_ = std::max(0.0, max_acceleration);

        front_left_motor_.limiter.setMaxRate(max_wheel_acceleration_);
        front_right_motor_.limiter.setMaxRate(max_wheel_acceleration_);
        back_left_motor_.limiter.setMaxRate(max_wheel_acceleration_);
        back_right_motor_.limiter.setMaxRate(max_wheel_acceleration_);

        LIBSTP_LOG_INFO(
            "MecanumKinematics::setWheelLimits max_velocity={} max_acceleration={}",
            max_wheel_velocity_,
            max_wheel_acceleration_);
    }

    std::size_t MecanumKinematics::wheelCount() const
    {
        return 4;
    }

    MotorCommands MecanumKinematics::applyCommand(const foundation::ChassisCmd& cmd, double dt)
    {
        LIBSTP_LOG_INFO(
            "MecanumKinematics::applyCommand dt={} cmd vx={} vy={} wz={}",
            dt,
            cmd.vx,
            cmd.vy,
            cmd.wz);

        const double L = (m_wheelbase + m_trackWidth) / 2.0;

        // Body frame convention: +x forward, +y to the right (matches odometry/motion modules)
        const double w_fl = (cmd.vx + cmd.vy - L * cmd.wz) / m_wheelRadius;
        const double w_fr = (cmd.vx - cmd.vy + L * cmd.wz) / m_wheelRadius;
        const double w_bl = (cmd.vx - cmd.vy - L * cmd.wz) / m_wheelRadius;
        const double w_br = (cmd.vx + cmd.vy + L * cmd.wz) / m_wheelRadius;

        LIBSTP_LOG_TRACE(
            "MecanumKinematics wheel speeds raw fl={} fr={} bl={} br={}",
            w_fl,
            w_fr,
            w_bl,
            w_br);

        const double fl_clamped = std::clamp(w_fl, -max_wheel_velocity_, max_wheel_velocity_);
        const double fr_clamped = std::clamp(w_fr, -max_wheel_velocity_, max_wheel_velocity_);
        const double bl_clamped = std::clamp(w_bl, -max_wheel_velocity_, max_wheel_velocity_);
        const double br_clamped = std::clamp(w_br, -max_wheel_velocity_, max_wheel_velocity_);

        LIBSTP_LOG_TRACE(
            "MecanumKinematics wheel speeds clamped fl={} fr={} bl={} br={} (max={})",
            fl_clamped,
            fr_clamped,
            bl_clamped,
            br_clamped,
            max_wheel_velocity_);

        double fl_accel = 0.0, fr_accel = 0.0, bl_accel = 0.0, br_accel = 0.0;
        const double fl_limited = front_left_motor_.limiter.step(fl_clamped, front_left_motor_.target_w, dt, fl_accel);
        const double fr_limited = front_right_motor_.limiter.
                                                     step(fr_clamped, front_right_motor_.target_w, dt, fr_accel);
        const double bl_limited = back_left_motor_.limiter.step(bl_clamped, back_left_motor_.target_w, dt, bl_accel);
        const double br_limited = back_right_motor_.limiter.step(br_clamped, back_right_motor_.target_w, dt, br_accel);

        LIBSTP_LOG_TRACE(
            "MecanumKinematics wheel speeds limited fl={} fr={} bl={} br={} accel_fl={} accel_fr={} accel_bl={} accel_br={}",
            fl_limited,
            fr_limited,
            bl_limited,
            br_limited,
            fl_accel,
            fr_accel,
            bl_accel,
            br_accel);

        front_left_motor_.target_w = fl_limited;
        front_right_motor_.target_w = fr_limited;
        back_left_motor_.target_w = bl_limited;
        back_right_motor_.target_w = br_limited;

        bool fl_sat = false, fr_sat = false, bl_sat = false, br_sat = false;
        front_left_motor_.adapter.setVelocityWithAccel(fl_limited, fl_accel, dt, &fl_sat);
        front_right_motor_.adapter.setVelocityWithAccel(fr_limited, fr_accel, dt, &fr_sat);
        back_left_motor_.adapter.setVelocityWithAccel(bl_limited, bl_accel, dt, &bl_sat);
        back_right_motor_.adapter.setVelocityWithAccel(br_limited, br_accel, dt, &br_sat);

        LIBSTP_LOG_INFO(
            "MecanumKinematics command applied fl={} fr={} bl={} br={} sat_fl={} sat_fr={} sat_bl={} sat_br={} dt={}",
            fl_limited,
            fr_limited,
            bl_limited,
            br_limited,
            fl_sat,
            fr_sat,
            bl_sat,
            br_sat,
            dt);

        MotorCommands result;
        result.wheel_velocities = {fl_limited, fr_limited, bl_limited, br_limited};
        result.saturated_any = fl_sat || fr_sat || bl_sat || br_sat;
        result.saturation_mask = (fl_sat ? 0x1 : 0x0) | (fr_sat ? 0x2 : 0x0) |
                                (bl_sat ? 0x4 : 0x0) | (br_sat ? 0x8 : 0x0);

        return result;
    }

    foundation::ChassisState MecanumKinematics::estimateState() const
    {
        const double w_fl = front_left_motor_.adapter.getVelocity();
        const double w_fr = front_right_motor_.adapter.getVelocity();
        const double w_bl = back_left_motor_.adapter.getVelocity();
        const double w_br = back_right_motor_.adapter.getVelocity();

        const double L = (m_wheelbase + m_trackWidth) / 2.0;

        // Body frame convention: +x forward, +y to the right
        const double vx = (w_fl + w_fr + w_bl + w_br) * m_wheelRadius / 4.0;
        const double vy = (w_fl - w_fr - w_bl + w_br) * m_wheelRadius / 4.0;
        const double w = (-w_fl + w_fr - w_bl + w_br) * m_wheelRadius / (4.0 * L);

        LIBSTP_LOG_TRACE(
            "MecanumKinematics::estimateState wheels fl={} fr={} bl={} br={} -> vx={} vy={} w={}",
            w_fl,
            w_fr,
            w_bl,
            w_br,
            vx,
            vy,
            w);

        return foundation::ChassisState{vx, vy, w};
    }

    void MecanumKinematics::hardStop()
    {
        LIBSTP_LOG_INFO("MecanumKinematics::hardStop invoked");
        front_left_motor_.target_w = 0.0;
        front_right_motor_.target_w = 0.0;
        back_left_motor_.target_w = 0.0;
        back_right_motor_.target_w = 0.0;

        front_left_motor_.adapter.resetController();
        front_right_motor_.adapter.resetController();
        back_left_motor_.adapter.resetController();
        back_right_motor_.adapter.resetController();

        front_left_motor_.adapter.brake();
        front_right_motor_.adapter.brake();
        back_left_motor_.adapter.brake();
        back_right_motor_.adapter.brake();
    }

    bool MecanumKinematics::supportsLateralMotion() const
    {
        return true; // Mecanum drive can strafe
    }

    void MecanumKinematics::resetEncoders()
    {
        front_left_motor_.adapter.resetEncoderTracking();
        front_right_motor_.adapter.resetEncoderTracking();
        back_left_motor_.adapter.resetEncoderTracking();
        back_right_motor_.adapter.resetEncoderTracking();
        LIBSTP_LOG_INFO("MecanumKinematics::resetEncoders - reset all motor encoder tracking");
    }

    std::vector<drive::CalibrationResult> MecanumKinematics::calibrateMotors()
    {
        return calibrateMotors(drive::CalibrationConfig{});
    }

    std::vector<drive::CalibrationResult> MecanumKinematics::calibrateMotors(
        const drive::CalibrationConfig& config)
    {
        LIBSTP_LOG_INFO("=== Starting MecanumKinematics motor calibration ===");

        std::vector<drive::CalibrationResult> results;

        // Calibrate front left motor
        LIBSTP_LOG_INFO("Calibrating front left motor...");
        drive::CalibrationResult fl_result = front_left_motor_.adapter.calibrate(config);
        results.push_back(fl_result);

        if (fl_result.success) {
            LIBSTP_LOG_INFO("Front left motor calibration successful");
        } else {
            LIBSTP_LOG_ERROR("Front left motor calibration failed: {}", fl_result.error_message);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Calibrate front right motor
        LIBSTP_LOG_INFO("Calibrating front right motor...");
        drive::CalibrationResult fr_result = front_right_motor_.adapter.calibrate(config);
        results.push_back(fr_result);

        if (fr_result.success) {
            LIBSTP_LOG_INFO("Front right motor calibration successful");
        } else {
            LIBSTP_LOG_ERROR("Front right motor calibration failed: {}", fr_result.error_message);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Calibrate back left motor
        LIBSTP_LOG_INFO("Calibrating back left motor...");
        drive::CalibrationResult bl_result = back_left_motor_.adapter.calibrate(config);
        results.push_back(bl_result);

        if (bl_result.success) {
            LIBSTP_LOG_INFO("Back left motor calibration successful");
        } else {
            LIBSTP_LOG_ERROR("Back left motor calibration failed: {}", bl_result.error_message);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Calibrate back right motor
        LIBSTP_LOG_INFO("Calibrating back right motor...");
        drive::CalibrationResult br_result = back_right_motor_.adapter.calibrate(config);
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
}
