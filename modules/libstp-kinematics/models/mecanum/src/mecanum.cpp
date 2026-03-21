//
// Created by tobias on 9/8/25.
//

#include "kinematics/mecanum/mecanum.hpp"
#include "foundation/types.hpp"
#include "foundation/config.hpp"
#include <stdexcept>
#include <cmath>

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

    std::vector<hal::motor::IMotor*> MecanumKinematics::getMotors() const
    {
        return {
            &const_cast<hal::motor::IMotor&>(front_left_motor_.motor()),
            &const_cast<hal::motor::IMotor&>(front_right_motor_.motor()),
            &const_cast<hal::motor::IMotor&>(back_left_motor_.motor()),
            &const_cast<hal::motor::IMotor&>(back_right_motor_.motor())
        };
    }

    IKinematics::StmOdometryConfig MecanumKinematics::getStmOdometryConfig() const
    {
        const double R = m_wheelRadius;
        const double L = (m_wheelbase + m_trackWidth) / 2.0;

        StmOdometryConfig cfg{};

        // vx = (w_fl + w_fr + w_bl + w_br) * R / 4
        cfg.inv_matrix[0] = {
            static_cast<float>(R / 4.0),
            static_cast<float>(R / 4.0),
            static_cast<float>(R / 4.0),
            static_cast<float>(R / 4.0)
        };

        // vy = (w_fl - w_fr - w_bl + w_br) * R / 4
        cfg.inv_matrix[1] = {
            static_cast<float>(R / 4.0),
            static_cast<float>(-R / 4.0),
            static_cast<float>(-R / 4.0),
            static_cast<float>(R / 4.0)
        };

        // wz = (-w_fl + w_fr - w_bl + w_br) * R / (4 * L)
        cfg.inv_matrix[2] = {
            static_cast<float>(-R / (4.0 * L)),
            static_cast<float>(R / (4.0 * L)),
            static_cast<float>(-R / (4.0 * L)),
            static_cast<float>(R / (4.0 * L))
        };

        // Per-motor ticks_to_rad from calibration
        const auto motors = getMotors();
        for (std::size_t i = 0; i < 4; ++i) {
            double t2r = motors[i]->getCalibration().ticks_to_rad;
            // STM32 reads raw BEMF ticks — negate for inverted motors so
            // the sign convention matches the kinematics matrix.
            if (motors[i]->isInverted()) t2r = -t2r;
            cfg.ticks_to_rad[i] = static_cast<float>(t2r);
        }

        return cfg;
    }

    void MecanumKinematics::applyPowerCommand(const foundation::ChassisVelocity& direction,
                                               int power_percent)
    {
        const double L = (m_wheelbase + m_trackWidth) / 2.0;

        // Same IK as applyCommand but we only care about ratios, not absolute speeds.
        // Dividing by wheelRadius is unnecessary since it cancels in normalization.
        double w_fl = direction.vx + direction.vy - L * direction.wz;
        double w_fr = direction.vx - direction.vy + L * direction.wz;
        double w_bl = direction.vx - direction.vy - L * direction.wz;
        double w_br = direction.vx + direction.vy + L * direction.wz;

        const double max_abs = std::max({std::abs(w_fl), std::abs(w_fr),
                                         std::abs(w_bl), std::abs(w_br)});
        if (max_abs < 1e-9) return;  // zero command

        const double scale = static_cast<double>(power_percent) / max_abs;
        const int p_fl = static_cast<int>(std::round(w_fl * scale));
        const int p_fr = static_cast<int>(std::round(w_fr * scale));
        const int p_bl = static_cast<int>(std::round(w_bl * scale));
        const int p_br = static_cast<int>(std::round(w_br * scale));

        LIBSTP_LOG_DEBUG(
            "MecanumKinematics::applyPowerCommand power={}% -> fl={} fr={} bl={} br={}",
            power_percent, p_fl, p_fr, p_bl, p_br);

        front_left_motor_.motor().setSpeed(p_fl);
        front_right_motor_.motor().setSpeed(p_fr);
        back_left_motor_.motor().setSpeed(p_bl);
        back_right_motor_.motor().setSpeed(p_br);
    }
}
