#pragma once
#include "kinematics/kinematics.hpp"
#include "drive/motor_adapter.hpp"
#include "foundation/config.hpp"
#include "hal/IMotor.hpp"
#include <array>
#include <vector>

namespace libstp::kinematics::mecanum
{
    /**
     * Four-wheel mecanum implementation of `IKinematics`.
     *
     * The model uses the shared LibSTP body-frame convention: +x forward,
     * +y right, +wz counter-clockwise.
     */
    class MecanumKinematics : public IKinematics
    {
    private:
        double m_wheelbase;
        double m_trackWidth;
        double m_wheelRadius;
        double m_maxWheelSpeed{0.0}; // 0 = no limiting

        // Per-axis velocity-command gain [vx, vy, wz]; folded into fwd_matrix in
        // getStmOdometryConfig() to compensate drivetrain efficiency (e.g.
        // mecanum roller slip). Calibrated by the autotune velocity phase.
        std::array<double, 3> m_velCmdGain{1.0, 1.0, 1.0};

        drive::MotorAdapter front_left_motor_;
        drive::MotorAdapter front_right_motor_;
        drive::MotorAdapter back_left_motor_;
        drive::MotorAdapter back_right_motor_;

    public:
        /**
         * @param front_left_motor Non-owning pointer to the front-left motor.
         * @param front_right_motor Non-owning pointer to the front-right motor.
         * @param back_left_motor Non-owning pointer to the back-left motor.
         * @param back_right_motor Non-owning pointer to the back-right motor.
         * @param wheelbase Front-to-back axle spacing in meters.
         * @param trackWidth Left-to-right wheel spacing in meters.
         * @param wheelRadius Radius of each drive wheel in meters.
         * @param velocityCommandGain Per-axis velocity-command gain (identity by
         *        default); folded into the STM32 forward kinematics to
         *        compensate drivetrain efficiency. Calibrated by the autotune
         *        velocity phase.
         */
        MecanumKinematics(hal::motor::IMotor* front_left_motor,
                         hal::motor::IMotor* front_right_motor,
                         hal::motor::IMotor* back_left_motor,
                         hal::motor::IMotor* back_right_motor,
                         double wheelbase,
                         double trackWidth,
                         double wheelRadius,
                         foundation::VelocityCommandGain velocityCommandGain = {});
        ~MecanumKinematics() override = default;

        /** Always returns `4`. */
        [[nodiscard]] std::size_t wheelCount() const override;

        /** Convert chassis velocity into front-left, front-right, back-left, back-right wheel speeds. */
        MotorCommands applyCommand(const foundation::ChassisVelocity& cmd, double dt) override;

        /** Recover chassis velocity from the four measured wheel speeds. */
        [[nodiscard]] foundation::ChassisVelocity estimateState() const override;

        /** Brake all four drive motors immediately. */
        void hardStop() override;

        /** Mecanum drivetrains can command lateral motion directly. */
        [[nodiscard]] bool supportsLateralMotion() const override;

        /** Clear encoder history in all four `MotorAdapter` instances. */
        void resetEncoders() override;

        [[nodiscard]] double getWheelRadius() const override { return m_wheelRadius; }

        /** Set max wheel speed (rad/s) for desaturation. 0 = disabled. */
        void setMaxWheelSpeed(double max_wheel_speed) { m_maxWheelSpeed = max_wheel_speed; }
        [[nodiscard]] double getMaxWheelSpeed() const { return m_maxWheelSpeed; }

        /**
         * Mecanum wheel speed is `(|vx| + |vy| + |wz| * (l + w)/2) / r`; on an
         * arc (`v = omega * radius`, one translational axis) the fastest wheel
         * runs at `omega * (radius + (l + w)/2) / r` — cap omega so it stays
         * inside the wheel-speed limit. Same lever for drive and strafe arcs.
         */
        [[nodiscard]] double maxYawRateForArcRadius(double radius_m,
                                                    bool /*lateral*/) const override
        {
            if (m_maxWheelSpeed <= 0.0)
                return std::numeric_limits<double>::infinity();
            const double lever = 0.5 * (m_wheelbase + m_trackWidth);
            const double max_wheel_mps = m_maxWheelSpeed * m_wheelRadius;
            return max_wheel_mps / (radius_m + lever);
        }

        /** Return the underlying motors in front-left, front-right, back-left, back-right order. */
        [[nodiscard]] std::vector<hal::motor::IMotor*> getMotors() override;

        [[nodiscard]] StmOdometryConfig getStmOdometryConfig() override;

        /** Set/get the per-axis velocity-command gain [vx, vy, wz]. */
        void setVelocityCommandGains(double gx, double gy, double gw) override;
        [[nodiscard]] std::array<double, 3> getVelocityCommandGains() const override;

        /** Command motors at raw open-loop power using mecanum inverse kinematics for direction. */
        void applyPowerCommand(const foundation::ChassisVelocity& direction,
                               int power_percent) override;
    };
}
