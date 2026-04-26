//
// Created by tobias on 9/8/25.
//

#pragma once
#include <array>
#include <cstddef>
#include <vector>

#include "foundation/types.hpp"
#include "hal/IMotor.hpp"

namespace libstp::kinematics
{
    /**
     * Result of mapping a chassis-space command onto the drivetrain.
     *
     * `wheel_velocities` is the primary output used by current callers. The
     * saturation fields are reserved for models that implement explicit limit
     * handling and want to report which wheel commands were clipped.
     */
    struct MotorCommands
    {
        std::vector<double> wheel_velocities{};
        bool saturated_any{false};
        std::uint32_t saturation_mask{0};
    };

    /**
     * Abstract drivetrain kinematics contract.
     *
     * Implementations own the mapping between chassis-space velocity commands
     * and the wheel-space commands understood by a specific drivetrain layout.
     * They also provide encoder-based chassis velocity estimation for drive
     * control and odometry.
     */
    struct IKinematics
    {
        virtual ~IKinematics() = default;

        /** Return the number of drive wheels modeled by this instance. */
        [[nodiscard]] virtual std::size_t wheelCount() const = 0;

        /**
         * Apply a chassis-space velocity command.
         *
         * @param cmd Desired body-frame velocity using the shared `vx`, `vy`,
         *            `wz` convention from `foundation::ChassisVelocity`.
         * @param dt Control-loop time step in seconds.
         * @return Per-wheel command data produced by the model.
         */
        virtual MotorCommands applyCommand(const foundation::ChassisVelocity& cmd, double dt) = 0;

        /**
         * Estimate the current chassis-space velocity from motor feedback.
         *
         * The returned value stays in the body frame. World-frame integration is
         * handled by odometry modules.
         */
        [[nodiscard]] virtual foundation::ChassisVelocity estimateState() const = 0;

        /** Immediately brake or otherwise stop all managed wheels. */
        virtual void hardStop() = 0;

        /**
         * @brief Query whether this kinematics model supports direct lateral (vy) motion
         * @return true if the robot can strafe sideways (e.g., mecanum, omni wheels)
         *         false if lateral motion requires rotation (e.g., differential drive)
         */
        [[nodiscard]] virtual bool supportsLateralMotion() const = 0;

        /**
         * @brief Reset encoder/BEMF counters to prepare for new odometry origin
         *
         * This should be called when odometry is reset to prevent stale counter values
         * from causing incorrect velocity/position estimates in the next update cycle.
         */
        virtual void resetEncoders() = 0;

        /**
         * @brief Get the wheel radius used by this kinematics model
         * @return Wheel radius in meters
         */
        [[nodiscard]] virtual double getWheelRadius() const = 0;

        /**
         * @brief Get pointers to all drive motors managed by this kinematics model
         * @return Vector of motor pointers (non-owning)
         *
         * Non-const because the returned pointers are used to command motors,
         * which is a write through the kinematics. A `const Kinematics&`
         * cannot hand out mutating handles.
         */
        [[nodiscard]] virtual std::vector<hal::motor::IMotor*> getMotors() = 0;

        /**
         * Pre-baked inverse kinematics matrix for STM32-side odometry.
         *
         * Returns a 3×4 row-major matrix that maps wheel angular velocities
         * (rad/s) directly to body-frame [vx (m/s), vy (m/s), wz (rad/s)].
         * Wheel radius and geometry constants are already folded into the
         * coefficients. Unused wheel columns are zero (e.g. differential
         * drive uses only columns 0 and 1).
         *
         * Also returns per-motor ticks_to_rad calibration values.
         */
        struct StmOdometryConfig
        {
            std::array<std::array<float, 4>, 3> inv_matrix{};
            std::array<float, 4> ticks_to_rad{};
            std::array<std::array<float, 3>, 4> fwd_matrix{};
        };

        // Non-const: needs to look up motor handles via getMotors() to
        // reorder kinematics-slot data into hardware-port order.
        [[nodiscard]] virtual StmOdometryConfig getStmOdometryConfig() = 0;

        /**
         * Command motors at raw open-loop power using the inverse kinematics
         * to determine per-wheel direction signs.
         *
         * The `direction` vector is passed through the same inverse kinematics
         * as `applyCommand`, but instead of converting to velocity targets, the
         * resulting wheel speed ratios are normalized so the largest wheel
         * receives exactly `power_percent` and every other wheel is scaled
         * proportionally.  Each motor is then driven via `setSpeed()` (raw PWM).
         *
         * This bypasses all firmware velocity PID and library velocity control,
         * giving direct open-loop access to the motors while still respecting
         * the drivetrain geometry.
         *
         * @param direction Desired chassis-space motion direction.  Only the
         *        ratios between `vx`, `vy`, and `wz` matter; the magnitude
         *        is normalized away.
         * @param power_percent Motor power from -100 to 100.  Sign is combined
         *        with the direction to determine final per-wheel sign.
         */
        virtual void applyPowerCommand(const foundation::ChassisVelocity& direction,
                                       int power_percent) = 0;
    };
}
