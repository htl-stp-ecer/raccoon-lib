//
// Created by tobias on 9/8/25.
//

#pragma once
#include <cstddef>
#include <vector>

#include "calibration/motor/calibration_config.hpp"
#include "calibration/motor/calibration_result.hpp"
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
         * Run the module's calibration flow for each managed drive motor.
         *
         * Implementations define ordering and any dwell time between motors.
         */
        virtual std::vector<calibration::CalibrationResult> calibrateMotors(
            const calibration::CalibrationConfig& config) = 0;

        /** Convenience overload that uses the default calibration config. */
        virtual std::vector<calibration::CalibrationResult> calibrateMotors();

        /**
         * @brief Get the wheel radius used by this kinematics model
         * @return Wheel radius in meters
         */
        [[nodiscard]] virtual double getWheelRadius() const = 0;

        /**
         * @brief Get pointers to all drive motors managed by this kinematics model
         * @return Vector of motor pointers (non-owning)
         */
        [[nodiscard]] virtual std::vector<hal::motor::IMotor*> getMotors() const = 0;
    };
}
