//
// Created by tobias on 9/8/25.
//

#pragma once
#include <cstddef>
#include <vector>

#include "calibration/motor/calibration_config.hpp"
#include "calibration/motor/calibration_result.hpp"
#include "foundation/types.hpp"

namespace libstp::kinematics
{
    struct MotorCommands
    {
        std::vector<double> wheel_velocities{};
        bool saturated_any{false};
        std::uint32_t saturation_mask{0};
    };

    struct IKinematics
    {
        virtual ~IKinematics() = default;
        [[nodiscard]] virtual std::size_t wheelCount() const = 0;

        virtual MotorCommands applyCommand(const foundation::ChassisVelocity& cmd, double dt) = 0;

        [[nodiscard]] virtual foundation::ChassisVelocity estimateState() const = 0;

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

        virtual std::vector<calibration::CalibrationResult> calibrateMotors(
            const calibration::CalibrationConfig& config) = 0;

        virtual std::vector<calibration::CalibrationResult> calibrateMotors();
    };
}
