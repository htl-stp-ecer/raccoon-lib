//
// STM32-sourced odometry — reads dead-reckoning state computed on the
// coprocessor instead of integrating locally.
//

#pragma once

#include <memory>
#include <string>

#include "odometry/odometry.hpp"
#include "hal/IIMU.hpp"
#include "hal/IOdometryBridge.hpp"
#include "kinematics/kinematics.hpp"

namespace libstp::odometry::stm32
{
    struct Stm32OdometryConfig {
        int imu_ready_timeout_ms{1000};

        /// Which axis to use for yaw rate extraction (forwarded to IIMU).
        std::string turn_axis{"world_z"};
    };

    /**
     * Odometry implementation that reads pose/velocity from the STM32
     * coprocessor via an IOdometryBridge.
     *
     * At construction the pre-baked inverse kinematics matrix is sent to the
     * STM32 so it can run dead reckoning on-board at full BEMF sample rate.
     * All getters simply return the latest snapshot from the coprocessor.
     */
    class Stm32Odometry : public IOdometry
    {
    public:
        Stm32Odometry(std::shared_ptr<hal::imu::IIMU> imu,
                       std::shared_ptr<kinematics::IKinematics> kinematics,
                       std::shared_ptr<hal::odometry_bridge::IOdometryBridge> bridge,
                       Stm32OdometryConfig config = {});

        void update(double dt) override;

        [[nodiscard]] foundation::Pose getPose() const override;
        [[nodiscard]] DistanceFromOrigin getDistanceFromOrigin() const override;
        [[nodiscard]] double getHeading() const override;
        [[nodiscard]] double getAbsoluteHeading() const override;
        [[nodiscard]] double getPathLength() const override;
        [[nodiscard]] double getHeadingError(double target_heading_rad) const override;
        void reset() override;

    private:
        Stm32OdometryConfig config_;
        std::shared_ptr<hal::imu::IIMU> imu_;
        std::shared_ptr<kinematics::IKinematics> kinematics_;
        std::shared_ptr<hal::odometry_bridge::IOdometryBridge> bridge_;

        // Origin tracking (for getDistanceFromOrigin)
        double origin_heading_{0.0};

        // Cumulative path length (monotonic odometer, not affected by reset)
        double path_length_{0.0};
        float last_pos_x_{0.0f};
        float last_pos_y_{0.0f};
        bool path_initialized_{false};

        void sendKinematicsConfig();
    };
}
