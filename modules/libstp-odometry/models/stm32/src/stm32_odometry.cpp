//
// STM32-sourced odometry implementation.
//

#include "odometry/stm32/stm32_odometry.hpp"
#include "odometry/angle_utils.hpp"
#include "foundation/config.hpp"
#include "core/LcmReader.hpp"
#include "core/LcmWriter.hpp"
#include <cmath>
#include <stdexcept>

namespace libstp::odometry::stm32
{
    Stm32Odometry::Stm32Odometry(std::shared_ptr<hal::imu::IIMU> imu,
                                   std::shared_ptr<kinematics::IKinematics> kinematics,
                                   Stm32OdometryConfig config)
        : config_(std::move(config))
        , imu_(std::move(imu))
        , kinematics_(std::move(kinematics))
    {
        if (!imu_) throw std::invalid_argument("imu cannot be null");
        if (!kinematics_) throw std::invalid_argument("kinematics cannot be null");

        imu_->setYawRateAxisMode(config_.turn_axis);

        if (!imu_->waitForReady(config_.imu_ready_timeout_ms)) {
            LIBSTP_LOG_WARN("Stm32Odometry::ctor - IMU not ready after {}ms timeout",
                          config_.imu_ready_timeout_ms);
        }

        sendKinematicsConfig();

        LIBSTP_LOG_TRACE("Stm32Odometry::ctor initialized — kinematics config sent to STM32");
    }

    void Stm32Odometry::sendKinematicsConfig()
    {
        auto cfg = kinematics_->getStmOdometryConfig();
        platform::wombat::core::LcmDataWriter::instance().sendKinematicsConfig(
            cfg.inv_matrix, cfg.ticks_to_rad, cfg.fwd_matrix);
    }

    void Stm32Odometry::update(double /*dt*/)
    {
        // No-op: the STM32 integrates autonomously.
        // The LcmReader background thread keeps the cache fresh.
    }

    foundation::Pose Stm32Odometry::getPose() const
    {
        auto snap = platform::wombat::core::LcmReader::instance().readOdometry();
        foundation::Pose pose;
        pose.position = Eigen::Vector3f(snap.pos_x, snap.pos_y, 0.0f);
        pose.heading = snap.heading;
        return pose;
    }

    DistanceFromOrigin Stm32Odometry::getDistanceFromOrigin() const
    {
        auto snap = platform::wombat::core::LcmReader::instance().readOdometry();

        const Eigen::Vector3f pos(snap.pos_x, snap.pos_y, 0.0f);

        const auto cos_o = static_cast<float>(std::cos(origin_heading_));
        const auto sin_o = static_cast<float>(std::sin(origin_heading_));
        const Eigen::Vector3f forward_at_origin(cos_o, sin_o, 0.0f);
        const Eigen::Vector3f right_at_origin(-sin_o, cos_o, 0.0f);

        return DistanceFromOrigin{
            pos.dot(forward_at_origin),
            pos.dot(right_at_origin),
            pos.norm()
        };
    }

    double Stm32Odometry::getHeading() const
    {
        auto snap = platform::wombat::core::LcmReader::instance().readOdometry();
        return wrapAngle(static_cast<double>(snap.heading));
    }

    double Stm32Odometry::getAbsoluteHeading() const
    {
        return imu_->getHeading();
    }

    double Stm32Odometry::getHeadingError(double target_heading_rad) const
    {
        return angularError(getHeading(), target_heading_rad);
    }

    void Stm32Odometry::reset()
    {
        if (imu_ && !imu_->waitForReady(config_.imu_ready_timeout_ms)) {
            LIBSTP_LOG_WARN("Stm32Odometry::reset - IMU not ready after {}ms timeout",
                          config_.imu_ready_timeout_ms);
        }

        if (imu_) {
            imu_->setYawRateAxisMode(config_.turn_axis);
        }

        origin_heading_ = 0.0;

        // Tell STM32 to zero its integrated pose
        platform::wombat::core::LcmDataWriter::instance().resetOdometry();

        // Re-send kinematics config (ensures STM32 encoder tracking is re-initialized)
        sendKinematicsConfig();

        // Block until the STM32 publishes near-zero odometry, confirming
        // the reset was processed.  This prevents the motion controller
        // from reading stale pre-reset position data.
        if (!platform::wombat::core::LcmReader::instance().waitForOdometryReset(500)) {
            LIBSTP_LOG_WARN("Stm32Odometry::reset — timed out waiting for STM32 reset confirmation");
        }

        // Zero the local cache once more to eliminate any sub-threshold residual
        platform::wombat::core::LcmReader::instance().resetOdometry();

        LIBSTP_LOG_TRACE("Stm32Odometry::reset — STM32 odometry reset confirmed");
    }
}
