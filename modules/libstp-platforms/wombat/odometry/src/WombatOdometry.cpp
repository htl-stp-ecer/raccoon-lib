//
// Wombat-platform odometry — single source of truth for pose on real hardware.
//
// Reads STM32-computed dead-reckoning state directly over LCM (no bridge
// indirection). Pushes the kinematics matrix to the coprocessor at construction
// so the STM32 can integrate at full BEMF sample rate.
//
// The class is deliberately not exposed through a header: callers obtain an
// instance via `libstp::hal::platform::Platform::createOdometry()`.
//

#include "core/LcmReader.hpp"
#include "core/LcmWriter.hpp"
#include "foundation/config.hpp"
#include "foundation/logging.hpp"
#include "hal/IIMU.hpp"
#include "hal/IMU.hpp"
#include "hal/Platform.hpp"
#include "kinematics/kinematics.hpp"
#include "hal/angle_utils.hpp"
#include "hal/odometry.hpp"

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

namespace
{
    using libstp::odometry::DistanceFromOrigin;
    using libstp::odometry::IOdometry;
    using libstp::odometry::wrapAngle;
    using libstp::odometry::angularError;

    constexpr int kImuReadyTimeoutMs = 1000;
    constexpr int kStm32ResetTimeoutMs = 500;

    class WombatOdometry final : public IOdometry
    {
    public:
        WombatOdometry(std::shared_ptr<libstp::hal::imu::IIMU> imu,
                       std::shared_ptr<libstp::kinematics::IKinematics> kinematics)
            : imu_(std::move(imu))
            , kinematics_(std::move(kinematics))
        {
            if (!imu_) throw std::invalid_argument("imu cannot be null");
            if (!kinematics_) throw std::invalid_argument("kinematics cannot be null");

            if (!imu_->waitForReady(kImuReadyTimeoutMs))
            {
                LIBSTP_LOG_WARN("WombatOdometry::ctor IMU not ready after {}ms",
                                kImuReadyTimeoutMs);
            }

            sendKinematicsConfig();
            LIBSTP_LOG_TRACE("WombatOdometry initialized");
        }

        void update(double /*dt*/) override
        {
            const auto snap = ::platform::wombat::core::LcmReader::instance().readOdometry();
            if (path_initialized_)
            {
                const float dx = snap.pos_x - last_pos_x_;
                const float dy = snap.pos_y - last_pos_y_;
                path_length_ += std::sqrt(dx * dx + dy * dy);
            }
            last_pos_x_ = snap.pos_x;
            last_pos_y_ = snap.pos_y;
            path_initialized_ = true;
        }

        [[nodiscard]] libstp::foundation::Pose getPose() const override
        {
            const auto snap = ::platform::wombat::core::LcmReader::instance().readOdometry();
            libstp::foundation::Pose pose;
            pose.position = Eigen::Vector3f(snap.pos_x, snap.pos_y, 0.0f);
            pose.heading = snap.heading;
            return pose;
        }

        [[nodiscard]] DistanceFromOrigin getDistanceFromOrigin() const override
        {
            const auto snap = ::platform::wombat::core::LcmReader::instance().readOdometry();
            const Eigen::Vector3f pos(snap.pos_x, snap.pos_y, 0.0f);

            const auto cos_o = static_cast<float>(std::cos(origin_heading_));
            const auto sin_o = static_cast<float>(std::sin(origin_heading_));
            const Eigen::Vector3f forward_at_origin(cos_o, sin_o, 0.0f);
            const Eigen::Vector3f right_at_origin(-sin_o, cos_o, 0.0f);

            return DistanceFromOrigin{
                pos.dot(forward_at_origin),
                pos.dot(right_at_origin),
                pos.norm(),
            };
        }

        [[nodiscard]] double getHeading() const override
        {
            const auto snap = ::platform::wombat::core::LcmReader::instance().readOdometry();
            return wrapAngle(static_cast<double>(snap.heading));
        }

        [[nodiscard]] double getAbsoluteHeading() const override
        {
            return imu_->getHeading();
        }

        [[nodiscard]] double getPathLength() const override
        {
            return path_length_;
        }

        [[nodiscard]] double getHeadingError(double target_heading_rad) const override
        {
            return angularError(getHeading(), target_heading_rad);
        }

        void reset() override
        {
            if (!imu_->waitForReady(kImuReadyTimeoutMs))
            {
                LIBSTP_LOG_WARN("WombatOdometry::reset IMU not ready after {}ms",
                                kImuReadyTimeoutMs);
            }

            origin_heading_ = 0.0;

            ::platform::wombat::core::LcmDataWriter::instance().resetOdometry();
            sendKinematicsConfig();

            if (!::platform::wombat::core::LcmReader::instance().waitForOdometryReset(
                    kStm32ResetTimeoutMs))
            {
                LIBSTP_LOG_WARN("WombatOdometry::reset timed out waiting for STM32 confirmation");
            }
            ::platform::wombat::core::LcmReader::instance().resetOdometry();

            LIBSTP_LOG_WARN("WombatOdometry::reset complete");
        }

    private:
        void sendKinematicsConfig()
        {
            const auto cfg = kinematics_->getStmOdometryConfig();
            ::platform::wombat::core::LcmDataWriter::instance().sendKinematicsConfig(
                cfg.inv_matrix, cfg.ticks_to_rad, cfg.fwd_matrix);
        }

        std::shared_ptr<libstp::hal::imu::IIMU> imu_;
        std::shared_ptr<libstp::kinematics::IKinematics> kinematics_;

        double origin_heading_{0.0};
        double path_length_{0.0};
        float last_pos_x_{0.0f};
        float last_pos_y_{0.0f};
        bool path_initialized_{false};
    };
}

namespace libstp::hal::platform
{
    std::shared_ptr<libstp::odometry::IOdometry> Platform::createOdometry(
        std::shared_ptr<libstp::kinematics::IKinematics> kinematics)
    {
        auto imu = std::make_shared<libstp::hal::imu::IMU>();
        return std::make_shared<WombatOdometry>(std::move(imu), std::move(kinematics));
    }
}
