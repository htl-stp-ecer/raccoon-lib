//
// Wombat-platform odometry — single source of truth for pose on real hardware.
//
// Reads STM32-computed dead-reckoning state directly over transport (no bridge
// indirection). Pushes the kinematics matrix to the coprocessor at construction
// so the STM32 can integrate at full BEMF sample rate.
//
// The class is deliberately not exposed through a header: callers obtain an
// instance via `libstp::hal::platform::Platform::createOdometry()`.
//

#include "core/TransportReader.hpp"
#include "core/TransportWriter.hpp"
#include "foundation/chassis_control_context.hpp"
#include "foundation/config.hpp"
#include "foundation/logging.hpp"
#include "foundation/speed_mode_context.hpp"
#include "hal/IIMU.hpp"
#include "hal/IMU.hpp"
#include "hal/Platform.hpp"
#include "kinematics/kinematics.hpp"
#include "hal/angle_utils.hpp"
#include "hal/odometry.hpp"

#include <Eigen/Core>
#include <atomic>
#include <cmath>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <utility>

namespace
{
    using libstp::odometry::DistanceFromOrigin;
    using libstp::odometry::IOdometry;
    using libstp::odometry::OdometrySource;
    using libstp::odometry::wrapAngle;
    using libstp::odometry::angularError;

    constexpr int kImuReadyTimeoutMs = 1000;
    constexpr int kStm32ResetTimeoutMs = 500;

    const char* sourceName(OdometrySource source)
    {
        switch (source)
        {
        case OdometrySource::CalibrationBoard:
            return "CALIBRATION_BOARD";
        case OdometrySource::Internal:
        default:
            return "INTERNAL";
        }
    }

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
            // Accumulate path length from whichever source currently drives the
            // primary pose. On a source switch the baseline is dropped so the
            // discontinuity between the two coordinate frames is not counted as
            // travelled distance. Read the raw caches directly (no BEMF
            // assertion) so path tracking never throws from the update loop.
            const OdometrySource source = resolveSource();
            float x;
            float y;
            if (source == OdometrySource::CalibrationBoard)
            {
                const auto snap =
                    ::platform::wombat::core::TransportReader::instance().readCalibOdometry();
                x = snap.pos_x;
                y = snap.pos_y;
            }
            else
            {
                const auto snap =
                    ::platform::wombat::core::TransportReader::instance().readOdometry();
                x = snap.pos_x;
                y = snap.pos_y;
            }

            if (path_initialized_ && source == path_source_)
            {
                const float dx = x - last_pos_x_;
                const float dy = y - last_pos_y_;
                path_length_ += std::sqrt(dx * dx + dy * dy);
            }
            last_pos_x_ = x;
            last_pos_y_ = y;
            path_source_ = source;
            path_initialized_ = true;
        }

        [[nodiscard]] libstp::foundation::Pose getPose() const override
        {
            return poseFor(resolveSource());
        }

        [[nodiscard]] DistanceFromOrigin getDistanceFromOrigin() const override
        {
            return distanceFromOrigin(poseFor(resolveSource()));
        }

        [[nodiscard]] double getHeading() const override
        {
            return wrapAngle(static_cast<double>(poseFor(resolveSource()).heading));
        }

        [[nodiscard]] double getAbsoluteHeading() const override
        {
            return imu_->getHeading();
        }

        [[nodiscard]] double getPathLength() const override
        {
            libstp::foundation::SpeedModeContext::instance().assertBemfAvailable("IOdometry::getPathLength");
            return path_length_;
        }

        [[nodiscard]] double getHeadingError(double target_heading_rad) const override
        {
            return angularError(getHeading(), target_heading_rad);
        }

        [[nodiscard]] OdometrySource getActiveSource() const override
        {
            return resolveSource();
        }

        [[nodiscard]] libstp::foundation::Pose getPoseFromSource(OdometrySource source) const override
        {
            // Direct, preference-independent read so callers can passively
            // compare the calibration board against the internal estimate
            // without re-routing the motion system or resetting the frame.
            return poseFor(source);
        }

        [[nodiscard]] bool isSourceAvailable(OdometrySource source) const override
        {
            if (source == OdometrySource::Internal)
                return true;
            return ::platform::wombat::core::TransportReader::instance().isCalibBoardConnected();
        }

        void setPreferredSource(OdometrySource source) override
        {
            std::lock_guard<std::mutex> lock(source_switch_mutex_);
            if (preferred_source_ == source)
                return;
            preferred_source_ = source;
            LIBSTP_LOG_INFO("[Odometry] preferred source -> {}", sourceName(source));
        }

        [[nodiscard]] OdometrySource getPreferredSource() const override
        {
            std::lock_guard<std::mutex> lock(source_switch_mutex_);
            return preferred_source_;
        }

        [[nodiscard]] libstp::foundation::Pose getInternalPose() const override
        {
            return internalPose();
        }

        [[nodiscard]] double getInternalHeading() const override
        {
            return wrapAngle(static_cast<double>(internalPose().heading));
        }

        [[nodiscard]] DistanceFromOrigin getInternalDistanceFromOrigin() const override
        {
            return distanceFromOrigin(internalPose());
        }

        void reset() override
        {
            performReset("manual reset");
        }

    private:
        void performReset(const char* reason)
        {
            if (!imu_->waitForReady(kImuReadyTimeoutMs))
            {
                LIBSTP_LOG_WARN("WombatOdometry::reset IMU not ready after {}ms",
                                kImuReadyTimeoutMs);
            }

            origin_heading_ = 0.0;
            path_initialized_ = false;

            ::platform::wombat::core::TransportWriter::instance().resetOdometry();
            // Keep the external reference aligned with the internal frame so the
            // two can be compared directly while tuning.
            ::platform::wombat::core::TransportWriter::instance().resetCalibOdometry();
            sendKinematicsConfig();

            if (!::platform::wombat::core::TransportReader::instance().waitForOdometryReset(
                    kStm32ResetTimeoutMs))
            {
                LIBSTP_LOG_WARN("WombatOdometry::reset timed out waiting for STM32 confirmation");
            }
            ::platform::wombat::core::TransportReader::instance().resetOdometry();

            LIBSTP_LOG_INFO("WombatOdometry::reset complete ({})", reason);
        }

        /// Decide which source backs the primary pose right now. If the source
        /// flips mid-run, log it and hard-reset the odometry frame so callers
        /// never stitch together samples from two different systems.
        [[nodiscard]] OdometrySource resolveSource() const
        {
            const bool calib =
                ::platform::wombat::core::TransportReader::instance().isCalibBoardConnected();
            std::lock_guard<std::mutex> lock(source_switch_mutex_);
            const OdometrySource preferred = preferred_source_;
            const OdometrySource source =
                (preferred == OdometrySource::CalibrationBoard && calib)
                    ? OdometrySource::CalibrationBoard
                    : OdometrySource::Internal;
            const OdometrySource previous = active_source_;
            if (!source_initialized_)
            {
                active_source_ = source;
                source_initialized_ = true;
                return source;
            }

            if (source != previous)
            {
                LIBSTP_LOG_INFO("[Odometry] source {} -> {}; hard-resetting odometry frame",
                                sourceName(previous), sourceName(source));
                active_source_ = source;
                const_cast<WombatOdometry*>(this)->performReset("odometry source swap");
            }
            return source;
        }

        /// Pose from the requested source, in meters / radians.
        [[nodiscard]] libstp::foundation::Pose poseFor(OdometrySource source) const
        {
            return source == OdometrySource::CalibrationBoard ? calibPose() : internalPose();
        }

        /// Cheap STM32 dead-reckoning pose. Requires BEMF telemetry.
        [[nodiscard]] libstp::foundation::Pose internalPose() const
        {
            libstp::foundation::SpeedModeContext::instance().assertBemfAvailable("IOdometry::getPose");
            const auto snap = ::platform::wombat::core::TransportReader::instance().readOdometry();
            libstp::foundation::Pose pose;
            pose.position = Eigen::Vector3f(snap.pos_x, snap.pos_y, 0.0f);
            pose.heading = snap.heading;
            return pose;
        }

        /// External calibration-board pose (already converted to m / rad).
        /// Independent of BEMF, so no speed-mode assertion is needed.
        [[nodiscard]] libstp::foundation::Pose calibPose() const
        {
            const auto snap = ::platform::wombat::core::TransportReader::instance().readCalibOdometry();
            libstp::foundation::Pose pose;
            // The calib board's optical-flow position frame is mounted rotated
            // 180° from the robot drive frame: a forward drive shows as (−x,−y)
            // in the raw board position while the board heading points ~forward.
            // Confirmed on hardware as a CONSTANT 180° rotation (displacement
            // angle − heading ≈ −180° at every heading; not a reflection).
            // Negate the position to undo it so heading-relative consumers
            // (LinearMotion projects displacement onto the start heading) see a
            // forward drive as +forward. Heading is left untouched — turn
            // control nulls heading ERROR so a constant offset cancels — and the
            // straight-line norm is sign-independent (Phase 5/6 + velocity-gain
            // calibration stay valid).
            pose.position = Eigen::Vector3f(-snap.pos_x, -snap.pos_y, 0.0f);
            pose.heading = snap.heading;
            return pose;
        }

        /// Project a pose onto the origin frame fixed by the last reset().
        [[nodiscard]] DistanceFromOrigin distanceFromOrigin(
            const libstp::foundation::Pose& pose) const
        {
            const Eigen::Vector3f& pos = pose.position;

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

        void sendKinematicsConfig()
        {
            const auto cfg = kinematics_->getStmOdometryConfig();
            ::platform::wombat::core::TransportWriter::instance().sendKinematicsConfig(
                cfg.inv_matrix, cfg.ticks_to_rad, cfg.fwd_matrix, cfg.bemf_offset);
        }

        std::shared_ptr<libstp::hal::imu::IIMU> imu_;
        std::shared_ptr<libstp::kinematics::IKinematics> kinematics_;

        double origin_heading_{0.0};
        double path_length_{0.0};
        float last_pos_x_{0.0f};
        float last_pos_y_{0.0f};
        bool path_initialized_{false};
        OdometrySource path_source_{OdometrySource::Internal};

        mutable std::mutex source_switch_mutex_;
        mutable bool source_initialized_{false};
        OdometrySource preferred_source_{OdometrySource::Internal};
        mutable OdometrySource active_source_{OdometrySource::Internal};
    };
}

namespace libstp::hal::platform
{
    namespace
    {
        // On-MCU chassis sink registered with the foundation context: forwards a
        // body-frame velocity command to the STM32 (which does the IK + per-motor
        // PID). Non-capturing so it stores as a plain function pointer.
        bool wombatChassisSink(double vx, double vy, double wz)
        {
            ::platform::wombat::core::TransportWriter::instance().setChassisVelocity(
                static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(wz));
            return true;
        }
    }

    std::shared_ptr<libstp::odometry::IOdometry> Platform::createOdometry(
        std::shared_ptr<libstp::kinematics::IKinematics> kinematics)
    {
        auto imu = std::make_shared<libstp::hal::imu::IMU>();
        auto odom = std::make_shared<WombatOdometry>(std::move(imu), std::move(kinematics));

        // Register the on-MCU chassis loop so Drive::update routes body-velocity
        // commands to the STM32 instead of running host-side IK. Done here: this
        // runs once at robot startup and is where the kinematics matrices the
        // STM32 needs for its IK are pushed.
        libstp::foundation::ChassisControlContext::instance().setSink(&wombatChassisSink);

        return odom;
    }

    bool Platform::commandChassisVelocity(float vx, float vy, float wz)
    {
        ::platform::wombat::core::TransportWriter::instance().setChassisVelocity(vx, vy, wz);
        return true;
    }
}
