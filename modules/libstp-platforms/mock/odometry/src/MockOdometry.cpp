//
// Mock-platform odometry — single source of truth for pose in tests/sim.
//
// Reads pose from the in-process MockPlatform/SimWorld. Behaves like a real
// STM32 coprocessor: pose is reported relative to the most recent reset, and
// zeroes when reset() is called. When no sim is attached, all queries return
// zero — mirroring the legacy mock OdometryBridge behavior tests rely on.
//
// The class is deliberately not exposed through a header: callers obtain an
// instance via `libstp::hal::platform::Platform::createOdometry()`.
//

#include "core/MockPlatform.hpp"
#include "foundation/config.hpp"
#include "foundation/logging.hpp"
#include "foundation/speed_mode_context.hpp"
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

    constexpr float kCmToMeters = 0.01f;

    class MockOdometry final : public IOdometry
    {
    public:
        explicit MockOdometry(std::shared_ptr<libstp::kinematics::IKinematics> kinematics)
            : kinematics_(std::move(kinematics))
        {
            // kinematics may be null in test contexts; MockOdometry does not use
            // it (the in-process SimWorld owns the dynamics). It is kept on the
            // signature to match the wombat factory and IOdometry contract.
            LIBSTP_LOG_TRACE("MockOdometry initialized");
        }

        void update(double /*dt*/) override
        {
            auto& platform = ::platform::mock::core::MockPlatform::instance();
            if (!platform.hasSim())
            {
                return;
            }
            platform.autoTickIfEnabled();
            const auto rel = platform.simRelativePose();
            const float x = rel.x * kCmToMeters;
            const float y = rel.y * kCmToMeters;
            if (path_initialized_)
            {
                const float dx = x - last_pos_x_;
                const float dy = y - last_pos_y_;
                path_length_ += std::sqrt(dx * dx + dy * dy);
            }
            last_pos_x_ = x;
            last_pos_y_ = y;
            path_initialized_ = true;
        }

        [[nodiscard]] libstp::foundation::Pose getPose() const override
        {
            libstp::foundation::SpeedModeContext::instance().assertBemfAvailable("IOdometry::getPose");
            auto& platform = ::platform::mock::core::MockPlatform::instance();
            libstp::foundation::Pose pose;
            if (!platform.hasSim())
            {
                pose.position = Eigen::Vector3f::Zero();
                pose.heading = 0.0f;
                return pose;
            }
            platform.autoTickIfEnabled();
            const auto rel = platform.simRelativePose();
            pose.position = Eigen::Vector3f(rel.x * kCmToMeters, rel.y * kCmToMeters, 0.0f);
            pose.heading = rel.theta;
            return pose;
        }

        [[nodiscard]] DistanceFromOrigin getDistanceFromOrigin() const override
        {
            libstp::foundation::SpeedModeContext::instance().assertBemfAvailable("IOdometry::getDistanceFromOrigin");
            const auto pose = getPose();
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

        [[nodiscard]] double getHeading() const override
        {
            auto& platform = ::platform::mock::core::MockPlatform::instance();
            if (!platform.hasSim())
            {
                return 0.0;
            }
            platform.autoTickIfEnabled();
            return wrapAngle(static_cast<double>(platform.simRelativePose().theta));
        }

        [[nodiscard]] double getAbsoluteHeading() const override
        {
            auto& platform = ::platform::mock::core::MockPlatform::instance();
            if (!platform.hasSim())
            {
                return 0.0;
            }
            platform.autoTickIfEnabled();
            return wrapAngle(static_cast<double>(platform.simPose().theta));
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

        void reset() override
        {
            origin_heading_ = 0.0;
            last_pos_x_ = 0.0f;
            last_pos_y_ = 0.0f;
            path_initialized_ = false;
            ::platform::mock::core::MockPlatform::instance().resetSimOrigin();
            LIBSTP_LOG_TRACE("MockOdometry::reset complete");
        }

    private:
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
        return std::make_shared<MockOdometry>(std::move(kinematics));
    }

    void Platform::commandChassisVelocity(float /*vx*/, float /*vy*/, float /*wz*/)
    {
        // No coprocessor on the mock platform — chassis velocity is a no-op.
    }
}
