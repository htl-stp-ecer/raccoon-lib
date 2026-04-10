//
// Mock OdometryBridge — returns simulated odometry state from MockPlatform.
//

#include "hal/OdometryBridge.hpp"
#include "core/MockPlatform.hpp"
#include "foundation/config.hpp"

namespace libstp::hal::odometry_bridge
{
    namespace
    {
        constexpr float kCmToMeters = 0.01f;
    }

    OdometryBridge::OdometryBridge() = default;

    OdometrySnapshot OdometryBridge::readOdometry()
    {
        auto& platform = platform::mock::core::MockPlatform::instance();
        if (!platform.hasSim())
        {
            return OdometrySnapshot{};
        }

        const auto pose = platform.simPose();
        OdometrySnapshot snap{};
        snap.pos_x = pose.x * kCmToMeters;
        snap.pos_y = pose.y * kCmToMeters;
        snap.heading = pose.theta;
        snap.wz = platform.simYawRate();
        // vx/vy in body frame — we don't expose the body twist from SimWorld
        // yet; leave them zero until Step consumers need them.
        return snap;
    }

    void OdometryBridge::resetLocalOdometry()
    {
        LIBSTP_LOG_TRACE("MockOdometryBridge::resetLocalOdometry");
    }

    bool OdometryBridge::waitForOdometryReset(int /*timeout_ms*/)
    {
        // Mock resets are instantaneous.
        return true;
    }

    void OdometryBridge::sendKinematicsConfig(
        const std::array<std::array<float, 4>, 3>& /*inv_matrix*/,
        const std::array<float, 4>& /*ticks_to_rad*/,
        const std::array<std::array<float, 3>, 4>& /*fwd_matrix*/)
    {
        LIBSTP_LOG_TRACE("MockOdometryBridge::sendKinematicsConfig");
    }

    void OdometryBridge::resetCoprocessorOdometry()
    {
        LIBSTP_LOG_TRACE("MockOdometryBridge::resetCoprocessorOdometry");
    }
}
