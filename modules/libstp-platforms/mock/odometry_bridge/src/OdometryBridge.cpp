//
// Mock OdometryBridge — returns simulated odometry state from MockPlatform.
//

#include "hal/OdometryBridge.hpp"
#include "foundation/config.hpp"

namespace libstp::hal::odometry_bridge
{
    OdometryBridge::OdometryBridge() = default;

    OdometrySnapshot OdometryBridge::readOdometry()
    {
        // Mock returns zeroed state — tests can extend MockPlatform if needed.
        return OdometrySnapshot{};
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
