//
// Wombat OdometryBridge — delegates to LcmReader/LcmDataWriter for STM32 communication.
//

#include "hal/OdometryBridge.hpp"
#include "core/LcmReader.hpp"
#include "core/LcmWriter.hpp"

namespace libstp::hal::odometry_bridge
{
    OdometryBridge::OdometryBridge() = default;

    OdometrySnapshot OdometryBridge::readOdometry()
    {
        auto snap = platform::wombat::core::LcmReader::instance().readOdometry();
        return OdometrySnapshot{
            snap.pos_x, snap.pos_y, snap.heading,
            snap.vx, snap.vy, snap.wz
        };
    }

    void OdometryBridge::resetLocalOdometry()
    {
        platform::wombat::core::LcmReader::instance().resetOdometry();
    }

    bool OdometryBridge::waitForOdometryReset(int timeout_ms)
    {
        return platform::wombat::core::LcmReader::instance().waitForOdometryReset(timeout_ms);
    }

    void OdometryBridge::sendKinematicsConfig(
        const std::array<std::array<float, 4>, 3>& inv_matrix,
        const std::array<float, 4>& ticks_to_rad,
        const std::array<std::array<float, 3>, 4>& fwd_matrix)
    {
        platform::wombat::core::LcmDataWriter::instance().sendKinematicsConfig(
            inv_matrix, ticks_to_rad, fwd_matrix);
    }

    void OdometryBridge::resetCoprocessorOdometry()
    {
        platform::wombat::core::LcmDataWriter::instance().resetOdometry();
    }
}
