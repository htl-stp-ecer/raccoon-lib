//
// OdometryBridge — concrete HAL class for coprocessor odometry communication.
//
// Method bodies are provided by the selected platform bundle (wombat, mock, etc.).
//
#pragma once

#include "hal/IOdometryBridge.hpp"
#include <memory>

namespace libstp::hal::odometry_bridge
{
    /**
     * Concrete odometry bridge whose method bodies are provided by the
     * selected platform bundle.
     *
     * Wombat: delegates to LcmReader/LcmDataWriter for STM32 communication.
     * Mock:   returns zeroed/simulated state from MockPlatform.
     */
    class OdometryBridge : public IOdometryBridge
    {
    public:
        OdometryBridge();
        ~OdometryBridge() override = default;

        [[nodiscard]] OdometrySnapshot readOdometry() override;
        void resetLocalOdometry() override;
        bool waitForOdometryReset(int timeout_ms) override;
        void sendKinematicsConfig(
            const std::array<std::array<float, 4>, 3>& inv_matrix,
            const std::array<float, 4>& ticks_to_rad,
            const std::array<std::array<float, 3>, 4>& fwd_matrix) override;
        void resetCoprocessorOdometry() override;
    };
}
