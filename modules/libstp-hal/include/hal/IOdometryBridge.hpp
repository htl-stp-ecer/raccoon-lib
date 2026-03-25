//
// IOdometryBridge — HAL interface for coprocessor-based odometry communication.
//
// Abstracts the transport layer (LCM, mock, etc.) so odometry models
// can read pose data and send configuration without importing platform internals.
//
#pragma once

#include <array>

namespace libstp::hal::odometry_bridge
{
    /// Snapshot of coprocessor-computed odometry state.
    struct OdometrySnapshot {
        float pos_x{0.0f};
        float pos_y{0.0f};
        float heading{0.0f};
        float vx{0.0f};
        float vy{0.0f};
        float wz{0.0f};
    };

    /**
     * @brief Interface for coprocessor odometry communication.
     *
     * Platform drivers implement this to bridge between the odometry model
     * and the underlying transport (raccoon/LCM for wombat, in-process state
     * for mock, etc.).
     */
    struct IOdometryBridge
    {
        virtual ~IOdometryBridge() = default;

        /// Read the latest odometry snapshot from the coprocessor.
        [[nodiscard]] virtual OdometrySnapshot readOdometry() = 0;

        /// Zero the local odometry cache.
        virtual void resetLocalOdometry() = 0;

        /// Block until the coprocessor confirms a reset (near-zero pose).
        /// Returns true if confirmed, false on timeout.
        virtual bool waitForOdometryReset(int timeout_ms) = 0;

        /// Send kinematics configuration to the coprocessor for on-board integration.
        virtual void sendKinematicsConfig(
            const std::array<std::array<float, 4>, 3>& inv_matrix,
            const std::array<float, 4>& ticks_to_rad,
            const std::array<std::array<float, 3>, 4>& fwd_matrix) = 0;

        /// Request the coprocessor to zero its integrated odometry pose.
        virtual void resetCoprocessorOdometry() = 0;
    };
}
