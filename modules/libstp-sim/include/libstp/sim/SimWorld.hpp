#pragma once

#include "libstp/sim/Pose2D.hpp"
#include "libstp/sim/RobotConfig.hpp"
#include "libstp/sim/WorldMap.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace libstp::sim
{
    /// Motor mapping + kinematic parameters the sim needs beyond the pure
    /// geometry in RobotConfig. Kept here (not in RobotConfig) because this is
    /// sim-specific tuning, not robot ground truth.
    struct SimMotorMap
    {
        uint8_t leftPort{0};
        uint8_t rightPort{1};
        bool leftInverted{false};
        bool rightInverted{false};

        /// Wheel angular velocity (rad/s) produced by a 100% command.
        /// Used as the linear scale from commanded signed percent to wheel ω.
        float maxWheelVelocityRadS{30.0f};

        /// First-order motor lag in seconds. wheel_ω approaches target_ω with
        /// time constant τ (≈ rise time from 0 to 63% of a step).
        float motorTimeConstantSec{0.1f};
    };

    /// Ground-truth simulated robot. Owns a pose, integrates motor commands
    /// through a diff-drive kinematic model, and snaps motion through the
    /// collision layer so it respects walls.
    ///
    /// **Not** a singleton by design — tests create their own and the mock
    /// HAL will hold a single instance at the platform layer.
    class SimWorld
    {
    public:
        SimWorld() = default;

        void configure(const RobotConfig& robot, const SimMotorMap& motors);
        void setMap(WorldMap map);
        const WorldMap& map() const noexcept { return m_map; }

        void setPose(const Pose2D& pose);
        Pose2D pose() const noexcept { return m_pose; }
        float yawRateRadS() const noexcept { return m_yawRateRadS; }

        /// Signed motor command in the range [-100, 100]. Matches the scale
        /// used by HAL Motor::setSpeed.
        void setMotorCommand(uint8_t port, int signedPercent);

        /// Advance the simulation by dt seconds (fixed-step integration for
        /// determinism).
        void tick(float dt);

        /// World-frame position of a sensor mount attached to this robot.
        Vec2 sensorWorldPosition(const SensorMount& mount) const;

        /// Convenience: true iff the line sensor mount is currently over a
        /// black line, with optional noise in the 0..1 probability domain
        /// (default 0 = deterministic).
        bool isSensorOnBlackLine(const SensorMount& mount) const;

        /// Current left/right wheel angular velocity (rad/s) — exposed for
        /// tests and for simulated BEMF output.
        float wheelOmegaLeft() const noexcept { return m_wheelOmega[kLeft]; }
        float wheelOmegaRight() const noexcept { return m_wheelOmega[kRight]; }

        const std::vector<Pose2D>& trace() const noexcept { return m_trace; }
        void clearTrace() { m_trace.clear(); }

    private:
        static constexpr std::size_t kNumMotorPorts = 4;
        static constexpr std::size_t kLeft = 0;
        static constexpr std::size_t kRight = 1;

        RobotConfig m_robot{};
        SimMotorMap m_motors{};
        WorldMap m_map{};
        Pose2D m_pose{};

        std::array<int, kNumMotorPorts> m_motorCommand{};        // signed percent
        std::array<float, 2> m_wheelOmega{};                      // rad/s; [left,right]
        float m_yawRateRadS{0.0f};
        std::vector<Pose2D> m_trace;

        float targetOmega(std::size_t wheelIndex) const;
    };
}
