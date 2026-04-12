#pragma once

#include "libstp/sim/Pose2D.hpp"
#include "libstp/sim/RobotConfig.hpp"
#include "libstp/sim/WorldMap.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
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

        /// Motor encoder calibration: radians per encoder tick. Matches the
        /// default in foundation::MotorCalibration (≈ 2π/1440). Used to
        /// convert between BEMF integer commands (Motor::setVelocity) and
        /// wheel angular velocity inside the sim.
        float ticksToRad{0.00436332313f};

        /// BEMF sample rate in Hz used by MotorAdapter when encoding wheel ω
        /// as a BEMF integer. The wombat firmware samples motor BEMF at
        /// 200 Hz, so `setVelocity(N)` means "N encoder ticks per 5 ms".
        /// The sim must use the same scale to recover wheel ω from the
        /// commanded integer.
        float bemfSampleRateHz{200.0f};

        /// Viscous (velocity-proportional) drag coefficient in 1/s. At each
        /// tick the wheel ω is reduced by ``viscousDragCoeff * ω * dt``.
        /// Models bearing friction and back-EMF drag that scale with speed.
        /// Default 0 = no viscous drag (legacy behavior).
        float viscousDragCoeff{0.0f};

        /// Coulomb (dry/constant) friction in rad/s². A constant angular
        /// deceleration opposing motion, independent of speed. Models
        /// gearbox friction and wheel-surface interaction. Clamped so it
        /// cannot reverse direction. Default 0 = no Coulomb friction.
        float coulombFrictionRadSS{0.0f};

        /// Standard deviation of Gaussian noise added to BEMF integer
        /// readings (in raw BEMF units). Models real encoder quantization
        /// and electrical noise. Default 0 = clean readings.
        float bemfNoiseStddev{0.0f};
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

        /// Register a line sensor on `analogPort`. Sampled by `readAnalog`:
        /// returns ~0 when the mount is over a black line, ~1023 otherwise.
        void attachLineSensor(uint8_t analogPort, float forwardCm, float strafeCm,
                              std::string name = {});

        /// Register a distance sensor on `analogPort`. Uses raycast against
        /// walls. Returns the KIPR ET empirical polynomial value in [0, 4095]
        /// truncated to [0, 1023] to match the analog ADC range.
        void attachDistanceSensor(uint8_t analogPort, float forwardCm, float strafeCm,
                                  float mountAngleRad = 0.0f,
                                  float maxRangeCm = 100.0f,
                                  std::string name = {});

        /// Detach any sensor bound to `analogPort`.
        void detachSensor(uint8_t analogPort);

        /// Sample the analog value currently visible at `analogPort`. Returns
        /// `std::nullopt` if no sensor is registered on that port (the mock
        /// HAL then falls through to its stock static analog values).
        std::optional<uint16_t> readAnalog(uint8_t analogPort) const;

        void setPose(const Pose2D& pose);
        Pose2D pose() const noexcept { return m_pose; }
        float yawRateRadS() const noexcept { return m_yawRateRadS; }

        /// Signed motor command in the range [-100, 100]. Matches the scale
        /// used by HAL Motor::setSpeed.
        void setMotorCommand(uint8_t port, int signedPercent);

        /// Closed-loop velocity command in BEMF integer units (matches
        /// HAL Motor::setVelocity). Converted internally to a wheel ω target
        /// via the SimMotorMap.ticksToRad calibration.
        void setMotorVelocityCommand(uint8_t port, int bemfUnits);

        /// Live BEMF feedback for `port`, derived from the sim's current
        /// wheel ω and the calibration. Returned in raw integer BEMF units so
        /// it round-trips with `Motor::getBemf`.
        int motorBemf(uint8_t port) const;

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

        enum class SensorKind : uint8_t { Line, Distance };

        struct SensorEntry
        {
            SensorKind kind{SensorKind::Line};
            SensorMount mount{};
            float mountAngleRad{0.0f};  // distance sensor aim direction in robot frame
            float maxRangeCm{100.0f};
        };

        RobotConfig m_robot{};
        SimMotorMap m_motors{};
        WorldMap m_map{};
        Pose2D m_pose{};

        std::array<float, kNumMotorPorts> m_motorTargetOmega{};   // rad/s, signed
        std::array<float, 2> m_wheelOmega{};                      // rad/s; [left,right]
        float m_yawRateRadS{0.0f};
        std::vector<Pose2D> m_trace;
        std::unordered_map<uint8_t, SensorEntry> m_sensors;

        float targetOmega(std::size_t wheelIndex) const;
        uint16_t sampleLineSensor(const SensorEntry&) const;
        uint16_t sampleDistanceSensor(const SensorEntry&) const;
        std::size_t portToWheelIndex(uint8_t port) const;
        bool wheelInverted(std::size_t wheelIndex) const;

        /// Mutable RNG for BEMF noise (const-correct: reading is logically const
        /// even though the RNG state advances).
        mutable std::mt19937 m_rng{42};
        mutable std::normal_distribution<float> m_noiseDist{0.0f, 1.0f};
    };
}
