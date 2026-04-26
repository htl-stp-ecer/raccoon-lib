#include "libstp/sim/SimWorld.hpp"

#include "libstp/sim/Collision.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace libstp::sim
{
    namespace
    {
        constexpr float kMetersToCm = 100.0f;
    }

    void SimWorld::configure(const RobotConfig& robot, const SimMotorMap& motors)
    {
        m_robot = robot;
        m_motors = motors;
        m_wheelOmega.fill(0.0f);
        m_yawRateRadS = 0.0f;
        m_motorTargetOmega.fill(0.0f);
    }

    void SimWorld::setMap(WorldMap map) { m_map = std::move(map); }

    void SimWorld::setPose(const Pose2D& pose)
    {
        m_pose = pose;
        m_trace.clear();
        m_trace.push_back(m_pose);
    }

    void SimWorld::setMotorCommand(uint8_t port, int signedPercent)
    {
        if (port >= m_motorTargetOmega.size()) return;
        const int clamped = std::clamp(signedPercent, -100, 100);
        m_motorTargetOmega[port] =
            (static_cast<float>(clamped) / 100.0f) * m_motors.maxWheelVelocityRadS;
    }

    void SimWorld::setMotorVelocityCommand(uint8_t port, int bemfUnits)
    {
        if (port >= m_motorTargetOmega.size()) return;
        // The HAL's BEMF integer is in "encoder ticks per BEMF sample period"
        // (not per second). MotorAdapter encodes via:
        //     bemf = ω / (ticks_to_rad · bemfSampleRateHz)
        // Invert it here to recover the commanded wheel ω in rad/s.
        m_motorTargetOmega[port] =
            static_cast<float>(bemfUnits) * m_motors.ticksToRad * m_motors.bemfSampleRateHz;
    }

    int SimWorld::motorBemf(uint8_t port) const
    {
        const auto wheelIdx = portToWheelIndex(port);
        if (wheelIdx >= activeWheelCount()) return 0;
        const float omega = m_wheelOmega[wheelIdx];
        const float divisor = std::max(1e-9f, m_motors.ticksToRad * m_motors.bemfSampleRateHz);
        float bemf = omega / divisor;
        if (m_motors.bemfNoiseStddev > 0.0f)
        {
            bemf += m_motors.bemfNoiseStddev * m_noiseDist(m_rng);
        }
        return static_cast<int>(std::lround(bemf));
    }

    std::size_t SimWorld::activeWheelCount() const noexcept
    {
        return m_motors.kind == DrivetrainKind::Mecanum ? 4 : 2;
    }

    std::size_t SimWorld::portToWheelIndex(uint8_t port) const
    {
        if (m_motors.kind == DrivetrainKind::Mecanum)
        {
            if (port == m_motors.flPort) return kFL;
            if (port == m_motors.frPort) return kFR;
            if (port == m_motors.blPort) return kBL;
            if (port == m_motors.brPort) return kBR;
            return std::numeric_limits<std::size_t>::max();
        }
        if (port == m_motors.leftPort) return kLeft;
        if (port == m_motors.rightPort) return kRight;
        return std::numeric_limits<std::size_t>::max();
    }

    uint8_t SimWorld::wheelPort(std::size_t wheelIndex) const
    {
        if (m_motors.kind == DrivetrainKind::Mecanum)
        {
            switch (wheelIndex)
            {
                case kFL: return m_motors.flPort;
                case kFR: return m_motors.frPort;
                case kBL: return m_motors.blPort;
                case kBR: return m_motors.brPort;
                default:  return 0;
            }
        }
        return wheelIndex == kLeft ? m_motors.leftPort : m_motors.rightPort;
    }

    bool SimWorld::wheelInverted(std::size_t wheelIndex) const
    {
        if (m_motors.kind == DrivetrainKind::Mecanum)
        {
            switch (wheelIndex)
            {
                case kFL: return m_motors.flInverted;
                case kFR: return m_motors.frInverted;
                case kBL: return m_motors.blInverted;
                case kBR: return m_motors.brInverted;
                default:  return false;
            }
        }
        return wheelIndex == kLeft ? m_motors.leftInverted : m_motors.rightInverted;
    }

    float SimWorld::targetOmega(std::size_t wheelIndex) const
    {
        const uint8_t port = wheelPort(wheelIndex);
        const float cmd = m_motorTargetOmega[port];
        return wheelInverted(wheelIndex) ? -cmd : cmd;
    }

    void SimWorld::tick(float dt)
    {
        if (dt <= 0.0f) return;

        // First-order motor response. Exact discrete form — stable for any dt.
        const float tau = std::max(1e-4f, m_motors.motorTimeConstantSec);
        const float alpha = 1.0f - std::exp(-dt / tau);
        const std::size_t nWheels = activeWheelCount();
        for (std::size_t i = 0; i < nWheels; ++i)
        {
            const float target = targetOmega(i);
            m_wheelOmega[i] += (target - m_wheelOmega[i]) * alpha;

            // Viscous drag: velocity-proportional resistance (bearings, back-EMF).
            if (m_motors.viscousDragCoeff > 0.0f)
            {
                m_wheelOmega[i] *= (1.0f - m_motors.viscousDragCoeff * dt);
            }

            // Coulomb friction: constant deceleration opposing motion.
            if (m_motors.coulombFrictionRadSS > 0.0f &&
                std::abs(m_wheelOmega[i]) > 1e-3f)
            {
                const float frictionDelta = m_motors.coulombFrictionRadSS * dt;
                if (std::abs(m_wheelOmega[i]) <= frictionDelta)
                {
                    m_wheelOmega[i] = 0.0f;   // clamp — don't reverse through friction
                }
                else
                {
                    m_wheelOmega[i] -= std::copysign(frictionDelta, m_wheelOmega[i]);
                }
            }
        }

        // Forward kinematics → chassis velocity in body frame. The mecanum
        // mixing matrix mirrors MecanumKinematics::estimateState so the sim
        // and the production kinematics agree on sign conventions.
        const float r = m_robot.wheelRadiusM;
        float vxMps = 0.0f;
        float vyMps = 0.0f;
        float yawRate = 0.0f;

        if (m_motors.kind == DrivetrainKind::Mecanum)
        {
            // Standard 4-wheel mecanum FK (matches MecanumKinematics::estimateState).
            const float wheelbase = std::max(1e-4f, m_robot.wheelbaseM);
            const float trackWidth = std::max(1e-4f, m_robot.trackWidthM);
            const float L = 0.5f * (wheelbase + trackWidth);
            const float fl = m_wheelOmega[kFL];
            const float fr = m_wheelOmega[kFR];
            const float bl = m_wheelOmega[kBL];
            const float br = m_wheelOmega[kBR];
            vxMps = (fl + fr + bl + br) * r * 0.25f;
            vyMps = (fl - fr - bl + br) * r * 0.25f;
            yawRate = (-fl + fr - bl + br) * r / (4.0f * L);
        }
        else
        {
            const float trackWidth = std::max(1e-4f, m_robot.trackWidthM);
            vxMps = 0.5f * r * (m_wheelOmega[kLeft] + m_wheelOmega[kRight]);
            yawRate = (r / trackWidth) * (m_wheelOmega[kRight] - m_wheelOmega[kLeft]);
        }
        m_yawRateRadS = yawRate;

        const float dxLocalCm = vxMps * kMetersToCm * dt;
        const float dyLocalCm = vyMps * kMetersToCm * dt;
        const float dThetaRad = yawRate * dt;

        const Pose2D target = applyLocalDelta(m_pose, dxLocalCm, dyLocalCm, dThetaRad);

        const auto walls = collision::buildCollisionWalls(m_map);
        if (walls.empty())
        {
            m_pose = target;
        }
        else
        {
            const auto path = collision::simulateSegment(m_pose, target, m_robot, walls);
            if (!path.empty())
            {
                m_pose = path.back();
            }
        }
        m_trace.push_back(m_pose);
    }

    Vec2 SimWorld::sensorWorldPosition(const SensorMount& mount) const
    {
        const float forwardFromRc = mount.forwardCm - m_robot.rotationCenterForwardCm;
        const float strafeFromRc = mount.strafeCm - m_robot.rotationCenterStrafeCm;
        const Pose2D sensorPose = applyLocalDelta(m_pose, forwardFromRc, strafeFromRc, 0.0f);
        return {sensorPose.x, sensorPose.y};
    }

    bool SimWorld::isSensorOnBlackLine(const SensorMount& mount) const
    {
        const Vec2 p = sensorWorldPosition(mount);
        return m_map.isOnBlackLine(p.x, p.y);
    }

    void SimWorld::attachLineSensor(uint8_t analogPort, float forwardCm, float strafeCm, std::string name)
    {
        SensorEntry e{};
        e.kind = SensorKind::Line;
        e.mount.name = std::move(name);
        e.mount.forwardCm = forwardCm;
        e.mount.strafeCm = strafeCm;
        e.mount.analogPort = analogPort;
        m_sensors[analogPort] = std::move(e);
    }

    void SimWorld::attachDistanceSensor(uint8_t analogPort, float forwardCm, float strafeCm,
                                        float mountAngleRad, float maxRangeCm, std::string name)
    {
        SensorEntry e{};
        e.kind = SensorKind::Distance;
        e.mount.name = std::move(name);
        e.mount.forwardCm = forwardCm;
        e.mount.strafeCm = strafeCm;
        e.mount.analogPort = analogPort;
        e.mountAngleRad = mountAngleRad;
        e.maxRangeCm = maxRangeCm;
        m_sensors[analogPort] = std::move(e);
    }

    void SimWorld::detachSensor(uint8_t analogPort)
    {
        m_sensors.erase(analogPort);
    }

    std::optional<uint16_t> SimWorld::readAnalog(uint8_t analogPort) const
    {
        auto it = m_sensors.find(analogPort);
        if (it == m_sensors.end()) return std::nullopt;
        switch (it->second.kind)
        {
            case SensorKind::Line:     return sampleLineSensor(it->second);
            case SensorKind::Distance: return sampleDistanceSensor(it->second);
        }
        return std::nullopt;
    }

    uint16_t SimWorld::sampleLineSensor(const SensorEntry& e) const
    {
        // Real line sensors read HIGH on white, LOW on black — we mirror that
        // with 1023 (white) / 0 (black). Noise/thresholds are the consumer's
        // concern, not the sim's.
        return isSensorOnBlackLine(e.mount) ? 0 : 1023;
    }

    uint16_t SimWorld::sampleDistanceSensor(const SensorEntry& e) const
    {
        // World-frame sensor origin.
        const Vec2 origin = sensorWorldPosition(e.mount);
        // World-frame aim = robot heading + mount angle.
        const float worldAngle = m_pose.theta + e.mountAngleRad;

        const auto walls = collision::buildCollisionWalls(m_map);
        const float distanceCm = collision::raycastDistanceCm(
            origin.x, origin.y, worldAngle, e.maxRangeCm, walls);

        // KIPR ET empirical curve (from KIPR Simulator's EtSensor.ts, clamped
        // to the 0..1023 analog range our HAL exposes). The curve was fit to
        // real-world ET sensor readings.
        float value;
        if (distanceCm >= e.maxRangeCm)
        {
            value = 275.0f;   // "no target" floor
        }
        else if (distanceCm >= 45.0f)
        {
            value = 188.0f;
        }
        else if (distanceCm <= 3.0f)
        {
            value = distanceCm * (727.5f / 3.0f);  // 0 → ~727
        }
        else if (distanceCm <= 10.0f)
        {
            value = 727.5f;
        }
        else
        {
            // 10 – 45 cm useful range — decays with distance.
            const float x = -1.2222f + 0.0444f * distanceCm;
            value = 307.75f - 279.3f * x + 180.9f * (x * x);
        }
        if (value < 0.0f) value = 0.0f;
        if (value > 1023.0f) value = 1023.0f;
        return static_cast<uint16_t>(value);
    }
}
