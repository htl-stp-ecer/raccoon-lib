#include "libstp/sim/SimWorld.hpp"

#include "libstp/sim/Collision.hpp"

#include <algorithm>
#include <cmath>

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
        m_wheelOmega = {0.0f, 0.0f};
        m_yawRateRadS = 0.0f;
        m_motorCommand.fill(0);
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
        if (port >= m_motorCommand.size()) return;
        m_motorCommand[port] = std::clamp(signedPercent, -100, 100);
    }

    float SimWorld::targetOmega(std::size_t wheelIndex) const
    {
        const uint8_t port = wheelIndex == kLeft ? m_motors.leftPort : m_motors.rightPort;
        const bool inverted = wheelIndex == kLeft ? m_motors.leftInverted : m_motors.rightInverted;
        const int cmd = m_motorCommand[port];
        const float normalized = static_cast<float>(cmd) / 100.0f;
        const float signedOmega = normalized * m_motors.maxWheelVelocityRadS;
        return inverted ? -signedOmega : signedOmega;
    }

    void SimWorld::tick(float dt)
    {
        if (dt <= 0.0f) return;

        // First-order motor response. Exact discrete form — stable for any dt.
        const float tau = std::max(1e-4f, m_motors.motorTimeConstantSec);
        const float alpha = 1.0f - std::exp(-dt / tau);
        for (std::size_t i = 0; i < 2; ++i)
        {
            const float target = targetOmega(i);
            m_wheelOmega[i] += (target - m_wheelOmega[i]) * alpha;
        }

        // Differential-drive forward kinematics (from the classic
        // omega_L/omega_R → (v, ω_z) equations used throughout libstp-drive).
        const float r = m_robot.wheelRadiusM;
        const float trackWidth = std::max(1e-4f, m_robot.trackWidthM);
        const float vMetersPerSec = 0.5f * r * (m_wheelOmega[kLeft] + m_wheelOmega[kRight]);
        const float yawRate = (r / trackWidth) * (m_wheelOmega[kRight] - m_wheelOmega[kLeft]);
        m_yawRateRadS = yawRate;

        const float dxLocalCm = vMetersPerSec * kMetersToCm * dt;
        const float dThetaRad = yawRate * dt;

        const Pose2D target = applyLocalDelta(m_pose, dxLocalCm, 0.0f, dThetaRad);

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
}
