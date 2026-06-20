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
        m_wheelAngleRad.fill(0.0f);
        m_yawRateRadS = 0.0f;
        m_motorTargetOmega.fill(0.0f);
    }

    void SimWorld::setMap(WorldMap map) { m_map = std::move(map); }

    void SimWorld::setPose(const Pose2D& pose)
    {
        m_pose = pose;
        // The dead-reckoned odometry frame starts coincident with the physical
        // pose; it then integrates body velocities independently (see below).
        m_odoPose = pose;
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

    int SimWorld::motorPositionTicks(uint8_t port) const
    {
        const auto wheelIdx = portToWheelIndex(port);
        if (wheelIdx >= activeWheelCount()) return 0;
        const float ticksToRad = std::max(1e-9f, m_motors.ticksToRad);
        return static_cast<int>(std::lround(m_wheelAngleRad[wheelIdx] / ticksToRad));
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

        // Wheel dynamics: commanded ω acts like the no-load speed target,
        // while viscous + Coulomb losses reduce the net acceleration.
        // This keeps drag in the plant itself instead of as a post-hoc clamp,
        // which makes low-speed and high-drag behavior more characterizable.
        const float tau = std::max(1e-4f, m_motors.motorTimeConstantSec);
        const float alpha = 1.0f - std::exp(-dt / tau);
        const std::size_t nWheels = activeWheelCount();
        for (std::size_t i = 0; i < nWheels; ++i)
        {
            const float target = targetOmega(i);
            const float omega = m_wheelOmega[i];
            const float driveAccel = (target - omega) * (alpha / std::max(dt, 1e-6f));
            const float viscousAccel = m_motors.viscousDragCoeff * omega;

            float frictionSign = 0.0f;
            if (std::abs(omega) > 1e-4f)
            {
                frictionSign = std::copysign(1.0f, omega);
            }
            else if (std::abs(target) > 1e-4f)
            {
                frictionSign = std::copysign(1.0f, target);
            }

            const float coulombAccel = m_motors.coulombFrictionRadSS * frictionSign;
            if (std::abs(omega) <= 1e-4f &&
                std::abs(driveAccel) <= std::abs(coulombAccel) + 1e-6f)
            {
                m_wheelOmega[i] = 0.0f;
            }
            else
            {
                float nextOmega = omega + (driveAccel - viscousAccel - coulombAccel) * dt;
                if (std::abs(target) <= 1e-4f && omega * nextOmega < 0.0f)
                {
                    nextOmega = 0.0f;
                }
                m_wheelOmega[i] = nextOmega;
            }

            m_wheelAngleRad[i] += m_wheelOmega[i] * dt;
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

        // Dead-reckon the controller-facing odometry from the body velocities,
        // exactly the way real wheel/IMU odometry integrates — and crucially
        // INDEPENDENT of the line-218 lateral negation applied to the physical
        // pose below. `vyMps` is the production +vy command (= robot RIGHT, see
        // MecanumKinematics), so integrating it with the standard body→world
        // rotation makes the odometry's "right-positive" lateral projection
        // (LinearMotion::projectBodyFrame) grow for a +vy command, while the
        // forward projection grows for a +vx command and the heading grows for
        // a +wz command — all consistently at any heading. This is what lets a
        // fixed-distance strafe close its lateral loop instead of running away,
        // without disturbing the forward / turn loops (which already worked).
        {
            const float co = std::cos(m_odoPose.theta);
            const float so = std::sin(m_odoPose.theta);
            m_odoPose.x += (vxMps * co - vyMps * so) * kMetersToCm * dt;
            m_odoPose.y += (vxMps * so + vyMps * co) * kMetersToCm * dt;
            m_odoPose.theta = normalizeAngle(m_odoPose.theta + yawRate * dt);
        }

        const float dxLocalCm = vxMps * kMetersToCm * dt;
        // ChassisVelocity / MecanumKinematics convention is +y = RIGHT (see
        // MecanumKinematics::applyCommand), but applyLocalDelta treats +dyLocal
        // as the robot's LEFT (90 deg CCW). Negate so a +vy command moves the
        // sim robot RIGHT, matching the production IK and the real robot.
        const float dyLocalCm = -vyMps * kMetersToCm * dt;
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
        // Real reflectance line sensors read LOW on white and HIGH on black —
        // and IRSensor::probabilityOfBlack maps raw→black on exactly that
        // convention (white = low threshold, black = high threshold; cf.
        // cube-bot calibration white_tresh ~200-530, black_tresh ~3200+). Emit
        // on the same 0..1023 analog scale the HAL exposes: 1023 (black) / 0
        // (white). Noise/thresholds are the consumer's concern, not the sim's.
        return isSensorOnBlackLine(e.mount) ? 1023 : 0;
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
