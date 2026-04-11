#include "core/MockPlatform.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace platform::mock::core
{
    namespace
    {
        double defaultClockSeconds()
        {
            using clock = std::chrono::steady_clock;
            static const auto origin = clock::now();
            return std::chrono::duration<double>(clock::now() - origin).count();
        }
    }

    MockPlatform::MockPlatform()
        : m_rng(std::chrono::steady_clock::now().time_since_epoch().count())
        , m_noise_dist(0.0f, 0.01f)  // Small amount of noise
        , m_start_time(std::chrono::high_resolution_clock::now())
        , m_clock(&defaultClockSeconds)
    {
        // Initialize with realistic default values
        std::fill(m_analog_values.begin(), m_analog_values.end(), 512);  // Mid-range ADC value
        m_digital_values = 0x0000;  // All digital inputs false initially
    }

    bool MockPlatform::init()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        // Mock initialization always succeeds
        return true;
    }

    bool MockPlatform::update()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        updateMotorSimulation();
        return true;
    }

    void MockPlatform::setMotor(uint8_t port, MotorDir dir, uint32_t value)
    {
        if (port >= m_motors.size()) return;

        std::lock_guard<std::mutex> lock(m_mutex);
        m_motors[port].direction = dir;
        m_motors[port].speed = value;

        if (m_sim)
        {
            m_sim->setMotorCommand(port, motorCommandToSignedPercent(dir, value));
        }
    }

    void MockPlatform::setMotorVelocity(uint8_t port, int signedBemfUnits)
    {
        if (port >= m_motors.size()) return;

        std::lock_guard<std::mutex> lock(m_mutex);
        m_motors[port].direction =
            signedBemfUnits > 0 ? MotorDir::CW :
            signedBemfUnits < 0 ? MotorDir::CCW : MotorDir::Off;
        m_motors[port].speed = static_cast<uint32_t>(std::abs(signedBemfUnits));

        if (m_sim)
        {
            m_sim->setMotorVelocityCommand(port, signedBemfUnits);
        }
    }

    int MockPlatform::motorCommandToSignedPercent(MotorDir dir, uint32_t value) const
    {
        // Motor HAL encodes percent as duty = |percent| * 4, with dir giving
        // the sign (CW = positive, CCW = negative). Recover the signed percent.
        const int magnitude = static_cast<int>(value / 4u);
        if (dir == MotorDir::CW) return magnitude;
        if (dir == MotorDir::CCW) return -magnitude;
        return 0;
    }

    float MockPlatform::getBemf(uint8_t port) const
    {
        if (port >= m_motors.size()) return 0.0f;

        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_sim)
        {
            // Live BEMF derived from the sim's actual wheel angular velocity.
            // Lets the chassis velocity controller close the loop against
            // the same physics that's integrating the pose.
            return static_cast<float>(m_sim->motorBemf(port));
        }
        return m_motors[port].bemf;
    }

    void MockPlatform::setServo(uint8_t port, ServoMode mode, uint16_t pos)
    {
        if (port >= m_servos.size()) return;
        
        std::lock_guard<std::mutex> lock(m_mutex);
        m_servos[port].mode = mode;
        m_servos[port].position = pos;
    }

    uint16_t MockPlatform::getAnalog(uint8_t port) const
    {
        if (port >= m_analog_values.size()) return 0;

        std::lock_guard<std::mutex> lock(m_mutex);
        const_cast<MockPlatform*>(this)->autoTickLocked();
        if (m_sim)
        {
            if (auto sampled = m_sim->readAnalog(port))
            {
                return *sampled;
            }
        }
        // Fall through to the stock static value when no sim sensor is bound.
        float base_value = static_cast<float>(m_analog_values[port]);
        float noisy_value = base_value + addNoise(base_value) * 10.0f;  // Small variation
        return static_cast<uint16_t>(std::clamp(noisy_value, 0.0f, 1023.0f));
    }

    bool MockPlatform::getDigital(uint8_t bit) const
    {
        if (bit >= 16) return false;
        
        std::lock_guard<std::mutex> lock(m_mutex);
        return (m_digital_values & (1u << bit)) != 0;
    }

    uint16_t MockPlatform::getDigitalRaw() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_digital_values;
    }

    // IMU getters with realistic simulation
    float MockPlatform::getGyroX() const
    {
        return addNoise(m_imu.gyro_x);
    }

    float MockPlatform::getGyroY() const
    {
        return addNoise(m_imu.gyro_y);
    }

    float MockPlatform::getGyroZ() const
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            const_cast<MockPlatform*>(this)->autoTickLocked();
        }
        if (m_sim)
        {
            return addNoise(m_sim->yawRateRadS());
        }
        return addNoise(m_imu.gyro_z);
    }

    float MockPlatform::getAccelX() const
    {
        return addNoise(m_imu.accel_x);
    }

    float MockPlatform::getAccelY() const
    {
        return addNoise(m_imu.accel_y);
    }

    float MockPlatform::getAccelZ() const
    {
        return addNoise(m_imu.accel_z);
    }

    float MockPlatform::getMagX() const
    {
        return addNoise(m_imu.mag_x);
    }

    float MockPlatform::getMagY() const
    {
        return addNoise(m_imu.mag_y);
    }

    float MockPlatform::getMagZ() const
    {
        return addNoise(m_imu.mag_z);
    }

    uint32_t MockPlatform::getLastUpdateUs() const
    {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - m_start_time);
        return static_cast<uint32_t>(duration.count());
    }

    // Test utilities
    void MockPlatform::setAnalogValue(uint8_t port, uint16_t value)
    {
        if (port >= m_analog_values.size()) return;
        
        std::lock_guard<std::mutex> lock(m_mutex);
        m_analog_values[port] = std::clamp(value, static_cast<uint16_t>(0), static_cast<uint16_t>(1023));
    }

    void MockPlatform::setDigitalValue(uint8_t bit, bool value)
    {
        if (bit >= 16) return;
        
        std::lock_guard<std::mutex> lock(m_mutex);
        if (value) {
            m_digital_values |= (1u << bit);
        } else {
            m_digital_values &= ~(1u << bit);
        }
    }

    void MockPlatform::setIMUValues(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_imu.gyro_x = gx;
        m_imu.gyro_y = gy;
        m_imu.gyro_z = gz;
        m_imu.accel_x = ax;
        m_imu.accel_y = ay;
        m_imu.accel_z = az;
        m_imu.mag_x = mx;
        m_imu.mag_y = my;
        m_imu.mag_z = mz;
    }

    // Private methods
    void MockPlatform::updateMotorSimulation()
    {
        for (auto& motor : m_motors) {
            if (motor.direction == MotorDir::Off) {
                motor.bemf = 0.0f;
                continue;
            }
            
            // Simulate motor behavior
            float speed_factor = static_cast<float>(motor.speed) / 400.0f;  // 0-400 range to 0-1
            float direction_factor = (motor.direction == MotorDir::CW) ? 1.0f : -1.0f;
            
            // Update position (encoder simulation)
            motor.position += speed_factor * direction_factor * 0.1f;  // Arbitrary scaling
            
            // Simulate BEMF (back electromotive force) - proportional to speed with some noise
            motor.bemf = speed_factor * direction_factor * 12.0f + addNoise(0.0f) * 0.5f;  // ~12V max with noise
        }
    }

    float MockPlatform::addNoise(float value) const
    {
        return value + m_noise_dist(m_rng);
    }

    // ─── Simulation integration ───

    void MockPlatform::configureSim(const libstp::sim::RobotConfig& robot,
                                    const libstp::sim::SimMotorMap& motors,
                                    libstp::sim::WorldMap map,
                                    const libstp::sim::Pose2D& startPose)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_sim = std::make_unique<libstp::sim::SimWorld>();
        m_sim->configure(robot, motors);
        m_sim->setMap(std::move(map));
        m_sim->setPose(startPose);
        // Default origin = the start pose, so a fresh sim starts at relative
        // (0, 0, 0). Subsequent odometry resets recapture the live pose.
        m_simOrigin = startPose;
    }

    void MockPlatform::detachSim()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_sim.reset();
    }

    bool MockPlatform::hasSim() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_sim != nullptr;
    }

    libstp::sim::SimWorld* MockPlatform::sim() { return m_sim.get(); }

    libstp::sim::Pose2D MockPlatform::simPose() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_sim) return {};
        return m_sim->pose();
    }

    libstp::sim::Pose2D MockPlatform::simRelativePose() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_sim) return {};
        const auto p = m_sim->pose();
        const float dx = p.x - m_simOrigin.x;
        const float dy = p.y - m_simOrigin.y;
        const float c = std::cos(m_simOrigin.theta);
        const float s = std::sin(m_simOrigin.theta);
        libstp::sim::Pose2D rel{};
        rel.x = dx * c + dy * s;
        rel.y = -dx * s + dy * c;
        rel.theta = libstp::sim::normalizeAngle(p.theta - m_simOrigin.theta);
        return rel;
    }

    void MockPlatform::resetSimOrigin()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_sim) return;
        m_simOrigin = m_sim->pose();
    }

    float MockPlatform::simYawRate() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_sim) return 0.0f;
        return m_sim->yawRateRadS();
    }

    void MockPlatform::tickSim(float dtSeconds)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_sim) return;
        m_sim->tick(dtSeconds);
    }

    void MockPlatform::setAutoTickEnabled(bool enabled)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_autoTick = enabled;
        m_lastTickTime = m_clock ? m_clock() : 0.0;
    }

    bool MockPlatform::isAutoTickEnabled() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_autoTick;
    }

    void MockPlatform::setClockSource(ClockFn clock)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_clock = std::move(clock);
        if (!m_clock) m_clock = &defaultClockSeconds;
        m_lastTickTime = m_clock();
    }

    void MockPlatform::setAutoTickMaxStep(float maxDtSeconds)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (maxDtSeconds > 0.0f) m_autoTickMaxStep = maxDtSeconds;
    }

    void MockPlatform::autoTickIfEnabled()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        autoTickLocked();
    }

    void MockPlatform::autoTickLocked()
    {
        if (!m_autoTick || !m_sim || !m_clock) return;
        const double now = m_clock();
        double dt = now - m_lastTickTime;
        m_lastTickTime = now;
        if (dt <= 0.0) return;
        if (dt > static_cast<double>(m_autoTickMaxStep)) dt = m_autoTickMaxStep;
        m_sim->tick(static_cast<float>(dt));
    }
}