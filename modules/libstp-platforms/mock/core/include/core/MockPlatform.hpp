#pragma once
#include <cstdint>
#include <array>
#include <atomic>
#include <random>
#include <chrono>
#include <mutex>

namespace platform::mock::core
{
    enum class MotorDir : uint8_t { Off = 0b00, CCW = 0b01, CW = 0b10, ServoLike = 0b11 };
    enum class ServoMode : uint8_t { FullyDisabled = 0, Disabled = 1, Enabled = 2 };

    /**
     * In-process backend used by the mock driver bundle.
     *
     * HAL driver implementations call this singleton instead of touching real
     * hardware. Tests can also seed state through the helper setters below.
     */
    class MockPlatform
    {
    public:
        static MockPlatform& instance()
        {
            static MockPlatform impl;
            return impl;
        }

        bool init();
        bool update();

        // Motor control
        void setMotor(uint8_t port, MotorDir dir, uint32_t value);
        float getBemf(uint8_t port) const;
        
        // Servo control
        void setServo(uint8_t port, ServoMode mode, uint16_t pos);
        
        // Analog sensors (simulated)
        uint16_t getAnalog(uint8_t port) const;
        
        // Digital inputs (simulated)
        bool getDigital(uint8_t bit) const;
        uint16_t getDigitalRaw() const;
        
        // IMU data (simulated)
        float getGyroX() const;
        float getGyroY() const;
        float getGyroZ() const;
        float getAccelX() const;
        float getAccelY() const;
        float getAccelZ() const;
        float getMagX() const;
        float getMagY() const;
        float getMagZ() const;
        
        uint32_t getLastUpdateUs() const;

        // Test utilities for contributors and higher-level tests.
        void setAnalogValue(uint8_t port, uint16_t value);
        void setDigitalValue(uint8_t bit, bool value);
        void setIMUValues(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

    private:
        MockPlatform();
        ~MockPlatform() = default;

        // Motor state
        struct MotorState {
            MotorDir direction = MotorDir::Off;
            uint32_t speed = 0;
            float position = 0.0f;  // Simulated encoder position
            float bemf = 0.0f;      // Back EMF simulation
        };
        
        // Servo state
        struct ServoState {
            ServoMode mode = ServoMode::Disabled;
            uint16_t position = 1500;  // Default center position
        };

        mutable std::mutex m_mutex;
        std::array<MotorState, 4> m_motors;
        std::array<ServoState, 4> m_servos;
        std::array<uint16_t, 6> m_analog_values;
        uint16_t m_digital_values;
        
        // IMU simulation
        mutable std::mt19937 m_rng;
        mutable std::normal_distribution<float> m_noise_dist;
        
        struct IMUState {
            float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
            float accel_x = 0.0f, accel_y = 0.0f, accel_z = 9.81f;  // Default gravity
            float mag_x = 0.0f, mag_y = 0.0f, mag_z = 1.0f;        // Default magnetic field
        } m_imu;
        
        std::chrono::high_resolution_clock::time_point m_start_time;
        
        void updateMotorSimulation();
        float addNoise(float value) const;
    };

    // Free functions keep mock driver code shaped similarly to wombat driver code.
    inline void setMotor(uint8_t port, MotorDir dir, uint32_t value)
    {
        MockPlatform::instance().setMotor(port, dir, value);
    }

    inline void setServo(uint8_t port, ServoMode mode, uint16_t pos)
    {
        MockPlatform::instance().setServo(port, mode, pos);
    }

    inline uint16_t analog(uint8_t idx)
    {
        return MockPlatform::instance().getAnalog(idx);
    }

    inline bool digital(uint8_t bit)
    {
        return MockPlatform::instance().getDigital(bit);
    }

    inline uint16_t digitalRaw()
    {
        return MockPlatform::instance().getDigitalRaw();
    }

    inline float bemf(uint8_t mot)
    {
        return MockPlatform::instance().getBemf(mot);
    }

    inline float gyroX() { return MockPlatform::instance().getGyroX(); }
    inline float gyroY() { return MockPlatform::instance().getGyroY(); }
    inline float gyroZ() { return MockPlatform::instance().getGyroZ(); }
    
    inline float accelX() { return MockPlatform::instance().getAccelX(); }
    inline float accelY() { return MockPlatform::instance().getAccelY(); }
    inline float accelZ() { return MockPlatform::instance().getAccelZ(); }
    
    inline float magX() { return MockPlatform::instance().getMagX(); }
    inline float magY() { return MockPlatform::instance().getMagY(); }
    inline float magZ() { return MockPlatform::instance().getMagZ(); }
    
    inline uint32_t lastUpdateUs()
    {
        return MockPlatform::instance().getLastUpdateUs();
    }
}
