#pragma once

#include <raccoon/Transport.h>
#include <raccoon/Channels.h>
#include <raccoon/Options.h>
#include <exlcm/vector3f_t.hpp>
#include <exlcm/scalar_i8_t.hpp>
#include <exlcm/scalar_i32_t.hpp>
#include <exlcm/scalar_f_t.hpp>
#include <string>
#include <unordered_map>
#include <mutex>
#include <optional>
#include <thread>
#include <atomic>
#include <functional>

namespace platform::wombat::core {
    class LcmReader {
    public:
        explicit LcmReader();
        ~LcmReader();

        static LcmReader& instance()
        {
            static LcmReader impl;
            return impl;
        }

        exlcm::scalar_i8_t readServoMode(int port);
        exlcm::scalar_i32_t readServoValue(int port);

        exlcm::vector3f_t readGyro();
        exlcm::vector3f_t readAccel();
        exlcm::vector3f_t readLinearAccel();
        exlcm::vector3f_t readAccelVelocity();
        void resetAccelVelocity();
        exlcm::vector3f_t readMag();
        exlcm::scalar_f_t readHeading();
        exlcm::scalar_i32_t readBemf(int idx);
        int32_t readMotorPosition(int port);
        bool readMotorDone(int port);

        exlcm::scalar_i32_t readAnalog(int port);
        exlcm::scalar_i32_t readDigital(int port);

        exlcm::scalar_f_t readTemp();

        // Wait for IMU orientation data to be received from the coprocessor
        // Returns true if data was received within timeout_ms, false otherwise
        bool waitForImuReady(int timeout_ms = 1000);

        // Deprecated: callback mechanism removed, use getIntegratedVelocity via IIMU instead
        void setLinearAccelCallback(std::function<void(float, float, float)> /*callback*/) {}

    private:
        raccoon::Transport transport_;

        // Background thread for listening
        std::thread listener_thread_;
        std::atomic<bool> running_{false};

        // Caches for all sensors values
        std::mutex cache_mutex_;
        std::unordered_map<int, int8_t> servo_mode_cache_;
        std::unordered_map<int, int32_t> servo_value_cache_;
        std::unordered_map<int, int32_t> bemf_cache_;
        std::unordered_map<int, int32_t> motor_position_cache_;
        std::unordered_map<int, int32_t> motor_done_cache_;
        std::unordered_map<int, int32_t> analog_cache_;
        std::unordered_map<int, int32_t> digital_cache_;

        exlcm::vector3f_t gyro_cache_{};
        exlcm::vector3f_t accel_cache_{};
        exlcm::vector3f_t linear_accel_cache_{};
        exlcm::vector3f_t accel_velocity_cache_{};
        exlcm::vector3f_t accel_velocity_offset_{};
        exlcm::vector3f_t mag_cache_{};
        exlcm::scalar_f_t heading_cache_{};
        exlcm::scalar_f_t temp_cache_{};

        // Track whether real IMU heading data has been received
        std::atomic<bool> imu_heading_received_{false};

        // Background listening function
        void listenLoop();
    };

    enum class MotorDir : uint8_t { Off = 0b00, CCW = 0b01, CW = 0b10, ServoLike = 0b11 };

    enum class ServoMode : uint8_t { FullyDisabled = 0, Disabled = 1, Enabled = 2 };
}
