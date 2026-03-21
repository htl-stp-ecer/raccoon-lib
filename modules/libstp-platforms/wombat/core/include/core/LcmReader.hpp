#pragma once

#include <raccoon/Transport.h>
#include <raccoon/Channels.h>
#include <raccoon/Options.h>
#include <raccoon/vector3f_t.hpp>
#include <raccoon/scalar_i8_t.hpp>
#include <raccoon/scalar_i32_t.hpp>
#include <raccoon/scalar_f_t.hpp>
#include <string>
#include <unordered_map>
#include <mutex>
#include <optional>
#include <thread>
#include <atomic>
#include <functional>

namespace platform::wombat::core {
    /**
     * Cached raccoon transport reader for the wombat bundle.
     *
     * The reader owns a background listener thread that keeps the latest values
     * for sensors and device status so HAL wrappers can expose synchronous APIs.
     */
    class LcmReader {
    public:
        explicit LcmReader();
        ~LcmReader();

        static LcmReader& instance()
        {
            static LcmReader impl;
            return impl;
        }

        raccoon::scalar_i8_t readServoMode(int port);
        raccoon::scalar_f_t readServoValue(int port);

        /// Return the latest cached IMU vectors from raccoon subscriptions.
        raccoon::vector3f_t readGyro();
        raccoon::vector3f_t readAccel();
        raccoon::vector3f_t readLinearAccel();
        raccoon::vector3f_t readAccelVelocity();
        /// Snapshot the current integrated-velocity reading as the new zero point.
        void resetAccelVelocity();
        raccoon::vector3f_t readMag();
        raccoon::scalar_f_t readHeading();
        raccoon::scalar_i32_t readBemf(int idx);
        int32_t readMotorPosition(int port);
        bool readMotorDone(int port);

        raccoon::scalar_i32_t readAnalog(int port);
        raccoon::scalar_i32_t readDigital(int port);

        raccoon::scalar_f_t readTemp();

        /// STM32-computed odometry state.
        struct OdometrySnapshot {
            float pos_x{0.0f};
            float pos_y{0.0f};
            float heading{0.0f};
            float vx{0.0f};
            float vy{0.0f};
            float wz{0.0f};
        };
        OdometrySnapshot readOdometry();

        /// Zero the local odometry cache (call alongside STM32 reset command).
        void resetOdometry();

        /// Wait until heading data has been observed at least once.
        bool waitForImuReady(int timeout_ms = 1000);

        // Deprecated: callback mechanism removed, use getIntegratedVelocity via IIMU instead.
        void setLinearAccelCallback(std::function<void(float, float, float)> /*callback*/) {}

    private:
        raccoon::Transport transport_;

        // Background thread for listening.
        std::thread listener_thread_;
        std::atomic<bool> running_{false};

        // Cached copies of the most recent transport data.
        std::mutex cache_mutex_;
        std::unordered_map<int, int8_t> servo_mode_cache_;
        std::unordered_map<int, float> servo_value_cache_;
        std::unordered_map<int, int32_t> bemf_cache_;
        std::unordered_map<int, int32_t> motor_position_cache_;
        std::unordered_map<int, int32_t> motor_done_cache_;
        std::unordered_map<int, int32_t> analog_cache_;
        std::unordered_map<int, int32_t> digital_cache_;

        raccoon::vector3f_t gyro_cache_{};
        raccoon::vector3f_t accel_cache_{};
        raccoon::vector3f_t linear_accel_cache_{};
        raccoon::vector3f_t accel_velocity_cache_{};
        raccoon::vector3f_t accel_velocity_offset_{};
        raccoon::vector3f_t mag_cache_{};
        raccoon::scalar_f_t heading_cache_{};
        raccoon::scalar_f_t temp_cache_{};

        // STM32 odometry cache
        OdometrySnapshot odom_cache_{};

        // Track whether real IMU heading data has been received.
        std::atomic<bool> imu_heading_received_{false};

        // Background listening function.
        void listenLoop();
    };

    enum class MotorDir : uint8_t { Off = 0b00, CCW = 0b01, CW = 0b10, ServoLike = 0b11 };

    enum class ServoMode : uint8_t { FullyDisabled = 0, Disabled = 1, Enabled = 2 };
}
