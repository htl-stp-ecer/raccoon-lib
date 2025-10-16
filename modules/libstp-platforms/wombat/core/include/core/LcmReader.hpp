#pragma once

#include <lcm/lcm-cpp.hpp>
#include <exlcm/vector3f_t.hpp>
#include <exlcm/scalar_i8_t.hpp>
#include <exlcm/scalar_i32_t.hpp>
#include <exlcm/scalar_f_t.hpp>
#include <exlcm/quaternionf_t.hpp>
#include <string>
#include <unordered_map>
#include <mutex>
#include <optional>
#include <thread>
#include <atomic>

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

        exlcm::scalar_i32_t readMotorValue(int port);
        exlcm::scalar_i8_t readMotorDir(int port);

        exlcm::scalar_i8_t readServoMode(int port);
        exlcm::scalar_i32_t readServoValue(int port);

        exlcm::vector3f_t readGyro();
        exlcm::vector3f_t readAccel();
        exlcm::vector3f_t readMag();
        exlcm::quaternionf_t readOrientation();
        exlcm::scalar_i32_t readBemf(int idx);

        exlcm::scalar_i32_t readAnalog(int port);
        exlcm::scalar_i32_t readDigital(int port);

        exlcm::scalar_f_t readTemp();

    private:
        lcm::LCM lcm_;

        // Background thread for listening
        std::thread listener_thread_;
        std::atomic<bool> running_{false};

        // Caches for all sensor values
        std::mutex cache_mutex_;
        std::unordered_map<int, int32_t> motor_value_cache_;
        std::unordered_map<int, int8_t> motor_dir_cache_;
        std::unordered_map<int, int8_t> servo_mode_cache_;
        std::unordered_map<int, int32_t> servo_value_cache_;
        std::unordered_map<int, int32_t> bemf_cache_;
        std::unordered_map<int, int32_t> analog_cache_;
        std::unordered_map<int, int32_t> digital_cache_;

        exlcm::vector3f_t gyro_cache_{};
        exlcm::vector3f_t accel_cache_{};
        exlcm::vector3f_t mag_cache_{};
        exlcm::quaternionf_t orientation_cache_{};
        exlcm::scalar_f_t temp_cache_{};

        // Background listening function
        void listenLoop();

        // Message handlers
        void handleMotorValue(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg);
        void handleMotorDir(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i8_t* msg);
        void handleServoMode(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i8_t* msg);
        void handleServoValue(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg);
        void handleGyro(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::vector3f_t* msg);
        void handleAccel(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::vector3f_t* msg);
        void handleMag(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::vector3f_t* msg);
        void handleOrientation(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::quaternionf_t* msg);
        void handleBemf(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg);
        void handleAnalog(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg);
        void handleDigital(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg);
        void handleTemp(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_f_t* msg);
    };

    enum class MotorDir : uint8_t { Off = 0b00, CCW = 0b01, CW = 0b10, ServoLike = 0b11 };

    enum class ServoMode : uint8_t { FullyDisabled = 0, Disabled = 1, Enabled = 2 };
}
