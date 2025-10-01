#pragma once

#include <lcm/lcm-cpp.hpp>
#include <exlcm/vector3f_t.hpp>
#include <exlcm/scalar_i8_t.hpp>
#include <exlcm/scalar_i32_t.hpp>
#include <exlcm/scalar_f_t.hpp>
#include <string>

namespace platform::wombat::core {
    class LcmReader {
    public:
        explicit LcmReader();
        ~LcmReader() = default;

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
        exlcm::scalar_i32_t readBemf(int idx);

        exlcm::scalar_i32_t readAnalog(int port);
        exlcm::scalar_i32_t readDigital(int port);

        exlcm::scalar_f_t readTemp();

    private:
        lcm::LCM lcm_;

        template <typename T>
        T readOnce(const std::string& channel, int timeout_ms = 500);
    };

    enum class MotorDir : uint8_t { Off = 0b00, CCW = 0b01, CW = 0b10, ServoLike = 0b11 };

    enum class ServoMode : uint8_t { FullyDisabled = 0, Disabled = 1, Enabled = 2 };
}