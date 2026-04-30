#pragma once

#include <raccoon/Transport.h>
#include <raccoon/Channels.h>
#include <raccoon/vector3f_t.hpp>
#include <raccoon/scalar_i8_t.hpp>
#include <raccoon/scalar_i32_t.hpp>
#include <raccoon/scalar_f_t.hpp>
#include <raccoon/orientation_matrix_t.hpp>
#include <raccoon/kinematics_config_t.hpp>

#include <array>

#include <string>

namespace platform::wombat::core
{
    /**
     * Command publisher for the wombat bundle.
     *
     * HAL motor and servo wrappers delegate writes here so transport details stay
     * out of the public HAL classes.
     */
    class LcmDataWriter
    {
    public:
        static LcmDataWriter& instance()
        {
            static LcmDataWriter writer;
            return writer;
        }

        /// Publish motor mode command (0=OFF, 1=PASSIVE_BRAKE).
        void setMotorMode(uint8_t port, int mode);
        /// Publish open-loop motor power for one port.
        void setMotor(uint8_t port, int valueData);
        /// Publish a firmware-side velocity target in BEMF units.
        void setMotorVelocity(uint8_t port, int32_t velocity);
        /// Publish an absolute position move command.
        void setMotorPosition(uint8_t port, int32_t velocity, int32_t goalPosition);
        /// Publish a relative position move command.
        void setMotorRelative(uint8_t port, int32_t velocity, int32_t deltaPosition);
        /// Publish a servo position command in degrees (0-180).
        void setServo(uint8_t port, float degrees);
        /// Publish a servo mode update.
        void setServoMode(uint8_t port, uint8_t mode);
        /// Publish a smooth servo command; interpolation runs in the C++ reader at 200 Hz.
        void setSmoothServo(uint8_t port, float targetDeg, float speedDegPerSec, int easing);
        /// Publish per-port PID gains.
        void setMotorPid(uint8_t port, float kp, float ki, float kd);
        /// Reset firmware motor position for one port.
        void resetMotorPosition(uint8_t port);

        /// Toggle the STM32-wide shutdown flag for the safest available stop path.
        void setShutdown(bool enabled);

        /// Send a heartbeat to stm32-data-reader. Must be called every ~100 ms
        /// while missions are running or the hardware watchdog fires.
        void sendHeartbeat();

        /// Send kinematics config to STM32 for on-board odometry.
        void sendKinematicsConfig(const std::array<std::array<float, 4>, 3>& inv_matrix,
                                  const std::array<float, 4>& ticks_to_rad,
                                  const std::array<std::array<float, 3>, 4>& fwd_matrix);

        /// Request STM32 to reset its integrated odometry pose.
        void resetOdometry();
    private:
        explicit LcmDataWriter();
        ~LcmDataWriter() = default;

        raccoon::Transport transport_;
    };
}
