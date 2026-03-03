#pragma once

#include <raccoon/Transport.h>
#include <raccoon/Channels.h>
#include <raccoon/vector3f_t.hpp>
#include <raccoon/scalar_i8_t.hpp>
#include <raccoon/scalar_i32_t.hpp>
#include <raccoon/scalar_f_t.hpp>
#include <raccoon/orientation_matrix_t.hpp>

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

        /// Publish motor stop latch command. Non-zero stops; 0 clears the latch.
        void setMotorStop(uint8_t port, int value);
        /// Publish open-loop motor power for one port.
        void setMotor(uint8_t port, int valueData);
        /// Publish a firmware-side velocity target in BEMF units.
        void setMotorVelocity(uint8_t port, int32_t velocity);
        /// Publish an absolute position move command.
        void setMotorPosition(uint8_t port, int32_t velocity, int32_t goalPosition);
        /// Publish a relative position move command.
        void setMotorRelative(uint8_t port, int32_t velocity, int32_t deltaPosition);
        /// Publish a servo position command.
        void setServo(uint8_t port, int valueData);
        /// Publish a servo mode update.
        void setServoMode(uint8_t port, uint8_t mode);
        /// Publish per-port PID gains.
        void setMotorPid(uint8_t port, float kp, float ki, float kd);
        /// Reset firmware motor position for one port.
        void resetMotorPosition(uint8_t port);

        /// Toggle the STM32-wide shutdown flag for the safest available stop path.
        void setShutdown(bool enabled);
    private:
        explicit LcmDataWriter();
        ~LcmDataWriter() = default;

        raccoon::Transport transport_;
    };
}
