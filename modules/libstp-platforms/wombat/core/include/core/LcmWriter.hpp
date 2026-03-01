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
    class LcmDataWriter
    {
    public:
        static LcmDataWriter& instance()
        {
            static LcmDataWriter writer;
            return writer;
        }

        // Publish motor stop latch command. Non-zero -> stop/latch; 0 -> wake/enable.
        void setMotorStop(uint8_t port, int value);
        void setMotor(uint8_t port, int valueData);
        // PID velocity control - firmware handles PID. velocity in BEMF units.
        void setMotorVelocity(uint8_t port, int32_t velocity);
        // PID position control (absolute) - velocity=BEMF speed, goalPosition=BEMF ticks
        void setMotorPosition(uint8_t port, int32_t velocity, int32_t goalPosition);
        // PID position control (relative) - velocity=BEMF speed, delta=BEMF ticks from current
        void setMotorRelative(uint8_t port, int32_t velocity, int32_t deltaPosition);
        void setServo(uint8_t port, int valueData);
        void resetBemfCounters();

        // STM32 shutdown flag - disables all motors and servos at firmware level.
        // This is the safest way to ensure motors stop on program exit/crash.
        void setShutdown(bool enabled);
    private:
        explicit LcmDataWriter();
        ~LcmDataWriter() = default;

        raccoon::Transport transport_;
    };
}
