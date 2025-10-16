#pragma once

#include "drive/drive.hpp"
#include "hal/IMU.hpp"

namespace libstp::motion
{
    struct MotionContext
    {
        drive::Drive& drive;
        hal::imu::IMU& imu;
    };

    class Motion
    {
    public:
        explicit Motion(MotionContext ctx) : ctx_(ctx) {}
        virtual ~Motion() = default;

        virtual void start() = 0;
        virtual void update(double dt) = 0;
        [[nodiscard]] virtual bool isFinished() const = 0;

    protected:
        [[nodiscard]] drive::Drive& drive() { return ctx_.drive; }
        [[nodiscard]] const drive::Drive& drive() const { return ctx_.drive; }
        [[nodiscard]] hal::imu::IMU& imu() { return ctx_.imu; }
        [[nodiscard]] const hal::imu::IMU& imu() const { return ctx_.imu; }

        MotionContext ctx_;
        bool started_{false};
    };
}
