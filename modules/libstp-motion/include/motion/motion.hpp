#pragma once

#include "drive/drive.hpp"
#include "odometry/odometry.hpp"
#include "motion/motion_config.hpp"

namespace libstp::motion
{
    struct MotionContext
    {
        drive::Drive& drive;
        odometry::IOdometry& odometry;
        const UnifiedMotionPidConfig& pid_config;
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
        [[nodiscard]] odometry::IOdometry& odometry() { return ctx_.odometry; }
        [[nodiscard]] const odometry::IOdometry& odometry() const { return ctx_.odometry; }

        MotionContext ctx_;
        bool started_{false};
    };
}
