#pragma once

#include "drive/drive.hpp"
#include "odometry/odometry.hpp"
#include "motion/motion_config.hpp"

namespace libstp::motion
{
    /**
     * Shared runtime dependencies for a motion primitive.
     *
     * Motion controllers borrow these references for their entire lifetime and
     * assume the referenced drivetrain and odometry instances outlive them.
     */
    struct MotionContext
    {
        drive::Drive& drive;
        odometry::IOdometry& odometry;
        const UnifiedMotionPidConfig& pid_config;
    };

    /**
     * Base interface for stateful motion primitives.
     *
     * Concrete motions follow a simple lifecycle:
     * - `start()` captures initial state and primes the controller.
     * - `update(dt)` advances the controller by one loop iteration.
     * - `isFinished()` reports whether the goal has been reached.
     *
     * Callers are responsible for invoking `update(dt)` at a stable cadence.
     */
    class Motion
    {
    public:
        explicit Motion(MotionContext ctx) : ctx_(ctx) {}
        virtual ~Motion() = default;

        /** Capture initial state and prepare controller internals. */
        virtual void start() = 0;
        /** Advance the controller by one iteration using the loop delta in seconds. */
        virtual void update(double dt) = 0;
        /** Return true once the primitive has met its completion criteria. */
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
