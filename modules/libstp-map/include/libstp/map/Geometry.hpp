#pragma once

// Plain geometry primitives shared by libstp::map. Keep these as PODs:
// they're hot-path types (particle filter projects sensor positions
// per-tick) and we want them trivially copyable into vectors.

namespace libstp::map
{
    /// Point in field coordinates, centimeters. Origin = bottom-left of the
    /// table, +X right, +Y up — matches the Botball convention used by every
    /// other coordinate-aware type in raccoon.
    struct Point2D
    {
        float xCm{0.0f};
        float yCm{0.0f};
    };

    /// Sensor mount offset from the robot geometric center, in robot-local
    /// coordinates. forward_cm is along the robot's heading, strafe_cm is 90°
    /// CCW from heading (i.e. positive = the robot's left). Mirrors
    /// raccoon.robot.geometry.SensorPosition so the Python side can pass
    /// either this struct or a duck-typed dataclass.
    struct SensorOffset
    {
        float forwardCm{0.0f};
        float strafeCm{0.0f};
    };

    /// Wheel mount offset from the robot geometric center. Same axes as
    /// SensorOffset; lives here so a future kinematics path can compute
    /// per-wheel field positions without pulling in libstp-sim.
    struct WheelOffset
    {
        float forwardCm{0.0f};
        float strafeCm{0.0f};
    };
}
