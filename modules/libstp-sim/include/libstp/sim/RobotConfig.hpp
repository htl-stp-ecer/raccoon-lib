#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace libstp::sim
{
    /// Mount point for a sensor on the robot, in the robot's local frame.
    /// forwardCm/strafeCm are relative to the robot's geometric center
    /// (matches the web-ide LineSensor convention: +forward = front, +strafe = left).
    struct SensorMount
    {
        std::string name;
        float forwardCm{0.0f};
        float strafeCm{0.0f};
        uint8_t analogPort{0};
        float clearanceCm{0.0f};
    };

    /// Robot footprint + kinematics + sensor placement. Mirrors the shape of
    /// web-ide ConnectionInfo + RobotConfig so ftmap/device payloads drop in
    /// without translation.
    struct RobotConfig
    {
        float widthCm{18.0f};
        float lengthCm{18.0f};

        /// Rotation center offset from geometric center. Matches the web-ide
        /// physics.ts convention used by getRectangle().
        float rotationCenterForwardCm{0.0f};
        float rotationCenterStrafeCm{0.0f};

        /// Drive kinematics (meters).
        std::string driveType{"differential"};
        float wheelRadiusM{0.03f};
        float trackWidthM{0.15f};
        float wheelbaseM{0.15f};

        std::vector<SensorMount> lineSensors;
        std::vector<SensorMount> distanceSensors;
    };
}
