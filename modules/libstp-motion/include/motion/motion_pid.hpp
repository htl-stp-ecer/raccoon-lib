#pragma once

#include <memory>
#include "foundation/pid.hpp"

namespace libstp::motion
{
    // Forward declaration
    struct UnifiedMotionPidConfig;

    /**
     * PID controller type for factory creation.
     */
    enum class PidType
    {
        Distance,  // For forward/backward distance control
        Heading,   // For heading/yaw control
    };

    /**
     * Factory function to create PID controllers from unified config.
     *
     * Extracts the appropriate PidConfig (distance or heading) from the
     * unified motion config and returns a ready-to-use PidController.
     */
    std::unique_ptr<foundation::PidController> createPidController(
        const UnifiedMotionPidConfig& unified_config,
        PidType type);
}
