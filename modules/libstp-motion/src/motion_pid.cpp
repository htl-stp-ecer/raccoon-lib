#include "motion/motion_pid.hpp"
#include "motion/motion_config.hpp"

namespace libstp::motion
{
    std::unique_ptr<foundation::PidController> createPidController(
        const UnifiedMotionPidConfig& unified_config,
        PidType type)
    {
        switch (type)
        {
        case PidType::Distance:
            return std::make_unique<foundation::PidController>(unified_config.distance);
        case PidType::Heading:
            return std::make_unique<foundation::PidController>(unified_config.heading);
        }
        return std::make_unique<foundation::PidController>(unified_config.distance);
    }
}
