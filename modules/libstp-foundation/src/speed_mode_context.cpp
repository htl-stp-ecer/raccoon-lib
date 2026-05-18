#include "foundation/speed_mode_context.hpp"

#include <stdexcept>
#include <string>

namespace libstp::foundation
{
    SpeedModeContext& SpeedModeContext::instance()
    {
        static SpeedModeContext s_instance;
        return s_instance;
    }

    bool SpeedModeContext::isSpeedModeEnabled() const noexcept
    {
        return speed_mode_enabled_.load(std::memory_order_acquire);
    }

    void SpeedModeContext::setSpeedModeEnabled(bool enabled) noexcept
    {
        speed_mode_enabled_.store(enabled, std::memory_order_release);
    }

    void SpeedModeContext::assertBemfAvailable(const char* context) const
    {
        if (speed_mode_enabled_.load(std::memory_order_acquire))
        {
            const char* tag = context ? context : "<unknown>";
            throw std::logic_error(
                std::string(tag)
                + ": BEMF is disabled (speed mode active). "
                  "Distance/angle-based termination is unavailable. "
                  "Provide an explicit until-condition instead.");
        }
    }
}
