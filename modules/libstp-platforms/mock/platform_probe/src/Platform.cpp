//
// Mock-bundle implementation of `libstp::hal::platform::Platform::probe()`.
//
// All components report ok by default so CI tests don't have to teach every
// fixture about the probe API. Tests that want to exercise failure paths
// flip an injected flag through `setMockProbeFailure()` and reset it
// afterwards via `clearMockProbeFailures()`.
//
#include "hal/Platform.hpp"

#include <array>
#include <cstdint>
#include <mutex>

namespace libstp::hal::platform
{
    namespace
    {
        constexpr std::size_t kComponentCount = 3;

        std::mutex& failureMutex()
        {
            static std::mutex m;
            return m;
        }

        std::array<bool, kComponentCount>& failureFlags()
        {
            static std::array<bool, kComponentCount> flags{false, false, false};
            return flags;
        }

        std::size_t indexOf(const Component component)
        {
            return static_cast<std::size_t>(component);
        }
    }

    ProbeResult Platform::probe(const ProbeOptions& options)
    {
        ProbeResult result;
        result.components.reserve(kComponentCount);

        bool any_required_failure = false;

        const auto check = [&](const Component c, bool required) {
            bool forced_fail;
            {
                std::lock_guard<std::mutex> lock(failureMutex());
                forced_fail = failureFlags()[indexOf(c)];
            }
            const bool ok = !forced_fail;
            result.components.push_back({
                c,
                ok,
                ok ? "mock: ok" : "mock: forced failure",
            });
            if (!ok && required) any_required_failure = true;
        };

        check(Component::Stm32Bridge, options.require_stm32);
        check(Component::Imu, options.require_imu);
        check(Component::Motors, options.require_motors);

        result.ok = !any_required_failure;
        return result;
    }

    void Platform::setMockProbeFailure(const Component component, const bool fail)
    {
        std::lock_guard<std::mutex> lock(failureMutex());
        failureFlags()[indexOf(component)] = fail;
    }

    void Platform::clearMockProbeFailures()
    {
        std::lock_guard<std::mutex> lock(failureMutex());
        failureFlags().fill(false);
    }
}
