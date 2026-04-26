//
// Shared portions of the platform health-check API.
//
// `Platform::probe()` and the test hooks are platform-specific and live in
// the selected bundle. Everything else — the error type, the rendering
// helpers and `requireHealthy()` — is identical across platforms and lives
// here so each bundle does not duplicate it.
//
#include "hal/Platform.hpp"
#include "foundation/logging.hpp"

#include <sstream>

namespace libstp::hal::platform
{
    const char* toString(const Component component)
    {
        switch (component)
        {
            case Component::Stm32Bridge: return "stm32_bridge";
            case Component::Imu:         return "imu";
            case Component::Motors:      return "motors";
        }
        return "unknown";
    }

    std::vector<ComponentStatus> ProbeResult::failedComponents() const
    {
        std::vector<ComponentStatus> failed;
        for (const auto& c : components)
        {
            if (!c.ok) failed.push_back(c);
        }
        return failed;
    }

    std::string ProbeResult::summary() const
    {
        std::ostringstream os;
        os << "Platform probe: " << (ok ? "OK" : "FAILED") << "\n";
        for (const auto& c : components)
        {
            os << "  - " << toString(c.component)
               << ": " << (c.ok ? "ok" : "FAIL");
            if (!c.detail.empty()) os << " (" << c.detail << ")";
            os << "\n";
        }
        return os.str();
    }

    ProbeFailedError::ProbeFailedError(ProbeResult result)
        : std::runtime_error(result.summary())
        , result_(std::move(result))
    {
    }

    void Platform::requireHealthy(const ProbeOptions& options)
    {
        ProbeResult result = probe(options);
        if (!result.ok)
        {
            LIBSTP_LOG_ERROR("[Platform] probe failed:\n{}", result.summary());
            throw ProbeFailedError(std::move(result));
        }
        LIBSTP_LOG_INFO("[Platform] probe ok ({} components)", result.components.size());
    }
}
