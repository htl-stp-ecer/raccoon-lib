//
// Wombat-bundle implementation of `libstp::hal::platform::Platform::probe()`.
//
// The probe relies on the LCM message stream that `stm32-data-reader`
// publishes: heading data confirms the bridge AND the IMU; BEMF data confirms
// motor telemetry. The mock-only test hooks compile to no-ops here so that
// production code paths can never accidentally force a failure.
//
#include "core/LcmReader.hpp"
#include "foundation/logging.hpp"
#include "hal/Platform.hpp"

#include <algorithm>
#include <chrono>

namespace libstp::hal::platform
{
    ProbeResult Platform::probe(const ProbeOptions& options)
    {
        ProbeResult result;
        result.components.reserve(3);

        auto& reader = ::platform::wombat::core::LcmReader::instance();

        // Stm32Bridge — proven by reception of *any* STM32-published message.
        // Heading is the most reliable since it is published continuously and
        // has retain semantics, so a fresh subscriber gets it immediately when
        // the bridge is alive.
        const bool stm32_ok = reader.waitForImuReady(options.stm32_timeout_ms);
        result.components.push_back({
            Component::Stm32Bridge,
            stm32_ok,
            stm32_ok ? "heading channel reachable"
                     : "no STM32 traffic — is stm32-data-reader running?",
        });

        // Imu — same signal as STM32 currently. Kept as a separate component
        // so callers can disable the requirement independently and so a future
        // tighter check (bias, sample rate) has a place to live.
        const bool imu_ok = stm32_ok || reader.waitForImuReady(options.imu_timeout_ms);
        result.components.push_back({
            Component::Imu,
            imu_ok,
            imu_ok ? "heading data flowing"
                   : "no IMU heading received within timeout",
        });

        // Motors — BEMF channel reception. Soft by default because the very
        // first BEMF publish can lag the IMU on a fresh boot.
        const bool motors_ok = reader.waitForBemfData(options.motor_timeout_ms);
        result.components.push_back({
            Component::Motors,
            motors_ok,
            motors_ok ? "BEMF telemetry received"
                      : "no motor BEMF received — STM32 motor block silent",
        });

        const auto required_failed = [&](Component c, bool ok) {
            switch (c)
            {
                case Component::Stm32Bridge: return options.require_stm32 && !ok;
                case Component::Imu:         return options.require_imu   && !ok;
                case Component::Motors:      return options.require_motors && !ok;
            }
            return false;
        };

        result.ok = std::none_of(
            result.components.begin(), result.components.end(),
            [&](const ComponentStatus& s) { return required_failed(s.component, s.ok); });

        return result;
    }

    // Test hooks are no-ops on the wombat: production code must never be able
    // to silently force a probe outcome.
    void Platform::setMockProbeFailure(Component, bool)
    {
        LIBSTP_LOG_WARN("[Platform] setMockProbeFailure ignored on wombat platform");
    }

    void Platform::clearMockProbeFailures()
    {
        // intentionally empty
    }
}
