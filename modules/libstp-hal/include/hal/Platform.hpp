//
// Hardware health-check API.
//
// Concrete implementations live in the selected platform bundle (wombat or
// mock). Higher-level code calls `Platform::probe()` once at startup to
// confirm that required sub-systems (STM32 bridge, IMU, motors) are reachable
// and producing data before any mission runs.
//
#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace libstp::hal::platform
{
    /// Sub-systems verified by `Platform::probe()`.
    ///
    /// `Stm32Bridge` confirms that `stm32-data-reader` is alive and forwarding
    /// data over LCM. `Imu` confirms that orientation data is available — on
    /// the wombat this is signalled by reception of the heading channel.
    /// `Motors` confirms that motor telemetry (BEMF) is being published.
    enum class Component : uint8_t
    {
        Stm32Bridge,
        Imu,
        Motors,
    };

    /// Stable string form for logging and Python repr.
    const char* toString(Component component);

    /// Per-component outcome of a probe call.
    struct ComponentStatus
    {
        Component component;
        bool ok;
        std::string detail;
    };

    /// Aggregate result of a probe call.
    ///
    /// `ok` is true when every required component passed. The `components`
    /// vector preserves call order so callers can render a stable report.
    struct ProbeResult
    {
        bool ok{false};
        std::vector<ComponentStatus> components{};

        /// Filter `components` down to the failed ones.
        [[nodiscard]] std::vector<ComponentStatus> failedComponents() const;

        /// Multi-line, human-readable summary suitable for logs and CLI output.
        [[nodiscard]] std::string summary() const;
    };

    /// Tunable thresholds for `Platform::probe()`.
    ///
    /// `*_timeout_ms` is the maximum time `probe()` will wait for that
    /// component's signal. `require_*` controls whether a missing signal turns
    /// into a failure (`ok = false`) or just an informational status entry.
    struct ProbeOptions
    {
        int stm32_timeout_ms{1500};
        int imu_timeout_ms{1500};
        int motor_timeout_ms{500};

        bool require_stm32{true};
        bool require_imu{true};
        // Off by default: the BEMF channel is published by stm32-data-reader
        // continuously, but on a freshly powered board the first publish can
        // take longer than the IMU heading. Treat motor telemetry as a soft
        // signal unless the caller explicitly opts in.
        bool require_motors{false};
    };

    /// Thrown by `Platform::requireHealthy()` when at least one required
    /// component fails. Carries the full `ProbeResult` so callers can render a
    /// detailed diagnostic without re-running the probe.
    class ProbeFailedError : public std::runtime_error
    {
    public:
        explicit ProbeFailedError(ProbeResult result);
        [[nodiscard]] const ProbeResult& result() const noexcept { return result_; }

    private:
        ProbeResult result_;
    };

    /// Hardware-health entry point. Implemented per platform bundle.
    class Platform
    {
    public:
        /// Run the platform health-check synchronously. Safe to call multiple
        /// times; callers typically invoke it once at the start of `run()`.
        static ProbeResult probe(const ProbeOptions& options = {});

        /// Convenience that calls `probe()` and throws `ProbeFailedError` when
        /// a required component fails. Used by `GenericRobot._run_missions`
        /// to refuse to start on a broken robot.
        static void requireHealthy(const ProbeOptions& options = {});

        /// Test hook (mock platform only). Forces the named component to
        /// report a failure on the next `probe()` call. No-op on wombat so
        /// production callers are immune to misuse.
        static void setMockProbeFailure(Component component, bool fail);

        /// Reset all forced failures set by `setMockProbeFailure`.
        static void clearMockProbeFailures();
    };
}
