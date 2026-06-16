#pragma once

#include <atomic>

namespace libstp::foundation
{
    /// Sink for a body-frame chassis velocity command [vx (m/s), vy (m/s),
    /// wz (rad/s)]. Returns true if the command was handled by an on-board
    /// (coprocessor) chassis loop, false otherwise. Must be a non-capturing
    /// function (stored as a plain pointer so it can live in a lock-free atomic).
    using ChassisVelocitySink = bool (*)(double vx, double vy, double wz);

    /**
     * Process-wide registry for the optional on-MCU chassis velocity loop.
     *
     * The flag/sink lives in `libstp-foundation` (a single shared instance)
     * so the drive layer (`Drive::update`) can forward the body-velocity
     * command to the coprocessor WITHOUT taking a link-time dependency on any
     * platform bundle — the platform bundle registers the sink at startup
     * (e.g. wombat `Platform::createOdometry` → `TransportWriter`). Mirrors
     * `SpeedModeContext`: `instance()` is defined out-of-line so the static
     * lives in `libfoundation.so` and every module shares the same registry.
     *
     * When a sink is registered (real hardware), the STM32 performs the inverse
     * kinematics + per-motor velocity PID, so the Pi runs NO host-side IK or
     * velocity control. With no sink (mock/sim), `command()` returns false and
     * the Drive falls back to host-side control.
     */
    class ChassisControlContext
    {
    public:
        /// Access the process-wide instance.
        static ChassisControlContext& instance();

        /// Register the on-MCU chassis sink. Called once by the platform bundle.
        void setSink(ChassisVelocitySink sink) noexcept
        {
            sink_.store(sink, std::memory_order_release);
        }

        /// True when an on-MCU chassis loop is registered.
        [[nodiscard]] bool hasSink() const noexcept
        {
            return sink_.load(std::memory_order_acquire) != nullptr;
        }

        /// Forward a body-frame velocity command to the registered sink.
        /// @return true if a sink handled it on-MCU; false if none is registered.
        [[nodiscard]] bool command(double vx, double vy, double wz) const noexcept
        {
            const ChassisVelocitySink s = sink_.load(std::memory_order_acquire);
            return s != nullptr && s(vx, vy, wz);
        }

        ChassisControlContext(const ChassisControlContext&)            = delete;
        ChassisControlContext& operator=(const ChassisControlContext&) = delete;
        ChassisControlContext(ChassisControlContext&&)                 = delete;
        ChassisControlContext& operator=(ChassisControlContext&&)      = delete;

    private:
        ChassisControlContext() = default;

        std::atomic<ChassisVelocitySink> sink_{nullptr};
    };
}
