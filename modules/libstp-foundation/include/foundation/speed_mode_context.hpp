#pragma once

#include <atomic>

namespace libstp::foundation
{
    /**
     * Process-wide flag indicating whether SpeedMode is active.
     *
     * SpeedMode disables the firmware-side BEMF closed-loop control: the
     * STM32 stops sampling BEMF (gaining ~10% top speed) but in exchange
     * gives up encoder-velocity feedback and therefore cm-accurate
     * distance / angle termination. While SpeedMode is on the motion
     * primitives must not be used with a distance- or angle-based goal;
     * the program is expected to terminate motions via explicit
     * `until`-conditions (sensors, timeouts, ...).
     *
     * The flag lives in `libstp-foundation` so both the drive layer
     * (`MotorAdapter`, which switches between BEMF-velocity and open-loop
     * PWM) and the motion layer (which validates that motions with a
     * positional goal are not started while SpeedMode is active) can
     * query it without introducing a cross-module dependency.
     *
     * Reads and writes use atomic operations so the Python step that
     * flips SpeedMode (in response to an LCM ACK from the firmware) does
     * not need to synchronize against the motion control loop with an
     * extra mutex.
     */
    class SpeedModeContext
    {
    public:
        /// Access the process-wide instance.
        static SpeedModeContext& instance();

        /// True when SpeedMode is active (BEMF disabled, no cm precision).
        [[nodiscard]] bool isSpeedModeEnabled() const noexcept;

        /// Flip the SpeedMode flag. Intended to be called from
        /// the `SetSpeedMode` Python step after the firmware ACK arrived.
        void setSpeedModeEnabled(bool enabled) noexcept;

        /// Throws `std::logic_error` if SpeedMode is currently active.
        ///
        /// `context` is included in the error message so the user knows
        /// which call site triggered the failure.
        void assertBemfAvailable(const char* context) const;

        SpeedModeContext(const SpeedModeContext&)            = delete;
        SpeedModeContext& operator=(const SpeedModeContext&) = delete;
        SpeedModeContext(SpeedModeContext&&)                 = delete;
        SpeedModeContext& operator=(SpeedModeContext&&)      = delete;

    private:
        SpeedModeContext() = default;

        std::atomic<bool> speed_mode_enabled_{false};
    };
}
