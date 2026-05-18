#pragma once

#include <chrono>

#include "foundation/motor.hpp"
#include "hal/IMotor.hpp"
#include "hal/PortRegistry.hpp"

namespace libstp::hal::motor
{
    /**
     * Concrete motor wrapper whose method bodies are provided by the selected
     * platform bundle.
     *
     * The public contract here is what higher-level modules and Python bindings
     * program against. Shared process-wide safety behavior, such as duplicate
     * port tracking and fail-safe shutdown hooks, lives in `libstp-hal`.
     */
    class Motor : public IMotor
    {
#ifdef SAFETY_CHECKS_ENABLED
        static inline detail::PortRegistry registry_{};

        static void registerMotorPort(int port);

        static void unregisterMotorPort(int port);
#endif

    public:
        /**
         * Construct a motor on the given port.
         *
         * `inverted` flips command and feedback sign conventions so callers can
         * keep chassis-level logic independent of wiring direction.
         */
        explicit Motor(int port, bool inverted, const foundation::MotorCalibration& calibration = {});

        ~Motor() override;

        /// Open-loop percent output in the range expected by the platform driver.
        void setSpeed(int percent) override;
        /// Closed-loop velocity target in firmware-specific BEMF units.
        void setVelocity(int velocity) override;
        /// Closed-loop absolute move in firmware-specific position units.
        void moveToPosition(int velocity, int goalPosition) override;
        /// Closed-loop relative move in firmware-specific position units.
        void moveRelative(int velocity, int deltaPosition) override;
        /// Return the current motor position using the driver's sign convention.
        [[nodiscard]] int getPosition() const override;
        /// Return the driver's current BEMF or velocity-like feedback value.
        [[nodiscard]] int getBemf() const override;
        /// Return whether the last position command has completed.
        [[nodiscard]] bool isDone() const override;

        /// Actively stop this motor using the platform's safest available path.
        void brake() override;
        /// Disable this motor completely (no power, no brake — free-spinning).
        void off() override;
        /// Reset the position counter to zero.
        void resetPositionCounter() override;

        [[nodiscard]] const foundation::MotorCalibration& getCalibration() const override;
        void setCalibration(const foundation::MotorCalibration& calibration) override;
        [[nodiscard]] int getPort() const override { return port_; }
        [[nodiscard]] bool isInverted() const override { return inverted_; }

        /// Push per-motor firmware-side velocity PID gains.
        ///
        /// On wombat this publishes to the STM32; on the mock platform the gains
        /// are stored for `getLastFirmwarePidGains()`.
        void setFirmwarePidGains(float kp, float ki, float kd) override;

        /// Put every motor managed by the active platform into its disabled state.
        static void disableAll();
        /// Re-enable globally disabled motors when the platform supports it.
        static void enableAll();

        // --------------------------------------------------------------------
        // Test/simulation hooks (active on mock platform; no-op on wombat).
        //
        // The members are declared in the shared class so test code that holds
        // a `Motor*` (and the autotune tests) can interrogate / control the
        // simulated motor without an upcast. On the wombat platform these are
        // never written and `getBemf()` ignores them.
        // --------------------------------------------------------------------

        /// Configure a simulated first-order BEMF step response. When active
        /// (mock platform only), `getBemf()` returns
        /// `steady_state * (1 - exp(-t / time_constant_s))` where `t` is the
        /// time since the last `setVelocity()` call.
        void setSimulatedBemfResponse(int steady_state_bemf, double time_constant_s);

        /// Disable the simulated BEMF response (mock platform only).
        void clearSimulatedBemfResponse();

        /// Read back the gains most recently passed to `setFirmwarePidGains()`.
        /// Useful for asserting tuner behavior in unit tests.
        void getLastFirmwarePidGains(float& kp, float& ki, float& kd) const override;

    private:
        int port_;
        bool inverted_;
        foundation::MotorCalibration calibration_;

        // --- Simulation / test state (mock platform writes; wombat ignores) --
        bool   sim_active_{false};
        int    sim_steady_state_{0};
        double sim_time_constant_s_{0.1};
        std::chrono::steady_clock::time_point sim_t_ref_{};

        float  last_fw_kp_{0.0f};
        float  last_fw_ki_{0.0f};
        float  last_fw_kd_{0.0f};
    };
}
