#pragma once

#include <memory>

namespace libstp::motion
{
    // Forward declaration
    struct UnifiedMotionPidConfig;

    class MotionPidController
    {
    public:
        struct Config
        {
            double kp{1.0};
            double ki{0.0};
            double kd{0.0};
            double integral_max{10.0};        // Anti-windup limit
            double integral_deadband{0.01};   // Don't integrate within this error
            double derivative_lpf_alpha{0.1}; // Low-pass filter coefficient
            double output_min{-10.0};
            double output_max{10.0};
        };

        explicit MotionPidController(Config config);

        double update(double error, double dt);
        void reset();
        void setGains(double kp, double ki, double kd);

        // Diagnostics
        double getIntegral() const { return integral_; }
        double getDerivative() const { return filtered_derivative_; }

    private:
        Config cfg_;
        double integral_{0.0};
        double prev_error_{0.0};
        double filtered_derivative_{0.0};
        bool first_update_{true};
    };

    /**
     * PID controller type for factory creation.
     */
    enum class PidType
    {
        Distance,  // For forward/backward distance control
        Heading,   // For heading/yaw control
        Lateral    // For lateral drift correction
    };

    /**
     * Factory function to create PID controllers from unified config.
     *
     * This eliminates boilerplate when creating multiple PID controllers
     * with the same shared parameters but different gains.
     *
     * @param unified_config The unified motion PID configuration
     * @param type The type of PID controller to create
     * @return A unique_ptr to the configured PID controller
     */
    std::unique_ptr<MotionPidController> createPidController(
        const UnifiedMotionPidConfig& unified_config,
        PidType type);
}
