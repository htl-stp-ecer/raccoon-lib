#pragma once

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace libstp::foundation
{
    /// Default encoder ticks per output-shaft revolution.
    /// Matches the KIPR Wombat motor encoder; override per-robot via
    /// MotorCalibration::ticks_to_rad if your hardware differs.
    inline constexpr double kDefaultEncoderTicksPerRev = 1440.0;

    /// Basic PID gains without integrator and output shaping parameters.
    struct PidGains
    {
        double kp{1.0}, ki{0.0}, kd{0.0};
    };

    /// Feedforward terms for velocity and acceleration control.
    struct Feedforward
    {
        double kS{0.0};
        double kV{0.0};
        double kA{0.0};
    };

    /// Calibration constants commonly needed for motor encoder conversions.
    struct MotorCalibration
    {
        double ticks_to_rad{2.0 * std::numbers::pi / kDefaultEncoderTicksPerRev};
        double vel_lpf_alpha{0.5};

        /// Throws std::invalid_argument if the calibration would silently
        /// corrupt odometry. ticks_to_rad must be strictly positive (zero
        /// would divide by zero downstream); vel_lpf_alpha is an IIR weight
        /// and must lie in [0, 1].
        void validate() const
        {
            if (!(ticks_to_rad > 0.0))
                throw std::invalid_argument("MotorCalibration: ticks_to_rad must be > 0");
            if (!(vel_lpf_alpha >= 0.0 && vel_lpf_alpha <= 1.0))
                throw std::invalid_argument("MotorCalibration: vel_lpf_alpha must be in [0, 1]");
        }
    };

    /// Stateless calculator for the configured feedforward output.
    class FeedforwardController
    {
    public:
        Feedforward config;

        explicit FeedforwardController(Feedforward ff = {}) : config(ff) {}

        [[nodiscard]] double calculate(double velocity, double acceleration = 0.0) const
        {
            double output = config.kV * velocity + config.kA * acceleration;
            if (velocity != 0.0)
            {
                output += config.kS * std::copysign(1.0, velocity);
            }
            return output;
        }
    };
}
