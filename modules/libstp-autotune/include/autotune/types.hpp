#pragma once

namespace libstp::autotune
{
    /**
     * @brief Measured physical limits for a single chassis axis.
     *
     * All velocity units are m/s (forward, lateral) or rad/s (angular).
     * Acceleration and deceleration are the corresponding per-second-squared values.
     */
    struct AxisResult
    {
        /// Peak velocity at full power (m/s or rad/s).
        double max_velocity{0.0};
        /// Acceleration from 10%–90% of max velocity (m/s² or rad/s²).
        double acceleration{0.0};
        /// Deceleration from 90%–10% during coast-down (m/s² or rad/s²).
        double deceleration{0.0};
    };

    /**
     * @brief Configuration for the DriveCharacterizer.
     */
    struct CharacterizeConfig
    {
        /// Raw PWM percentage applied during characterization (1–100).
        int power_percent{100};
        /// Number of independent trials per axis; median is taken.
        int trials{3};
        /// Maximum seconds to wait for the acceleration phase.
        double accel_timeout{3.0};
        /// Maximum seconds to record the deceleration (coast-down) phase.
        double decel_timeout{3.0};
        /// Sampling rate in Hz for position sampling (e.g. 500).
        int sample_hz{500};
    };

} // namespace libstp::autotune
