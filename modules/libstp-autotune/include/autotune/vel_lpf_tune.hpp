#pragma once

#include <map>
#include <vector>

#include "autotune/types.hpp"
#include "hal/IMotor.hpp"

namespace libstp::autotune
{
    /**
     * @brief Tunes the per-motor encoder-velocity LPF alpha
     *        (`MotorCalibration::vel_lpf_alpha`).
     *
     * For each candidate alpha the tuner replays the raw BEMF stream through an
     * in-memory IIR filter (`filt = (1-a)*filt + a*raw`) and scores the result
     * by combining (a) variance of the filtered series (noise) and (b) lag,
     * measured as the standard deviation of the per-sample difference.
     *
     * Crucially, the BEMF must be sampled *while the motor is spinning*: the LPF
     * filters the running velocity estimate, and a motor at standstill produces
     * only deadzone noise (nothing to tune). `tune()` therefore spins every
     * drive motor forward at `cfg.spin_percent`, lets them settle, samples all
     * motors simultaneously during one straight run, then stops and scores each.
     */
    class VelLpfTuner
    {
    public:
        explicit VelLpfTuner(const std::vector<hal::motor::IMotor*>& motors);

        /// Spin a single motor, capture its running BEMF, and tune its alpha.
        /// (Standalone helper; the chassis curves since only one wheel drives.)
        [[nodiscard]] VelLpfResult tuneMotor(
            hal::motor::IMotor* motor,
            const VelLpfConfig& cfg) const;

        /// Spin all motors forward together, sample simultaneously, tune each.
        [[nodiscard]] std::map<int, VelLpfResult> tune(
            const VelLpfConfig& cfg) const;

    private:
        /// Sweep alpha over a pre-captured raw BEMF series, pick the best score,
        /// and write the tuned alpha back to the motor's calibration.
        [[nodiscard]] VelLpfResult scoreFromRaw(
            hal::motor::IMotor* motor,
            std::vector<int>    raw,
            const VelLpfConfig& cfg) const;

        std::vector<hal::motor::IMotor*> motors_;
    };

} // namespace libstp::autotune
