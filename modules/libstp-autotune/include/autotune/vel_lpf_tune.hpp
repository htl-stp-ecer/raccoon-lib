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
     * The motor must be quiescent during measurement — that is the caller's
     * responsibility.
     */
    class VelLpfTuner
    {
    public:
        explicit VelLpfTuner(const std::vector<hal::motor::IMotor*>& motors);

        [[nodiscard]] VelLpfResult tuneMotor(
            hal::motor::IMotor* motor,
            const VelLpfConfig& cfg) const;

        [[nodiscard]] std::map<int, VelLpfResult> tune(
            const VelLpfConfig& cfg) const;

    private:
        std::vector<hal::motor::IMotor*> motors_;
    };

} // namespace libstp::autotune
