#pragma once

#include <map>
#include <vector>

#include "autotune/types.hpp"
#include "hal/IMotor.hpp"

namespace libstp::autotune
{
    /**
     * @brief Per-motor static-friction (kS) measurement.
     *
     * Sweeps the PWM duty from `start_pct` upward until the measured BEMF
     * median crosses a "moving" threshold; repeats in reverse. The minimum
     * percentage at which the motor starts spinning is the static-friction
     * compensation value used by feedforward controllers.
     */
    class StaticFrictionMeasurer
    {
    public:
        explicit StaticFrictionMeasurer(const std::vector<hal::motor::IMotor*>& motors);

        /**
         * @brief Measure a single motor.
         */
        [[nodiscard]] StaticFrictionResult measureMotor(
            hal::motor::IMotor*         motor,
            const StaticFrictionConfig& cfg) const;

        /**
         * @brief Measure every motor passed at construction.
         *
         * @return Map from motor port to measurement result.
         */
        [[nodiscard]] std::map<int, StaticFrictionResult> measure(
            const StaticFrictionConfig& cfg) const;

    private:
        std::vector<hal::motor::IMotor*> motors_;
    };

} // namespace libstp::autotune
