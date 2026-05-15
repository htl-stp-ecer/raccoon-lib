#pragma once

#include <map>
#include <string>
#include <vector>

#include "autotune/types.hpp"
#include "drive/drive.hpp"
#include "hal/odometry.hpp"

namespace libstp::autotune
{
    /**
     * @brief Characterizes a robot's physical drive limits at raw motor power.
     *
     * Bypasses all velocity control (library PID and firmware BEMF PID) by
     * commanding raw PWM via Drive::applyPowerCommand().  For each axis the
     * characterizer runs multiple accel + decel trials and takes the median of
     * each metric across trials.
     *
     * Sampling is performed at up to 500 Hz in a tight C++ sleep loop, which
     * allows significantly higher resolution than the Python asyncio-based
     * implementation (formerly ~100 Hz).
     *
     * @note DriveCharacterizer holds non-owning references to Drive and
     *       IOdometry.  The caller must ensure both objects outlive the
     *       DriveCharacterizer instance.
     */
    class DriveCharacterizer
    {
    public:
        /**
         * @param drive    Chassis drive controller (reference, not owned).
         * @param odometry Odometry source (reference, not owned).
         */
        DriveCharacterizer(drive::Drive& drive, odometry::IOdometry& odometry);

        /**
         * @brief Characterize a single axis.
         *
         * @param axis  One of "forward", "lateral", or "angular".
         * @param cfg   Measurement configuration.
         * @return AxisResult with median max_velocity, acceleration, and deceleration.
         *         Fields remain 0.0 if all trials fail.
         */
        [[nodiscard]] AxisResult characterizeAxis(const std::string& axis,
                                                  const CharacterizeConfig& cfg) const;

        /**
         * @brief Characterize multiple axes in sequence.
         *
         * @param axes  Axis names to measure (e.g. {"forward", "angular"}).
         * @param cfg   Measurement configuration shared across all axes.
         * @return Map from axis name to AxisResult.
         */
        [[nodiscard]] std::map<std::string, AxisResult>
        characterize(const std::vector<std::string>& axes,
                     const CharacterizeConfig& cfg) const;

    private:
        /// Brake all drive motors immediately.
        void brakeMotors() const;

        drive::Drive& drive_;
        odometry::IOdometry& odometry_;
    };

} // namespace libstp::autotune
