//
// Created by eternalRose on 11/13/25.
//

#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif
#include "hal/Analog.hpp"

namespace libstp::sensors::ir {
    /**
     * Reflectance sensor with black/white threshold classification helpers.
     *
     * The class extends the generic HAL analog sensor with calibration state
     * derived from observed black and white samples.
     */
    class IRSensor : public hal::analog::AnalogSensor
    {
    public:
        float whiteThreshold;
        float blackThreshold;
        float whiteMean = 0.0f;
        float blackMean = 0.0f;
        float whiteStdDev = 1.0f;
        float blackStdDev = 1.0f;

        /**
         * Construct an IR sensor on @p port.
         *
         * On construction the sensor automatically applies any stored
         * calibration for its port from the process-wide CalibrationStore
         * (set ``"default_port<port>"``). When no stored calibration is found
         * the sensor stays uncalibrated and any black/white query will throw
         * until it is calibrated.
         */
        explicit IRSensor(const int& port);

        /** Return true once thresholds have been established (stored, loaded or live). */
        [[nodiscard]] bool isCalibrated() const;

        /** Compute the arithmetic mean of a sample set. */
        static float mean(const std::vector<float>& v);

        /** Compute the sample standard deviation of a sample set. */
        static float stddev(const std::vector<float>& v);

        /** Compute the variance of a sample set around a known mean. */
        static float variance(const std::vector<float>& v, float m);

        /** Update thresholds and distribution statistics from raw readings. */
        bool calibrate(const std::vector<float>& values);

        /** Replace the classification thresholds without recomputing sample statistics. */
        void setCalibration(float newBlackThreshold, float newWhiteThreshold);

        /** Return true when the current reading is classified as white. */
        bool isOnWhite() const;

        /** Return true when the current reading is classified as black. */
        virtual bool isOnBlack();

        /** Estimate how likely the current reading belongs to the black cluster. */
        float probabilityOfBlack() const;
        /** Estimate how likely the current reading belongs to the white cluster. */
        float probabilityOfWhite() const;

    private:
        /** True once calibration thresholds have been set. */
        bool calibrated = false;

        /** Apply stored thresholds for this port from the CalibrationStore, if any. */
        void loadStoredCalibration();

        /** Throw if the sensor has not been calibrated yet. */
        void ensureCalibrated(const char* query) const;
    };
}
