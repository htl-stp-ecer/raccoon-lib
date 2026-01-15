//
// Created by eternalRose on 11/13/25.
//

#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif
#include "hal/Analog.hpp"

namespace libstp::sensors::ir {
    class IRSensor : public hal::analog::AnalogSensor
    {
    public:
        float whiteThreshold;
        float blackThreshold;
        float whiteMean = 0.0f;
        float blackMean = 0.0f;
        float whiteStdDev = 1.0f;
        float blackStdDev = 1.0f;

        explicit IRSensor(const int& port);

        static float mean(const std::vector<float>& v);

        static float stddev(const std::vector<float>& v);

        static float variance(const std::vector<float>& v, float m);

        bool calibrate(const std::vector<float>& values);

        void setCalibration(float newBlackThreshold, float newWhiteThreshold);

        bool isOnWhite() const;

        virtual bool isOnBlack();

        float probabilityOfBlack() const;
        float probabilityOfWhite() const;
    };
}