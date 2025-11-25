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
        int whiteThreshold;
        int blackThreshold;
        float whiteMean = 0.0f;
        float blackMean = 0.0f;
        float whiteStdDev = 1.0f;
        float blackStdDev = 1.0f;
        float calibrationFactor;

        explicit IRSensor(const int& port, float calibrationFactor);

        void setCalibration(int newBlackThreshold, int newWhiteThreshold);

        bool isOnWhite() const;

        virtual bool isOnBlack();

        float probabilityOfBlack();
        float probabilityOfWhite();
    };

}