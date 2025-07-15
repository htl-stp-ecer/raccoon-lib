//
// Created by tobias on 12/26/24.
//

#pragma once

#include <vector>
#include "hal/Analog.hpp"

namespace libstp::sensor
{
    class DistanceSensor final : public hal::analog::AnalogSensor
    {
    public:
        explicit DistanceSensor(const int& port);

        double getDistance();
    };

    class LightSensor : public hal::analog::AnalogSensor
    {
    public:
        int whiteThreshold;
        int blackThreshold;
        float whiteMean = 0.0f;
        float blackMean = 0.0f;
        float whiteStdDev = 1.0f;
        float blackStdDev = 1.0f;
        float calibrationFactor;

        explicit LightSensor(const int& port, float calibrationFactor);

        bool calibrate(const std::vector<int>& whiteValues, const std::vector<int>& blackValues);

        void wait_for_light() const;

        bool isOnWhite();

        virtual bool isOnBlack();
    };

    void calibrateLightSensors(const std::vector<LightSensor*>& lightSensors);

    bool areOnBlack(LightSensor* leftSensor, LightSensor* rightSensor);

    bool areOnWhite(LightSensor* leftSensor, LightSensor* rightSensor);

    void waitForButtonClick();

    bool isButtonClicked();
}
