//
// Created by tobias on 2/7/25.
//
#include "sensors/ir_light_sensor.hpp"

libstp::sensor::IrLightSensor::IrLightSensor(const int& port, const float& calibrationFactor):
    LightSensor(port, calibrationFactor)
{
}

bool libstp::sensor::IrLightSensor::isOnBlack()
{
    return read() > whiteThreshold;
}
