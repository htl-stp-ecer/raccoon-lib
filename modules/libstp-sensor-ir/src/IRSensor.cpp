//
// Created by eternalRose on 11/13/25.
//

#include "IRSensor.hpp"

#include <iostream>
#include <numeric>

#include "spdlog/spdlog.h"
using namespace libstp::sensors::ir;

IRSensor::IRSensor(const int &port, float calibrationFactor) : AnalogSensor(port),
                                                               whiteThreshold(0),
                                                               blackThreshold(0),
                                                               calibrationFactor(calibrationFactor * 0.5) {
}

void IRSensor::setCalibration(const float newBlackThreshold, const float newWhiteThreshold) {
    this->whiteThreshold=newWhiteThreshold;
    this->blackThreshold=newBlackThreshold;
}

bool IRSensor::isOnWhite() const {
    return read() < whiteThreshold;
}

bool IRSensor::isOnBlack() {
    return read() > blackThreshold;
}

float IRSensor::probabilityOfBlack() {
    float value = read();

    const float low = blackThreshold;
    const float high = whiteThreshold;

    if (value <= low) return 1.0f;
    if (value >= high) return 0.0f;
    const float t = (value - low) / (high - low);
    std::cout << value << " ; " <<  t << std::endl;

    return 1.0f - t;
}

float IRSensor::probabilityOfWhite() {
    return 1.0f - probabilityOfBlack();
}