//
// Created by eternalRose on 11/13/25.
//

#include "../include/IRSensor.hpp"

using namespace libstp::sensors::ir;

IRSensor::IRSensor(const int &port, float calibrationFactor) : AnalogSensor(port),
                                                               whiteThreshold(0),
                                                               blackThreshold(0),
                                                               calibrationFactor(calibrationFactor * 0.5) {
}



void IRSensor::setCalibration(const int newBlackThreshold, const int newWhiteThreshold) {
    this->whiteThreshold=newWhiteThreshold;
    this->blackThreshold=newBlackThreshold;
}

bool IRSensor::isOnWhite() const {
    return read() < whiteThreshold;
}

bool IRSensor::isOnBlack() {
    return read() > blackThreshold;
}

bool libstp::sensors::ir::areOnBlack(IRSensor *leftSensor, IRSensor *rightSensor) {
    return leftSensor->isOnBlack() || rightSensor->isOnBlack();
}

bool libstp::sensors::ir::areOnWhite(IRSensor *leftSensor, IRSensor *rightSensor) {
    return leftSensor->isOnWhite() || rightSensor->isOnWhite();
}
