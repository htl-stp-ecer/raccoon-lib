#include "ETSensor.hpp"

using namespace libstp::sensors::et;

ETSensor::ETSensor(int port)
    : AnalogSensor(port) {}

int ETSensor::raw() const {
    return read();
}
