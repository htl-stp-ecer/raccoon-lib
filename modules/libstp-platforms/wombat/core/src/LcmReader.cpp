#include "core/LcmReader.hpp"
#include <iostream>

using namespace platform::wombat::core;

LcmReader::LcmReader() {
    if (!lcm_.good()) {
        throw std::runtime_error("[LCM-Reader] Failed to initalize LCM");
    }
}

template <typename T>
T LcmReader::readOnce(const std::string& channel, int timeout_ms) {
    struct LocalHandler {
        T result{};
        bool gotMessage{false};

        void handleMessage(const lcm::ReceiveBuffer*,
                           const std::string&,
                           const T* msg) {
            result = *msg;
            gotMessage = true;
        }
    } localHandler;

    auto sub = lcm_.subscribe("libstp/" + channel, &LocalHandler::handleMessage, &localHandler);

    const int status = lcm_.handleTimeout(timeout_ms);
    lcm_.unsubscribe(sub);

    if (!localHandler.gotMessage || status <= 0) {
        throw std::runtime_error("[LCM-Reader] Time out occurred while reading the channel: " + channel);
    }

    return localHandler.result;
}


exlcm::scalar_i32_t LcmReader::readMotorValue(int port) {
    return readOnce<exlcm::scalar_i32_t>(std::string("motor/") + std::to_string(port) + std::string("/direction"));
}

exlcm::scalar_i8_t LcmReader::readMotorDir(int port) {
    return readOnce<exlcm::scalar_i8_t>(std::string("motor/") + std::to_string(port) + std::string("/value"));
}

exlcm::scalar_i8_t LcmReader::readServoMode(const int port) {
    return readOnce<exlcm::scalar_i8_t>(std::string("servo/") + std::to_string(port) + std::string("/mode"));
}

exlcm::scalar_i32_t LcmReader::readServoValue(const int port) {
    return readOnce<exlcm::scalar_i32_t>(std::string("servo/") + std::to_string(port) + std::string("/position"));
}

exlcm::vector3f_t LcmReader::readGyro() {
    return readOnce<exlcm::vector3f_t>(std::string("gyro/value"));
}

exlcm::vector3f_t LcmReader::readAccel() {
    return readOnce<exlcm::vector3f_t>(std::string("accel/value"));
}

exlcm::vector3f_t LcmReader::readMag() {
    return readOnce<exlcm::vector3f_t>("mag/value");
}

exlcm::scalar_i32_t LcmReader::readBemf(const int idx) {
    return readOnce<exlcm::scalar_i32_t>(std::string("bemf/") + std::to_string(idx) + std::string("/value"));
}

exlcm::scalar_i32_t LcmReader::readAnalog(const int port) {
    return readOnce<exlcm::scalar_i32_t>(std::string("analog/") + std::to_string(port) + std::string("/value"));
}

exlcm::scalar_i32_t LcmReader::readDigital(const int port) {
    return readOnce<exlcm::scalar_i32_t>(std::string("digital/") + std::to_string(port) + std::string("/value"));
}

exlcm::scalar_f_t LcmReader::readTemp() {
    return readOnce<exlcm::scalar_f_t>("temp/value");
}
