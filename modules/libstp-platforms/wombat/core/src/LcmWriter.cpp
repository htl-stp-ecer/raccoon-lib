#include "core/LcmWriter.hpp"

#include <stdexcept>

using namespace platform::wombat::core;

void LcmDataWriter::setMotor(uint8_t port, int valueData) {
    exlcm::scalar_i32_t publishedValue{};

    publishedValue.value = valueData;
    lcm.publish("libstp/motor/" + std::to_string(port) + "/power_cmd", &publishedValue);
}

void LcmDataWriter::setServo(uint8_t port, int valueData) {
    exlcm::scalar_i32_t publishedValue{};

    publishedValue.value = valueData;
    lcm.publish("libstp/servo/" + std::to_string(port) + "/position_cmd", &publishedValue);
}

LcmDataWriter::LcmDataWriter() {
    if (!lcm.good()) {
        throw std::runtime_error("[LCM-Writer] Failed to initialize LCM");
    }
}
