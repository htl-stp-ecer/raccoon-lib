#include "core/LcmWriter.hpp"

#include <stdexcept>
#include <chrono>

using namespace platform::wombat::core;

static int64_t currentTimestampUsec()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void LcmDataWriter::setMotor(uint8_t port, int valueData) {
    exlcm::scalar_i32_t publishedValue{};
    publishedValue.timestamp = currentTimestampUsec();
    publishedValue.value = valueData;
    lcm.publish("libstp/motor/" + std::to_string(port) + "/power_cmd", &publishedValue);
}

void LcmDataWriter::setMotorVelocity(uint8_t port, int32_t velocity) {
    exlcm::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = velocity;
    lcm.publish("libstp/motor/" + std::to_string(port) + "/velocity_cmd", &msg);
}

void LcmDataWriter::setMotorPosition(uint8_t port, int32_t velocity, int32_t goalPosition) {
    exlcm::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = static_cast<float>(velocity);
    msg.y = static_cast<float>(goalPosition);
    msg.z = 0.0f;
    lcm.publish("libstp/motor/" + std::to_string(port) + "/position_cmd", &msg);
}

void LcmDataWriter::setMotorRelative(uint8_t port, int32_t velocity, int32_t deltaPosition) {
    exlcm::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = static_cast<float>(velocity);
    msg.y = static_cast<float>(deltaPosition);
    msg.z = 0.0f;
    lcm.publish("libstp/motor/" + std::to_string(port) + "/relative_cmd", &msg);
}

void LcmDataWriter::setServo(uint8_t port, int valueData) {
    exlcm::scalar_i32_t publishedValue{};
    publishedValue.timestamp = currentTimestampUsec();
    publishedValue.value = valueData;
    lcm.publish("libstp/servo/" + std::to_string(port) + "/position_cmd", &publishedValue);
}

void LcmDataWriter::requestDataDump() {
    exlcm::scalar_i32_t dumpRequest{};
    dumpRequest.timestamp = currentTimestampUsec();
    dumpRequest.value = 1;
    lcm.publish("libstp/system/dump_request", &dumpRequest);
}

void LcmDataWriter::resetBemfCounters() {
    exlcm::scalar_i32_t resetCmd{};
    resetCmd.value = 1;

    // Reset BEMF counters for all 4 motor ports
    for (int port = 0; port < 4; ++port) {
        resetCmd.timestamp = currentTimestampUsec();
        lcm.publish("libstp/bemf/" + std::to_string(port) + "/reset_cmd", &resetCmd);
    }
}

void LcmDataWriter::setMotorStop(uint8_t port, int value) {
    exlcm::scalar_i32_t stopCmd{};
    stopCmd.timestamp = currentTimestampUsec();
    stopCmd.value = value;
    lcm.publish("libstp/motor/" + std::to_string(port) + "/stop_cmd", &stopCmd);
}

void LcmDataWriter::setShutdown(bool enabled) {
    exlcm::scalar_i32_t shutdownCmd{};
    shutdownCmd.timestamp = currentTimestampUsec();
    shutdownCmd.value = enabled ? 1 : 0;
    lcm.publish("libstp/system/shutdown_cmd", &shutdownCmd);
}

void LcmDataWriter::setImuGyroOrientation(const int8_t matrix[9]) {
    exlcm::orientation_matrix_t msg{};
    msg.timestamp = currentTimestampUsec();
    for (int i = 0; i < 9; ++i) msg.m[i] = matrix[i];
    lcm.publish("libstp/imu/gyro_orientation_cmd", &msg);
}

void LcmDataWriter::setImuCompassOrientation(const int8_t matrix[9]) {
    exlcm::orientation_matrix_t msg{};
    msg.timestamp = currentTimestampUsec();
    for (int i = 0; i < 9; ++i) msg.m[i] = matrix[i];
    lcm.publish("libstp/imu/compass_orientation_cmd", &msg);
}

void LcmDataWriter::setAxisRemap(const int8_t matrix[9]) {
    exlcm::orientation_matrix_t msg{};
    msg.timestamp = currentTimestampUsec();
    for (int i = 0; i < 9; ++i) msg.m[i] = matrix[i];
    lcm.publish("libstp/imu/axis_remap_cmd", &msg);
}

LcmDataWriter::LcmDataWriter() {
    if (!lcm.good()) {
        throw std::runtime_error("[LCM-Writer] Failed to initialize LCM");
    }
}
