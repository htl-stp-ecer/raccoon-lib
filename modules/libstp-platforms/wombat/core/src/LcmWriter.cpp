#include "core/LcmWriter.hpp"

#include <chrono>

using namespace platform::wombat::core;
namespace Channels = raccoon::Channels;

static int64_t currentTimestampUsec()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void LcmDataWriter::setMotor(uint8_t port, int valueData) {
    exlcm::scalar_i32_t publishedValue{};
    publishedValue.timestamp = currentTimestampUsec();
    publishedValue.value = valueData;
    transport_.publish(Channels::motorPowerCommand(port), publishedValue);
}

void LcmDataWriter::setMotorVelocity(uint8_t port, int32_t velocity) {
    exlcm::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = velocity;
    transport_.publish(Channels::motorVelocityCommand(port), msg);
}

void LcmDataWriter::setMotorPosition(uint8_t port, int32_t velocity, int32_t goalPosition) {
    exlcm::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = static_cast<float>(velocity);
    msg.y = static_cast<float>(goalPosition);
    msg.z = 0.0f;
    transport_.publish(Channels::motorPositionCommand(port), msg);
}

void LcmDataWriter::setMotorRelative(uint8_t port, int32_t velocity, int32_t deltaPosition) {
    exlcm::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = static_cast<float>(velocity);
    msg.y = static_cast<float>(deltaPosition);
    msg.z = 0.0f;
    transport_.publish(Channels::motorRelativeCommand(port), msg);
}

void LcmDataWriter::setServo(uint8_t port, int valueData) {
    exlcm::scalar_i32_t publishedValue{};
    publishedValue.timestamp = currentTimestampUsec();
    publishedValue.value = valueData;
    transport_.publish(Channels::servoPositionCommand(port), publishedValue);
}

void LcmDataWriter::resetBemfCounters() {
    exlcm::scalar_i32_t resetCmd{};
    resetCmd.value = 1;

    // Reset BEMF counters for all 4 motor ports
    for (int port = 0; port < 4; ++port) {
        resetCmd.timestamp = currentTimestampUsec();
        transport_.publish(Channels::bemfResetCommand(port), resetCmd);
    }
}

void LcmDataWriter::setMotorStop(uint8_t port, int value) {
    exlcm::scalar_i32_t stopCmd{};
    stopCmd.timestamp = currentTimestampUsec();
    stopCmd.value = value;
    transport_.publish(Channels::motorStopCommand(port), stopCmd);
}

void LcmDataWriter::setShutdown(bool enabled) {
    exlcm::scalar_i32_t shutdownCmd{};
    shutdownCmd.timestamp = currentTimestampUsec();
    shutdownCmd.value = enabled ? 1 : 0;
    transport_.publish(Channels::SHUTDOWN_CMD, shutdownCmd);
}

void LcmDataWriter::setImuGyroOrientation(const int8_t matrix[9]) {
    exlcm::orientation_matrix_t msg{};
    msg.timestamp = currentTimestampUsec();
    for (int i = 0; i < 9; ++i) msg.m[i] = matrix[i];
    transport_.publish(Channels::IMU_GYRO_ORIENTATION_CMD, msg);
}

void LcmDataWriter::setImuCompassOrientation(const int8_t matrix[9]) {
    exlcm::orientation_matrix_t msg{};
    msg.timestamp = currentTimestampUsec();
    for (int i = 0; i < 9; ++i) msg.m[i] = matrix[i];
    transport_.publish(Channels::IMU_COMPASS_ORIENTATION_CMD, msg);
}

void LcmDataWriter::setAxisRemap(const int8_t matrix[9]) {
    exlcm::orientation_matrix_t msg{};
    msg.timestamp = currentTimestampUsec();
    for (int i = 0; i < 9; ++i) msg.m[i] = matrix[i];
    transport_.publish(Channels::AXIS_REMAP_CMD, msg);
}

LcmDataWriter::LcmDataWriter()
    : transport_(raccoon::Transport::create())
{
}
