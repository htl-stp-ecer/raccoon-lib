#include "core/LcmWriter.hpp"

#include <chrono>

using namespace platform::wombat::core;
namespace Channels = raccoon::Channels;

static int64_t currentTimestampUsec()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void LcmDataWriter::setMotor(uint8_t port, int valueData)
{
    raccoon::scalar_i32_t publishedValue{};
    publishedValue.timestamp = currentTimestampUsec();
    publishedValue.value = valueData;
    transport_.publish(Channels::motorPowerCommand(port), publishedValue);
}

void LcmDataWriter::setMotorVelocity(uint8_t port, int32_t velocity)
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = velocity;
    transport_.publish(Channels::motorVelocityCommand(port), msg);
}

void LcmDataWriter::setMotorPosition(uint8_t port, int32_t velocity, int32_t goalPosition)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = static_cast<float>(velocity);
    msg.y = static_cast<float>(goalPosition);
    msg.z = 0.0f;
    transport_.publish(Channels::motorPositionCommand(port), msg);
}

void LcmDataWriter::setMotorRelative(uint8_t port, int32_t velocity, int32_t deltaPosition)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = static_cast<float>(velocity);
    msg.y = static_cast<float>(deltaPosition);
    msg.z = 0.0f;
    transport_.publish(Channels::motorRelativeCommand(port), msg);
}

void LcmDataWriter::setServo(uint8_t port, int valueData)
{
    raccoon::scalar_i32_t publishedValue{};
    publishedValue.timestamp = currentTimestampUsec();
    publishedValue.value = valueData;
    transport_.publish(Channels::servoPositionCommand(port), publishedValue);
}

void LcmDataWriter::resetBemfCounters()
{
    raccoon::scalar_i32_t resetCmd{};
    resetCmd.value = 1;

    // Reset BEMF counters for all 4 motor ports
    for (int port = 0; port < 4; ++port)
    {
        resetCmd.timestamp = currentTimestampUsec();
        transport_.publish(Channels::bemfResetCommand(port), resetCmd);
    }
}

void LcmDataWriter::setMotorStop(uint8_t port, int value)
{
    raccoon::scalar_i32_t stopCmd{};
    stopCmd.timestamp = currentTimestampUsec();
    stopCmd.value = value;
    transport_.publish(Channels::motorStopCommand(port), stopCmd);
}

void LcmDataWriter::setShutdown(bool enabled)
{
    raccoon::scalar_i32_t shutdownCmd{};
    shutdownCmd.timestamp = currentTimestampUsec();
    shutdownCmd.value = enabled ? 1 : 0;
    transport_.publish(Channels::SHUTDOWN_CMD, shutdownCmd);
}

LcmDataWriter::LcmDataWriter()
    : transport_(raccoon::Transport::create())
{
}
