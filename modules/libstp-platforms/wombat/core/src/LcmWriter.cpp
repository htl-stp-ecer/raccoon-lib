#include "core/LcmWriter.hpp"

#include <raccoon/Options.h>
#include <chrono>

using namespace platform::wombat::core;
namespace Channels = raccoon::Channels;

static const raccoon::PublishOptions reliableOpts{.reliable = true};

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
    transport_.publish(Channels::motorPositionCommand(port), msg, reliableOpts);
}

void LcmDataWriter::setMotorRelative(uint8_t port, int32_t velocity, int32_t deltaPosition)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = static_cast<float>(velocity);
    msg.y = static_cast<float>(deltaPosition);
    msg.z = 0.0f;
    transport_.publish(Channels::motorRelativeCommand(port), msg, reliableOpts);
}

void LcmDataWriter::setServo(uint8_t port, float degrees)
{
    raccoon::scalar_f_t publishedValue{};
    publishedValue.timestamp = currentTimestampUsec();
    publishedValue.value = degrees;
    transport_.publish(Channels::servoPositionCommand(port), publishedValue, reliableOpts);
}

void LcmDataWriter::setServoMode(uint8_t port, uint8_t mode)
{
    raccoon::scalar_i8_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.dir = static_cast<int8_t>(mode);
    transport_.publish(Channels::servoMode(port), msg, reliableOpts);
}

void LcmDataWriter::setMotorPid(uint8_t port, float kp, float ki, float kd)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = kp;
    msg.y = ki;
    msg.z = kd;
    transport_.publish(Channels::motorPidCommand(port), msg, reliableOpts);
}

void LcmDataWriter::resetMotorPosition(uint8_t port)
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = 1;
    transport_.publish(Channels::motorPositionResetCommand(port), msg, reliableOpts);
}

void LcmDataWriter::setMotorMode(uint8_t port, int mode)
{
    raccoon::scalar_i32_t modeCmd{};
    modeCmd.timestamp = currentTimestampUsec();
    modeCmd.value = mode;
    transport_.publish(Channels::motorModeCommand(port), modeCmd, reliableOpts);
}

void LcmDataWriter::setShutdown(bool enabled)
{
    raccoon::scalar_i32_t shutdownCmd{};
    shutdownCmd.timestamp = currentTimestampUsec();
    shutdownCmd.value = enabled ? 1 : 0;
    transport_.publish(Channels::SHUTDOWN_CMD, shutdownCmd, reliableOpts);
}

void LcmDataWriter::sendKinematicsConfig(const std::array<std::array<float, 4>, 3>& inv_matrix,
                                          const std::array<float, 4>& ticks_to_rad,
                                          const std::array<std::array<float, 3>, 4>& fwd_matrix)
{
    raccoon::kinematics_config_t msg{};
    msg.timestamp = currentTimestampUsec();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            msg.inv_matrix[r * 4 + c] = inv_matrix[r][c];
    for (int i = 0; i < 4; ++i)
        msg.ticks_to_rad[i] = ticks_to_rad[i];
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 3; ++c)
            msg.fwd_matrix[r * 3 + c] = fwd_matrix[r][c];
    transport_.publish(Channels::KINEMATICS_CONFIG_CMD, msg, reliableOpts);
}

void LcmDataWriter::resetOdometry()
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = 1;
    transport_.publish(Channels::ODOM_RESET_CMD, msg, reliableOpts);
}

LcmDataWriter::LcmDataWriter()
    : transport_(raccoon::Transport::create())
{
}
