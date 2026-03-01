#include "core/LcmReader.hpp"
#include "core/LcmWriter.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include "foundation/logging.hpp"

using namespace platform::wombat::core;
namespace Channels = raccoon::Channels;

namespace {
// Age tracking — all accessed from LCM listener thread only, no sync needed
double age_max_ms = 0.0;
double age_sum_ms = 0.0;
int    age_count  = 0;

void logAge(const std::string& channel, int64_t msg_timestamp) {
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    double age_ms = (now_us - msg_timestamp) / 1000.0;
    if (age_ms > age_max_ms) age_max_ms = age_ms;
    age_sum_ms += age_ms;
    ++age_count;
    LIBSTP_LOG_TRACE("[LcmReader] {} age: {:.1f}ms", channel, age_ms);
}
}

LcmReader::LcmReader()
    : transport_(raccoon::Transport::create())
{
    using raccoon::SubscribeOptions;

    // Retained subscribe options for channels that cache latest values
    static const SubscribeOptions retainedOpts{.requestRetained = true};

    // Subscribe to servo topics (ports 0-3) — retained
    for (int port = 0; port < 4; ++port) {
        auto modeChannel = Channels::servoMode(port);
        transport_.subscribe<raccoon::scalar_i8_t>(
            modeChannel,
            [this, port, ch = modeChannel](const raccoon::scalar_i8_t& msg) {
                logAge(ch, msg.timestamp);
                std::lock_guard<std::mutex> lock(cache_mutex_);
                servo_mode_cache_[port] = msg.dir;
            }, retainedOpts);

        auto posChannel = Channels::servoPosition(port);
        transport_.subscribe<raccoon::scalar_i32_t>(
            posChannel,
            [this, port, ch = posChannel](const raccoon::scalar_i32_t& msg) {
                logAge(ch, msg.timestamp);
                std::lock_guard<std::mutex> lock(cache_mutex_);
                servo_value_cache_[port] = msg.value;
            }, retainedOpts);
    }

    // Subscribe to IMU topics — high-frequency, no retain needed
    transport_.subscribe<raccoon::vector3f_t>(
        Channels::GYRO,
        [this](const raccoon::vector3f_t& msg) {
            logAge(Channels::GYRO, msg.timestamp);
            std::lock_guard<std::mutex> lock(cache_mutex_);
            gyro_cache_ = msg;
        });

    transport_.subscribe<raccoon::vector3f_t>(
        Channels::ACCELEROMETER,
        [this](const raccoon::vector3f_t& msg) {
            logAge(Channels::ACCELEROMETER, msg.timestamp);
            std::lock_guard<std::mutex> lock(cache_mutex_);
            accel_cache_ = msg;
        });

    transport_.subscribe<raccoon::vector3f_t>(
        Channels::LINEAR_ACCELERATION,
        [this](const raccoon::vector3f_t& msg) {
            logAge(Channels::LINEAR_ACCELERATION, msg.timestamp);
            std::lock_guard<std::mutex> lock(cache_mutex_);
            linear_accel_cache_ = msg;
        });

    transport_.subscribe<raccoon::vector3f_t>(
        Channels::ACCEL_VELOCITY,
        [this](const raccoon::vector3f_t& msg) {
            logAge(Channels::ACCEL_VELOCITY, msg.timestamp);
            std::lock_guard<std::mutex> lock(cache_mutex_);
            accel_velocity_cache_ = msg;
        });

    transport_.subscribe<raccoon::vector3f_t>(
        Channels::MAGNETOMETER,
        [this](const raccoon::vector3f_t& msg) {
            logAge(Channels::MAGNETOMETER, msg.timestamp);
            std::lock_guard<std::mutex> lock(cache_mutex_);
            mag_cache_ = msg;
        });

    transport_.subscribe<raccoon::scalar_f_t>(
        Channels::HEADING,
        [this](const raccoon::scalar_f_t& msg) {
            logAge(Channels::HEADING, msg.timestamp);
            std::lock_guard<std::mutex> lock(cache_mutex_);
            heading_cache_ = msg;
            imu_heading_received_ = true;
        });

    // Subscribe to BEMF topics (indices 0-3) — retained
    for (int idx = 0; idx < 4; ++idx) {
        auto ch = Channels::backEmf(idx);
        transport_.subscribe<raccoon::scalar_i32_t>(
            ch,
            [this, idx, ch](const raccoon::scalar_i32_t& msg) {
                logAge(ch, msg.timestamp);
                std::lock_guard<std::mutex> lock(cache_mutex_);
                bemf_cache_[idx] = msg.value;
            }, retainedOpts);
    }

    // Subscribe to motor position and done topics (ports 0-3) — retained
    for (int port = 0; port < 4; ++port) {
        auto posCh = Channels::motorPosition(port);
        transport_.subscribe<raccoon::scalar_i32_t>(
            posCh,
            [this, port, ch = posCh](const raccoon::scalar_i32_t& msg) {
                logAge(ch, msg.timestamp);
                std::lock_guard<std::mutex> lock(cache_mutex_);
                motor_position_cache_[port] = msg.value;
            }, retainedOpts);

        auto doneCh = Channels::motorDone(port);
        transport_.subscribe<raccoon::scalar_i32_t>(
            doneCh,
            [this, port, ch = doneCh](const raccoon::scalar_i32_t& msg) {
                logAge(ch, msg.timestamp);
                std::lock_guard<std::mutex> lock(cache_mutex_);
                motor_done_cache_[port] = msg.value;
            }, retainedOpts);
    }

    // Subscribe to analog topics (ports 0-7)
    for (int port = 0; port < 8; ++port) {
        auto ch = Channels::analog(port);
        transport_.subscribe<raccoon::scalar_i32_t>(
            ch,
            [this, port, ch](const raccoon::scalar_i32_t& msg) {
                logAge(ch, msg.timestamp);
                std::lock_guard<std::mutex> lock(cache_mutex_);
                analog_cache_[port] = msg.value;
            });
    }

    // Subscribe to digital topics (ports 0-15)
    for (int port = 0; port < 16; ++port) {
        auto ch = Channels::digital(port);
        transport_.subscribe<raccoon::scalar_i32_t>(
            ch,
            [this, port, ch](const raccoon::scalar_i32_t& msg) {
                logAge(ch, msg.timestamp);
                std::lock_guard<std::mutex> lock(cache_mutex_);
                digital_cache_[port] = msg.value;
            });
    }

    // Subscribe to temperature topic
    transport_.subscribe<raccoon::scalar_f_t>(
        Channels::TEMPERATURE,
        [this](const raccoon::scalar_f_t& msg) {
            logAge(Channels::TEMPERATURE, msg.timestamp);
            std::lock_guard<std::mutex> lock(cache_mutex_);
            temp_cache_ = msg;
        });

    // Initialize default values
    gyro_cache_.x = 0.0f;
    gyro_cache_.y = 0.0f;
    gyro_cache_.z = 0.0f;

    accel_cache_.x = 0.0f;
    accel_cache_.y = 0.0f;
    accel_cache_.z = 0.0f;

    linear_accel_cache_.x = 0.0f;
    linear_accel_cache_.y = 0.0f;
    linear_accel_cache_.z = 0.0f;

    mag_cache_.x = 0.0f;
    mag_cache_.y = 0.0f;
    mag_cache_.z = 0.0f;

    temp_cache_.value = 0.0f;
    heading_cache_.value = 0.0f;

    // Initialize BEMF cache with zeros (hardware default)
    for (int idx = 0; idx < 4; ++idx) {
        bemf_cache_[idx] = 0;
        motor_position_cache_[idx] = 0;
        motor_done_cache_[idx] = 0;
    }

    // Initialize digital cache with zeros (hardware default)
    for (int port = 0; port < 16; ++port) {
        digital_cache_[port] = 0;
    }

    // Start background listening thread
    running_ = true;
    listener_thread_ = std::thread(&LcmReader::listenLoop, this);

    // Reset BEMF counters to prevent stale values from previous runs
    LIBSTP_LOG_DEBUG("[LcmReader] Resetting BEMF counters...");
    LcmDataWriter::instance().resetBemfCounters();
    LIBSTP_LOG_DEBUG("[LcmReader] BEMF counter reset sent");
}

LcmReader::~LcmReader() {
    running_ = false;
    transport_.stop();
    if (listener_thread_.joinable()) {
        listener_thread_.join();
    }
}

void LcmReader::listenLoop() {
    LIBSTP_LOG_DEBUG("[LcmReader] Listen loop started");
    auto stats_start = std::chrono::steady_clock::now();
    int msgs_since_log = 0;

    while (running_) {
        // Non-blocking poll for messages
        int result = transport_.spinOnce(0);
        if (result < 0) {
            LIBSTP_LOG_ERROR("[LcmReader] Error in transport spinOnce");
            continue;
        }
        if (result > 0) {
            ++msgs_since_log;
            // Drain all pending messages without blocking
            while (running_ && transport_.spinOnce(0) > 0) {
                ++msgs_since_log;
            }
        } else {
            // No messages available — brief sleep to avoid busy-spinning
            usleep(100);
        }

        // Log throughput every 5 seconds
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - stats_start).count();
        if (elapsed >= 5000) {
            double avg_ms = (age_count > 0) ? (age_sum_ms / age_count) : 0.0;
            LIBSTP_LOG_DEBUG("[LcmReader] {} msgs in {:.1f}s ({:.0f}/s) | age avg={:.1f}ms max={:.1f}ms",
                msgs_since_log, elapsed / 1000.0,
                msgs_since_log * 1000.0 / elapsed,
                avg_ms, age_max_ms);
            msgs_since_log = 0;
            age_max_ms = 0.0;
            age_sum_ms = 0.0;
            age_count = 0;
            stats_start = now;
        }
    }
    LIBSTP_LOG_DEBUG("[LcmReader] Listen loop exiting");
}

// Read methods - return cached values
raccoon::scalar_i8_t LcmReader::readServoMode(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::scalar_i8_t result;
    auto it = servo_mode_cache_.find(port);
    result.dir = (it != servo_mode_cache_.end()) ? it->second : 1;  // Default: Disabled
    return result;
}

raccoon::scalar_i32_t LcmReader::readServoValue(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::scalar_i32_t result;
    auto it = servo_value_cache_.find(port);
    result.value = (it != servo_value_cache_.end()) ? it->second : 1024;  // Default: middle position
    return result;
}

raccoon::vector3f_t LcmReader::readGyro() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return gyro_cache_;
}

raccoon::vector3f_t LcmReader::readAccel() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return accel_cache_;
}

raccoon::vector3f_t LcmReader::readLinearAccel() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return linear_accel_cache_;
}

raccoon::vector3f_t LcmReader::readMag() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return mag_cache_;
}

raccoon::scalar_f_t LcmReader::readHeading() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return heading_cache_;
}

raccoon::scalar_i32_t LcmReader::readBemf(const int idx) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::scalar_i32_t result;
    auto it = bemf_cache_.find(idx);
    result.value = (it != bemf_cache_.end()) ? it->second : 0;
    return result;
}

int32_t LcmReader::readMotorPosition(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = motor_position_cache_.find(port);
    return (it != motor_position_cache_.end()) ? it->second : 0;
}

bool LcmReader::readMotorDone(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = motor_done_cache_.find(port);
    return (it != motor_done_cache_.end()) ? it->second != 0 : false;
}

raccoon::scalar_i32_t LcmReader::readAnalog(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::scalar_i32_t result;
    auto it = analog_cache_.find(port);
    result.value = (it != analog_cache_.end()) ? it->second : 0;
    return result;
}

raccoon::scalar_i32_t LcmReader::readDigital(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::scalar_i32_t result;
    auto it = digital_cache_.find(port);
    result.value = (it != digital_cache_.end()) ? it->second : 0;
    return result;
}

raccoon::scalar_f_t LcmReader::readTemp() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return temp_cache_;
}

raccoon::vector3f_t LcmReader::readAccelVelocity() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::vector3f_t result;
    result.x = accel_velocity_cache_.x - accel_velocity_offset_.x;
    result.y = accel_velocity_cache_.y - accel_velocity_offset_.y;
    result.z = accel_velocity_cache_.z - accel_velocity_offset_.z;
    return result;
}

void LcmReader::resetAccelVelocity() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    accel_velocity_offset_ = accel_velocity_cache_;
}

bool LcmReader::waitForImuReady(int timeout_ms) {
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(timeout_ms);

    while (!imu_heading_received_) {
        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed >= timeout) {
            std::cerr << "[LcmReader] Timeout waiting for IMU heading data" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    LIBSTP_LOG_TRACE("[LcmReader] IMU heading data received");
    return true;
}
