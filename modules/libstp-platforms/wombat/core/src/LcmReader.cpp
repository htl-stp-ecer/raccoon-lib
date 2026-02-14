#include "core/LcmReader.hpp"
#include "core/LcmWriter.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include "foundation/logging.hpp"

using namespace platform::wombat::core;

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

// Parse port/index number from channel string like "libstp/motor/2/value"
// Returns the integer between the 2nd and 3rd '/' (i.e. the port number)
int parsePort(const std::string& channel) {
    // Skip past "libstp/" (7 chars), then find next '/'
    size_t start = channel.find('/', 7);
    if (start == std::string::npos) return -1;
    ++start;
    size_t end = channel.find('/', start);
    if (end == std::string::npos) end = channel.size();
    int port = 0;
    for (size_t i = start; i < end; ++i) {
        port = port * 10 + (channel[i] - '0');
    }
    return port;
}
}

LcmReader::LcmReader() {
    if (!lcm_.good()) {
        throw std::runtime_error("[LCM-Reader] Failed to initialize LCM");
    }

    // Note: Motor value and direction are echoed back from the hardware
    // These subscriptions allow us to read what was actually set
    for (int port = 0; port < 4; ++port) {
        lcm_.subscribe("libstp/motor/" + std::to_string(port) + "/value",
                      &LcmReader::handleMotorValue, this);
        lcm_.subscribe("libstp/motor/" + std::to_string(port) + "/direction",
                      &LcmReader::handleMotorDir, this);
    }

    // Subscribe to all servo topics (assuming ports 0-3)
    for (int port = 0; port < 4; ++port) {
        lcm_.subscribe("libstp/servo/" + std::to_string(port) + "/mode",
                      &LcmReader::handleServoMode, this);
        lcm_.subscribe("libstp/servo/" + std::to_string(port) + "/position",
                      &LcmReader::handleServoValue, this);
    }

    // Subscribe to IMU topics
    lcm_.subscribe("libstp/gyro/value", &LcmReader::handleGyro, this);
    lcm_.subscribe("libstp/accel/value", &LcmReader::handleAccel, this);
    lcm_.subscribe("libstp/linear_accel/value", &LcmReader::handleLinearAccel, this);
    lcm_.subscribe("libstp/mag/value", &LcmReader::handleMag, this);
    lcm_.subscribe("libstp/imu/quaternion", &LcmReader::handleOrientation, this);

    // Subscribe to BEMF topics (assuming indices 0-3)
    for (int idx = 0; idx < 4; ++idx) {
        lcm_.subscribe("libstp/bemf/" + std::to_string(idx) + "/value",
                      &LcmReader::handleBemf, this);
    }

    // Subscribe to motor position and done topics
    for (int port = 0; port < 4; ++port) {
        lcm_.subscribe("libstp/motor/" + std::to_string(port) + "/position",
                      &LcmReader::handleMotorPosition, this);
        lcm_.subscribe("libstp/motor/" + std::to_string(port) + "/done",
                      &LcmReader::handleMotorDone, this);
    }

    // Subscribe to analog topics (assuming ports 0-7)
    for (int port = 0; port < 8; ++port) {
        lcm_.subscribe("libstp/analog/" + std::to_string(port) + "/value",
                      &LcmReader::handleAnalog, this);
    }

    // Subscribe to digital topics (assuming ports 0-15)
    for (int port = 0; port < 16; ++port) {
        lcm_.subscribe("libstp/digital/" + std::to_string(port) + "/value",
                      &LcmReader::handleDigital, this);
    }

    // Subscribe to temperature topic
    lcm_.subscribe("libstp/temp/value", &LcmReader::handleTemp, this);

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
    orientation_cache_.w = 1.0f;
    orientation_cache_.x = 0.0f;
    orientation_cache_.y = 0.0f;
    orientation_cache_.z = 0.0f;

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

    // Request initial data dump to populate caches quickly
    LIBSTP_LOG_DEBUG("[LcmReader] Requesting initial data dump...");
    LcmDataWriter::instance().requestDataDump();
    LIBSTP_LOG_DEBUG("[LcmReader] Data dump request sent");

    // Reset BEMF counters to prevent stale values from previous runs
    LIBSTP_LOG_DEBUG("[LcmReader] Resetting BEMF counters...");
    LcmDataWriter::instance().resetBemfCounters();
    LIBSTP_LOG_DEBUG("[LcmReader] BEMF counter reset sent");
}

LcmReader::~LcmReader() {
    running_ = false;
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
        int result = lcm_.handleTimeout(0);
        if (result < 0) {
            LIBSTP_LOG_ERROR("[LcmReader] Error in LCM handleTimeout");
            continue;
        }
        if (result > 0) {
            ++msgs_since_log;
            // Drain all pending messages without blocking
            while (running_ && lcm_.handleTimeout(0) > 0) {
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

// Message handlers with channel parsing
void LcmReader::handleMotorValue(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
    logAge(channel, msg->timestamp);
    int port = parsePort(channel);
    if (port >= 0) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        motor_value_cache_[port] = msg->value;
    }
}

void LcmReader::handleMotorDir(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i8_t* msg) {
    logAge(channel, msg->timestamp);
    int port = parsePort(channel);
    if (port >= 0) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        motor_dir_cache_[port] = msg->dir;
    }
}

void LcmReader::handleServoMode(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i8_t* msg) {
    logAge(channel, msg->timestamp);
    int port = parsePort(channel);
    if (port >= 0) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        servo_mode_cache_[port] = msg->dir;
    }
}

void LcmReader::handleServoValue(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
    logAge(channel, msg->timestamp);
    int port = parsePort(channel);
    if (port >= 0) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        servo_value_cache_[port] = msg->value;
    }
}

void LcmReader::handleGyro(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::vector3f_t* msg) {
    logAge(channel, msg->timestamp);
    std::lock_guard<std::mutex> lock(cache_mutex_);
    gyro_cache_ = *msg;
}

void LcmReader::handleAccel(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::vector3f_t* msg) {
    logAge(channel, msg->timestamp);
    std::lock_guard<std::mutex> lock(cache_mutex_);
    accel_cache_ = *msg;
}

void LcmReader::handleLinearAccel(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::vector3f_t* msg) {
    logAge(channel, msg->timestamp);
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        linear_accel_cache_ = *msg;
    }
    // Fire callback outside cache lock to avoid deadlock
    std::function<void(float, float, float)> cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = linear_accel_callback_;
    }
    if (cb) {
        cb(msg->x, msg->y, msg->z);
    }
}

void LcmReader::handleMag(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::vector3f_t* msg) {
    logAge(channel, msg->timestamp);
    std::lock_guard<std::mutex> lock(cache_mutex_);
    mag_cache_ = *msg;
}

void LcmReader::handleOrientation(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::quaternion_t* msg) {
    logAge(channel, msg->timestamp);
    std::lock_guard<std::mutex> lock(cache_mutex_);
    orientation_cache_ = *msg;
    imu_orientation_received_ = true;
}
void LcmReader::handleBemf(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
    logAge(channel, msg->timestamp);
    int idx = parsePort(channel);
    if (idx >= 0) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        bemf_cache_[idx] = msg->value;
    }
}

void LcmReader::handleMotorPosition(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
    logAge(channel, msg->timestamp);
    int port = parsePort(channel);
    if (port >= 0) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        motor_position_cache_[port] = msg->value;
    }
}

void LcmReader::handleMotorDone(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
    logAge(channel, msg->timestamp);
    int port = parsePort(channel);
    if (port >= 0) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        motor_done_cache_[port] = msg->value;
    }
}

void LcmReader::handleAnalog(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
    logAge(channel, msg->timestamp);
    int port = parsePort(channel);
    if (port >= 0) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        analog_cache_[port] = msg->value;
    }
}

void LcmReader::handleDigital(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
    logAge(channel, msg->timestamp);
    int port = parsePort(channel);
    if (port >= 0) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        digital_cache_[port] = msg->value;
    }
}

void LcmReader::handleTemp(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_f_t* msg) {
    logAge(channel, msg->timestamp);
    std::lock_guard<std::mutex> lock(cache_mutex_);
    temp_cache_ = *msg;
}

// Read methods - return cached values
exlcm::scalar_i32_t LcmReader::readMotorValue(int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    exlcm::scalar_i32_t result;
    auto it = motor_value_cache_.find(port);
    result.value = (it != motor_value_cache_.end()) ? it->second : 0;
    return result;
}

exlcm::scalar_i8_t LcmReader::readMotorDir(int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    exlcm::scalar_i8_t result;
    auto it = motor_dir_cache_.find(port);
    result.dir = (it != motor_dir_cache_.end()) ? it->second : 0;
    return result;
}

exlcm::scalar_i8_t LcmReader::readServoMode(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    exlcm::scalar_i8_t result;
    auto it = servo_mode_cache_.find(port);
    result.dir = (it != servo_mode_cache_.end()) ? it->second : 1;  // Default: Disabled
    return result;
}

exlcm::scalar_i32_t LcmReader::readServoValue(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    exlcm::scalar_i32_t result;
    auto it = servo_value_cache_.find(port);
    result.value = (it != servo_value_cache_.end()) ? it->second : 1024;  // Default: middle position
    return result;
}

exlcm::vector3f_t LcmReader::readGyro() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return gyro_cache_;
}

exlcm::vector3f_t LcmReader::readAccel() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return accel_cache_;
}

exlcm::vector3f_t LcmReader::readLinearAccel() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return linear_accel_cache_;
}

exlcm::vector3f_t LcmReader::readMag() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return mag_cache_;
}

exlcm::quaternion_t LcmReader::readOrientation() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return orientation_cache_;
}
exlcm::scalar_i32_t LcmReader::readBemf(const int idx) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    exlcm::scalar_i32_t result;
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

exlcm::scalar_i32_t LcmReader::readAnalog(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    exlcm::scalar_i32_t result;
    auto it = analog_cache_.find(port);
    result.value = (it != analog_cache_.end()) ? it->second : 0;
    return result;
}

exlcm::scalar_i32_t LcmReader::readDigital(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    exlcm::scalar_i32_t result;
    auto it = digital_cache_.find(port);
    result.value = (it != digital_cache_.end()) ? it->second : 0;
    return result;
}

exlcm::scalar_f_t LcmReader::readTemp() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return temp_cache_;
}

void LcmReader::setLinearAccelCallback(std::function<void(float, float, float)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    linear_accel_callback_ = std::move(callback);
}

bool LcmReader::waitForImuReady(int timeout_ms) {
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(timeout_ms);

    while (!imu_orientation_received_) {
        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed >= timeout) {
            std::cerr << "[LcmReader] Timeout waiting for IMU orientation data" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    LIBSTP_LOG_TRACE("[LcmReader] IMU orientation data received");
    return true;
}
