//
// Created by eternalRose on 1/29/26.
//

//
// LCM Writer Contract Test
// Ensures all channels accept proper messages
//

#include <gtest/gtest.h>
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <chrono>
#include <mutex>
#include <unordered_map>

#include "exlcm/scalar_i32_t.hpp"
#include "exlcm/scalar_i8_t.hpp"
#include "exlcm/scalar_f_t.hpp"
#include "exlcm/vector3f_t.hpp"
#include "exlcm/quaternion_t.hpp"

class LcmWriterContractTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(lcm.good());

        for (int port = 0; port < 4; ++port) {
            lcm.subscribe("libstp/motor/" + std::to_string(port) + "/power_cmd",
                          &LcmWriterContractTest::handleMotorValue, this);
            lcm.subscribe("libstp/motor/" + std::to_string(port) + "/stop_cmd",
                          &LcmWriterContractTest::handleMotorStop, this);
        }

        for (int port = 0; port < 4; ++port) {
            lcm.subscribe("libstp/servo/" + std::to_string(port) + "/position_cmd",
                          &LcmWriterContractTest::handleServoValue, this);
        }

        for (int port = 0; port < 4; ++port) {
            lcm.subscribe("libstp/bemf/" + std::to_string(port) + "/reset_cmd",
                          &LcmWriterContractTest::handleBemfReset, this);
        }

        lcm.subscribe("libstp/system/dump_request",
                      &LcmWriterContractTest::handleDumpRequest, this);

        running = true;
        listener_thread = std::thread([this]() {
            while (running) {
                lcm.handleTimeout(50);
            }
        });
    }

    void TearDown() override {
        running = false;
        if (listener_thread.joinable())
            listener_thread.join();
    }

    void handleMotorValue(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
        int port = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        motor_values_[port] = msg->value;
    }

    void handleMotorStop(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
        int port = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        motor_stops_[port] = msg->value;
    }

    void handleServoValue(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
        int port = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        servo_values_[port] = msg->value;
    }

    void handleBemfReset(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
        int port = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        bemf_reset_[port] = msg->value;
    }

    void handleDumpRequest(const lcm::ReceiveBuffer*, const std::string&, const exlcm::scalar_i32_t* msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        dump_request_ = msg->value;
    }

    lcm::LCM lcm;
    std::thread listener_thread;
    bool running{false};
    std::mutex mutex_;

    std::unordered_map<int,int> motor_values_;
    std::unordered_map<int,int> motor_stops_;
    std::unordered_map<int,int> servo_values_;
    std::unordered_map<int,int> bemf_reset_;
    int dump_request_{0};
};


TEST_F(LcmWriterContractTest, MotorPowerCommand) {
    exlcm::scalar_i32_t msg{}; msg.value = 77;
    lcm.publish("libstp/motor/1/power_cmd", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(motor_values_[1], 77);
}

TEST_F(LcmWriterContractTest, MotorStopCommand) {
    exlcm::scalar_i32_t msg{}; msg.value = 88;
    lcm.publish("libstp/motor/2/stop_cmd", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(motor_stops_[2], 88);
}

TEST_F(LcmWriterContractTest, ServoPositionCommand) {
    exlcm::scalar_i32_t msg{}; msg.value = 512;
    lcm.publish("libstp/servo/0/position_cmd", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(servo_values_[0], 512);
}

TEST_F(LcmWriterContractTest, BemfResetCommand) {
    exlcm::scalar_i32_t msg{}; msg.value = 1;
    lcm.publish("libstp/bemf/3/reset_cmd", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(bemf_reset_[3], 1);
}

TEST_F(LcmWriterContractTest, DataDumpRequest) {
    exlcm::scalar_i32_t msg{}; msg.value = 1;
    lcm.publish("libstp/system/dump_request", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(dump_request_, 1);
}
