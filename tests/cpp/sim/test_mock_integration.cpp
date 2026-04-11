// Integration tests proving the chain:
//   HAL Motor::setSpeed → MockPlatform::setMotor → SimWorld::setMotorCommand
//     → SimWorld::tick (via MockPlatform::tickSim)
//     → MockPlatform::simPose → OdometryBridge::readOdometry
//
// Only built when DRIVER_BUNDLE=mock (the platform_core singleton and the
// real HAL driver wiring both live in the mock bundle).

#include <gtest/gtest.h>

#include "core/MockPlatform.hpp"
#include "hal/Motor.hpp"
#include "hal/OdometryBridge.hpp"
#include "libstp/sim/SimWorld.hpp"

#include <cmath>

using namespace libstp::sim;
namespace mock = platform::mock::core;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;

    RobotConfig defaultRobot()
    {
        RobotConfig r{};
        r.widthCm = 18.0f;
        r.lengthCm = 18.0f;
        r.wheelRadiusM = 0.03f;
        r.trackWidthM = 0.15f;
        return r;
    }

    SimMotorMap defaultMotors()
    {
        SimMotorMap m{};
        m.leftPort = 0;
        m.rightPort = 1;
        m.maxWheelVelocityRadS = 30.0f;
        m.motorTimeConstantSec = 0.05f;
        return m;
    }

    void configureOpenTable(const Pose2D& startPose)
    {
        WorldMap map;
        map.setTable(400.0f, 300.0f);
        mock::MockPlatform::instance().configureSim(
            defaultRobot(), defaultMotors(), std::move(map), startPose);
    }

    void run(float seconds, float dt = 0.01f)
    {
        const int steps = static_cast<int>(std::round(seconds / dt));
        for (int i = 0; i < steps; ++i) mock::MockPlatform::instance().tickSim(dt);
    }
}

class MockSimIntegrationTest : public ::testing::Test
{
protected:
    void TearDown() override { mock::MockPlatform::instance().detachSim(); }
};

TEST_F(MockSimIntegrationTest, OdometryIsZeroWhenSimDetached)
{
    mock::MockPlatform::instance().detachSim();
    libstp::hal::odometry_bridge::OdometryBridge bridge;
    const auto snap = bridge.readOdometry();
    EXPECT_FLOAT_EQ(snap.pos_x, 0.0f);
    EXPECT_FLOAT_EQ(snap.pos_y, 0.0f);
    EXPECT_FLOAT_EQ(snap.heading, 0.0f);
    EXPECT_FALSE(mock::MockPlatform::instance().hasSim());
}

TEST_F(MockSimIntegrationTest, ConfigureSimExposesStartPoseThroughBridge)
{
    configureOpenTable({50.0f, 30.0f, kPi / 4.0f});
    libstp::hal::odometry_bridge::OdometryBridge bridge;
    const auto snap = bridge.readOdometry();
    // The bridge reports pose RELATIVE to the last reset, mirroring real
    // STM32 coprocessor behavior. configureSim() sets the initial origin to
    // the start pose, so a fresh sim reads (0, 0, 0) before any motion.
    EXPECT_NEAR(snap.pos_x, 0.0f, 1e-4f);
    EXPECT_NEAR(snap.pos_y, 0.0f, 1e-4f);
    EXPECT_NEAR(snap.heading, 0.0f, 1e-4f);

    // Ground-truth absolute pose still available via simPose().
    const auto abs_pose = mock::MockPlatform::instance().simPose();
    EXPECT_NEAR(abs_pose.x, 50.0f, 1e-4f);
    EXPECT_NEAR(abs_pose.y, 30.0f, 1e-4f);
    EXPECT_NEAR(abs_pose.theta, kPi / 4.0f, 1e-4f);
}

TEST_F(MockSimIntegrationTest, HalMotorDrivesSimForward)
{
    configureOpenTable({20.0f, 50.0f, 0.0f});

    // Drive forward through the real HAL Motor API. Both motors at +100%.
    libstp::hal::motor::Motor leftMotor{0, false};
    libstp::hal::motor::Motor rightMotor{1, false};
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(100);

    run(1.0f);

    libstp::hal::odometry_bridge::OdometryBridge bridge;
    const auto snap = bridge.readOdometry();
    // Bridge reports relative-to-start pose. Drive forward 1 s at ~0.9 m/s
    // → ~0.9 m, minus the brief motor ramp.
    EXPECT_GT(snap.pos_x, 0.80f);
    EXPECT_LT(snap.pos_x, 0.95f);
    EXPECT_NEAR(snap.pos_y, 0.0f, 5e-3f);
    EXPECT_NEAR(snap.heading, 0.0f, 5e-3f);
}

TEST_F(MockSimIntegrationTest, HalMotorTurnsSimInPlace)
{
    configureOpenTable({50.0f, 50.0f, 0.0f});

    libstp::hal::motor::Motor leftMotor{0, false};
    libstp::hal::motor::Motor rightMotor{1, false};
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(-100);

    run(0.2f);

    libstp::hal::odometry_bridge::OdometryBridge bridge;
    const auto snap = bridge.readOdometry();
    // Bridge reports relative-to-start pose. In-place spin → pos stays at 0.
    EXPECT_NEAR(snap.pos_x, 0.0f, 5e-3f);
    EXPECT_NEAR(snap.pos_y, 0.0f, 5e-3f);
    // Yaw rate = (r/W)·(ω_R − ω_L) = 0.2·(−60) ≈ −12 rad/s steady state.
    // Over 0.2 s (minus ramp), integrated heading ≈ −2.4 rad.
    EXPECT_LT(snap.heading, -1.5f);
    EXPECT_GT(snap.heading, -3.0f);
}

TEST_F(MockSimIntegrationTest, HalMotorInvertedFlipsSimDirection)
{
    configureOpenTable({100.0f, 50.0f, 0.0f});

    // `inverted=true` at the HAL layer: setSpeed(+100) means "CCW" physically.
    libstp::hal::motor::Motor leftMotor{0, true};
    libstp::hal::motor::Motor rightMotor{1, true};
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(100);

    run(1.0f);

    const auto pose = mock::MockPlatform::instance().simPose();
    // Both wheels physically reversed → robot moves −X.
    EXPECT_LT(pose.x, 100.0f);
}

TEST_F(MockSimIntegrationTest, BrakeStopsSimMotors)
{
    configureOpenTable({20.0f, 50.0f, 0.0f});

    libstp::hal::motor::Motor leftMotor{0, false};
    libstp::hal::motor::Motor rightMotor{1, false};
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(100);
    run(0.3f);
    const float xAfterDrive = mock::MockPlatform::instance().simPose().x;
    EXPECT_GT(xAfterDrive, 30.0f);

    leftMotor.brake();
    rightMotor.brake();
    run(1.0f);

    const float xAfterBrake = mock::MockPlatform::instance().simPose().x;
    // Brake sets motor duty to 0 → wheels decay toward 0 over τ.
    // After 1 s (20·τ), additional drift should be small.
    EXPECT_LT(xAfterBrake - xAfterDrive, 8.0f);
}

TEST_F(MockSimIntegrationTest, IMUGyroZReflectsSimYawRate)
{
    configureOpenTable({50.0f, 50.0f, 0.0f});

    libstp::hal::motor::Motor leftMotor{0, false};
    libstp::hal::motor::Motor rightMotor{1, false};
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(-100);
    // Let motors ramp up.
    run(0.2f);

    // gyroZ() should now reflect the sim's yaw rate, not the legacy static
    // m_imu.gyro_z value.
    const float gyroZ = mock::gyroZ();
    EXPECT_LT(gyroZ, -5.0f);   // should be around −12 rad/s minus noise
    EXPECT_GT(gyroZ, -20.0f);
}
