#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "autotune/bemf_velocity_tune.hpp"
#include "drive/drive.hpp"
#include "test_support/mock_imu.hpp"
#include "test_support/mock_kinematics.hpp"
#include "test_support/mock_motor.hpp"
#include "test_support/mock_odometry.hpp"

using libstp::autotune::BemfVelocityConfig;
using libstp::autotune::BemfVelocityTuner;
using libstp::odometry::OdometrySource;
using libstp::test::MockIMU;
using libstp::test::MockKinematics;
using libstp::test::MockMotor;
using libstp::test::MockOdometry;
using testing::NiceMock;

TEST(BemfVelocityTunerTest, FailsWhenOdometrySourceChangesMidTune)
{
    auto* kin_raw = new NiceMock<MockKinematics>();
    kin_raw->setupAsDifferential();

    auto motor = std::make_unique<NiceMock<MockMotor>>(0, false);
    motor->setCalibration(libstp::foundation::MotorCalibration{});
    std::vector<libstp::hal::motor::IMotor*> motors{motor.get()};

    ON_CALL(*kin_raw, getMotors()).WillByDefault([&motors]() { return motors; });

    NiceMock<MockIMU> imu;
    imu.setupDefaults();

    libstp::drive::ChassisVelocityControlConfig vel_cfg;
    libstp::drive::Drive drive(
        std::unique_ptr<libstp::kinematics::IKinematics>(kin_raw),
        vel_cfg,
        imu);

    NiceMock<MockOdometry> odom;
    odom.setupDefaults();
    ON_CALL(odom, getActiveSource())
        .WillByDefault([call_count = 0]() mutable {
            ++call_count;
            return call_count == 1 ? OdometrySource::CalibrationBoard : OdometrySource::Internal;
        });

    BemfVelocityTuner tuner(drive, odom);
    BemfVelocityConfig cfg;
    cfg.pwm_levels = {40};
    cfg.sweeps = 1;

    const auto result = tuner.tune(cfg);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(
        result.failure_reason,
        "active odometry source changed during BEMF tuning — stopping because a mid-run "
        "source swap invalidates the calibration");
}
