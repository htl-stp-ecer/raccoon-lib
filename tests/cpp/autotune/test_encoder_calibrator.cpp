// Unit tests for libstp::autotune::EncoderCalibrator.
//
// EncoderCalibrator depends on `drive::Drive` (which itself owns an
// IKinematics + holds an IIMU&) and an `IOdometry&`. Building a fully
// functional kinematics + odometry pipeline in a unit test is out of
// scope here; we instead exercise the early-out path that requires no
// motion and is therefore robust to the mock environment: a drive with
// zero motors must fail with `failure_reason == "no motors on drive"`.
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <vector>

#include "autotune/encoder_calibrator.hpp"
#include "autotune/types.hpp"
#include "drive/drive.hpp"
#include "test_support/mock_imu.hpp"
#include "test_support/mock_kinematics.hpp"
#include "test_support/mock_odometry.hpp"

using libstp::autotune::EncoderCalConfig;
using libstp::autotune::EncoderCalibrator;
using libstp::test::MockIMU;
using libstp::test::MockKinematics;
using libstp::test::MockOdometry;
using testing::NiceMock;
using testing::Return;

TEST(EncoderCalibratorTest, NoMotorsOnDriveFailsGracefully)
{
    // Build a kinematics mock that exposes zero motors. Drive takes
    // ownership of a unique_ptr<IKinematics>; allocate one explicitly.
    auto* kin_raw = new NiceMock<MockKinematics>();
    kin_raw->setupAsDifferential();
    ON_CALL(*kin_raw, getMotors())
        .WillByDefault(Return(std::vector<libstp::hal::motor::IMotor*>{}));

    NiceMock<MockIMU> imu;
    imu.setupDefaults();

    libstp::drive::ChassisVelocityControlConfig vel_cfg;
    libstp::drive::Drive drive(
        std::unique_ptr<libstp::kinematics::IKinematics>(kin_raw),
        vel_cfg,
        imu);

    NiceMock<MockOdometry> odom;
    odom.setupDefaults();

    EncoderCalibrator cal(drive, odom);

    EncoderCalConfig cfg;
    auto result = cal.calibrate(cfg);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.failure_reason, "no motors on drive");
}

TEST(EncoderCalibratorTest, ResultDefaultsAreSane)
{
    // White-box sanity check on the result struct's defaults — these
    // defaults are what callers see when calibrate() bails out early.
    libstp::autotune::EncoderCalResult r;
    EXPECT_FALSE(r.success);
    EXPECT_TRUE(r.failure_reason.empty());
    EXPECT_EQ(r.imu_total_angle_rad, 0.0);
    EXPECT_EQ(r.odom_total_angle_rad, 0.0);
    for (double t : r.ticks_to_rad)  EXPECT_EQ(t, 0.0);
    for (double s : r.scale_factors) EXPECT_EQ(s, 0.0);
}
