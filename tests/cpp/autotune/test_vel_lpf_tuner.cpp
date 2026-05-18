// Unit tests for libstp::autotune::VelLpfTuner.
//
// The tuner samples getBemf() while the motor is quiescent and scores
// candidate alphas based on filtered variance vs lag-change-rate. We
// drive the mock with simple deterministic streams to validate behavior.
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include "autotune/vel_lpf_tune.hpp"
#include "autotune/types.hpp"
#include "test_support/mock_motor.hpp"

using libstp::autotune::VelLpfConfig;
using libstp::autotune::VelLpfTuner;
using libstp::test::MockMotor;
using testing::_;
using testing::NiceMock;
using testing::Return;

namespace
{

VelLpfConfig makeFastConfig()
{
    VelLpfConfig cfg;
    cfg.alpha_min          = 0.1;
    cfg.alpha_max          = 0.9;
    cfg.alpha_step         = 0.1;
    cfg.measure_duration_s = 0.05;  // 50ms
    cfg.sample_hz          = 200;   // ~10 samples
    return cfg;
}

} // namespace

TEST(VelLpfTunerTest, RunsWithoutCrashAndApplies)
{
    NiceMock<MockMotor> motor(/*port=*/0);
    libstp::foundation::MotorCalibration cal{};
    cal.vel_lpf_alpha = 0.5;
    motor.setCalibration(cal);

    // Quiet motor: low constant BEMF noise.
    int counter = 0;
    ON_CALL(motor, getBemf())
        .WillByDefault([&counter]() -> int {
            return (counter++ % 3 == 0) ? 1 : 0;
        });

    VelLpfTuner tuner({&motor});
    auto result = tuner.tuneMotor(&motor, makeFastConfig());

    EXPECT_TRUE(result.applied);
    EXPECT_EQ(result.motor_port, 0);
}

TEST(VelLpfTunerTest, TunedAlphaInValidRange)
{
    NiceMock<MockMotor> motor(/*port=*/0);
    libstp::foundation::MotorCalibration cal{};
    motor.setCalibration(cal);

    int n = 0;
    ON_CALL(motor, getBemf())
        .WillByDefault([&n]() -> int { return (n++ & 1) ? 0 : 1; });

    VelLpfTuner tuner({&motor});
    auto cfg    = makeFastConfig();
    auto result = tuner.tuneMotor(&motor, cfg);

    EXPECT_GT(result.tuned_alpha, 0.0);
    EXPECT_LE(result.tuned_alpha, 1.0);
    // Must lie within the requested sweep range (modulo clamp at the
    // top of the source).
    EXPECT_GE(result.tuned_alpha, cfg.alpha_min - 1e-9);
    EXPECT_LE(result.tuned_alpha, cfg.alpha_max + 1e-9);
}

TEST(VelLpfTunerTest, CalibrationUpdatedOnMotor)
{
    NiceMock<MockMotor> motor(/*port=*/2);
    libstp::foundation::MotorCalibration cal{};
    cal.vel_lpf_alpha = 0.5;
    motor.setCalibration(cal);

    int n = 0;
    ON_CALL(motor, getBemf())
        .WillByDefault([&n]() -> int { return (n++ % 5 == 0) ? 2 : 0; });

    VelLpfTuner tuner({&motor});
    auto result = tuner.tuneMotor(&motor, makeFastConfig());

    ASSERT_TRUE(result.applied);
    // After tuning, the motor's calibration's alpha should match the
    // tuned alpha value (or be clamped into [0, 1]).
    const auto& updated = motor.getCalibration();
    EXPECT_NEAR(updated.vel_lpf_alpha, result.tuned_alpha, 1e-9);
}

TEST(VelLpfTunerTest, NullMotorReturnsDefaultResult)
{
    VelLpfTuner tuner({});
    auto result = tuner.tuneMotor(nullptr, makeFastConfig());
    EXPECT_FALSE(result.applied);
    EXPECT_EQ(result.motor_port, -1);
}

TEST(VelLpfTunerTest, SmokeTestMultiMotor)
{
    NiceMock<MockMotor> a(/*port=*/0);
    NiceMock<MockMotor> b(/*port=*/1);
    libstp::foundation::MotorCalibration cal{};
    a.setCalibration(cal);
    b.setCalibration(cal);

    ON_CALL(a, getBemf()).WillByDefault(Return(0));
    ON_CALL(b, getBemf()).WillByDefault(Return(1));

    VelLpfTuner tuner({&a, &b});
    auto results = tuner.tune(makeFastConfig());

    ASSERT_EQ(results.size(), 2u);
    EXPECT_TRUE(results[0].applied);
    EXPECT_TRUE(results[1].applied);
}
