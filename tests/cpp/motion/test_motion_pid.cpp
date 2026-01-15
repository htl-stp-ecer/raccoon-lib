#include <gtest/gtest.h>
#include "motion/motion_pid.hpp"
#include "test_support/test_fixtures.hpp"

using namespace libstp::motion;
using namespace libstp::test;

class MotionPidTest : public AlgorithmTestFixture {};

TEST_F(MotionPidTest, ProportionalOnly) {
    MotionPidController::Config cfg{
        .kp = 2.0,
        .ki = 0.0,
        .kd = 0.0,
        .output_min = -100.0,
        .output_max = 100.0
    };
    MotionPidController pid(cfg);

    double output = pid.update(1.0, kDefaultDt);
    EXPECT_NEAR(output, 2.0, kTolerance);  // kp * error
}

TEST_F(MotionPidTest, IntegralAccumulates) {
    MotionPidController::Config cfg{
        .kp = 0.0,
        .ki = 1.0,
        .kd = 0.0,
        .integral_max = 100.0,
        .integral_deadband = 0.0,  // No deadband
        .output_min = -100.0,
        .output_max = 100.0
    };
    MotionPidController pid(cfg);

    // First update starts integrating
    pid.update(1.0, 0.1);  // integral = 1.0 * 0.1 = 0.1
    double output = pid.update(1.0, 0.1);  // integral = 0.1 + 0.1 = 0.2

    EXPECT_NEAR(output, 0.2, kTolerance);  // ki * integral
    EXPECT_NEAR(pid.getIntegral(), 0.2, kTolerance);
}

TEST_F(MotionPidTest, IntegralAntiWindup) {
    MotionPidController::Config cfg{
        .kp = 0.0,
        .ki = 1.0,
        .kd = 0.0,
        .integral_max = 0.5,  // Clamp at 0.5
        .integral_deadband = 0.0,
        .output_min = -100.0,
        .output_max = 100.0
    };
    MotionPidController pid(cfg);

    // Apply large error for many iterations
    for (int i = 0; i < 100; ++i) {
        pid.update(10.0, 0.1);  // Would accumulate to 100 without anti-windup
    }

    EXPECT_NEAR(pid.getIntegral(), 0.5, kTolerance);  // Clamped to max
}

TEST_F(MotionPidTest, IntegralDeadband) {
    MotionPidController::Config cfg{
        .kp = 0.0,
        .ki = 1.0,
        .kd = 0.0,
        .integral_max = 100.0,
        .integral_deadband = 0.1,  // Don't integrate if |error| < 0.1
        .output_min = -100.0,
        .output_max = 100.0
    };
    MotionPidController pid(cfg);

    // Error within deadband - should not integrate
    for (int i = 0; i < 10; ++i) {
        pid.update(0.05, 0.1);
    }

    EXPECT_NEAR(pid.getIntegral(), 0.0, kTolerance);

    // Error outside deadband - should integrate
    pid.update(0.5, 0.1);
    EXPECT_GT(pid.getIntegral(), 0.0);
}

TEST_F(MotionPidTest, DerivativeFiltering) {
    MotionPidController::Config cfg{
        .kp = 0.0,
        .ki = 0.0,
        .kd = 1.0,
        .derivative_lpf_alpha = 0.5,  // 50% filtering
        .output_min = -100.0,
        .output_max = 100.0
    };
    MotionPidController pid(cfg);

    // First update establishes baseline
    pid.update(0.0, 0.1);

    // Step change in error - derivative should be filtered
    pid.update(1.0, 0.1);  // Raw derivative = (1.0 - 0.0) / 0.1 = 10.0

    // Filtered derivative should be less than raw due to LPF
    double filtered = pid.getDerivative();
    EXPECT_GT(filtered, 0.0);  // Should be positive
    EXPECT_LT(filtered, 10.0); // Should be filtered below raw
}

TEST_F(MotionPidTest, OutputSaturation) {
    MotionPidController::Config cfg{
        .kp = 100.0,  // Very high gain
        .ki = 0.0,
        .kd = 0.0,
        .output_min = -5.0,
        .output_max = 5.0
    };
    MotionPidController pid(cfg);

    // Large positive error
    double output = pid.update(1.0, kDefaultDt);
    EXPECT_NEAR(output, 5.0, kTolerance);  // Clamped to max

    // Large negative error
    output = pid.update(-1.0, kDefaultDt);
    EXPECT_NEAR(output, -5.0, kTolerance);  // Clamped to min
}

TEST_F(MotionPidTest, Reset) {
    MotionPidController::Config cfg{
        .kp = 1.0,
        .ki = 1.0,
        .kd = 1.0,
        .output_min = -100.0,
        .output_max = 100.0
    };
    MotionPidController pid(cfg);

    // Build up some state
    for (int i = 0; i < 10; ++i) {
        pid.update(1.0, 0.1);
    }

    EXPECT_GT(pid.getIntegral(), 0.0);

    // Reset should clear state
    pid.reset();

    EXPECT_NEAR(pid.getIntegral(), 0.0, kTolerance);
    EXPECT_NEAR(pid.getDerivative(), 0.0, kTolerance);
}

TEST_F(MotionPidTest, ZeroDtReturnsZero) {
    MotionPidController::Config cfg{
        .kp = 1.0,
        .ki = 1.0,
        .kd = 1.0,
        .output_min = -100.0,
        .output_max = 100.0
    };
    MotionPidController pid(cfg);

    // Zero dt should return 0 to avoid division issues
    double output = pid.update(1.0, 0.0);
    EXPECT_NEAR(output, 0.0, kTolerance);
}

TEST_F(MotionPidTest, NegativeDtReturnsZero) {
    MotionPidController::Config cfg{
        .kp = 1.0,
        .ki = 1.0,
        .kd = 1.0,
        .output_min = -100.0,
        .output_max = 100.0
    };
    MotionPidController pid(cfg);

    // Negative dt should return 0
    double output = pid.update(1.0, -0.1);
    EXPECT_NEAR(output, 0.0, kTolerance);
}

TEST_F(MotionPidTest, SetGains) {
    MotionPidController::Config cfg{
        .kp = 1.0,
        .ki = 0.0,
        .kd = 0.0,
        .output_min = -100.0,
        .output_max = 100.0
    };
    MotionPidController pid(cfg);

    // Original gain
    double output1 = pid.update(1.0, kDefaultDt);
    EXPECT_NEAR(output1, 1.0, kTolerance);

    // Change gains
    pid.reset();
    pid.setGains(5.0, 0.0, 0.0);

    double output2 = pid.update(1.0, kDefaultDt);
    EXPECT_NEAR(output2, 5.0, kTolerance);
}
