// Unit tests for libstp::autotune::StaticFrictionMeasurer.
//
// We simulate a motor that only "moves" (returns BEMF >= motion_threshold)
// once setSpeed() has been called with |percent| >= some configurable
// stiction PWM. Shared state between setSpeed and getBemf uses a
// std::shared_ptr captured by the ON_CALL lambdas.
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <cstdlib>
#include <memory>

#include "autotune/static_friction.hpp"
#include "autotune/types.hpp"
#include "test_support/mock_motor.hpp"

using libstp::autotune::StaticFrictionConfig;
using libstp::autotune::StaticFrictionMeasurer;
using libstp::test::MockMotor;
using testing::_;
using testing::NiceMock;
using testing::Return;

namespace
{

// Configure a stiction model: motor returns BEMF >= threshold once
// |setSpeed| >= stiction_pct. Returns the shared state for inspection.
struct StictionState
{
    std::shared_ptr<int> current_pct;
};

StictionState attachStictionModel(NiceMock<MockMotor>& motor,
                                  int stiction_pct,
                                  int bemf_above = 20)
{
    auto current = std::make_shared<int>(0);
    ON_CALL(motor, setSpeed(_))
        .WillByDefault([current](int pct) { *current = pct; });
    ON_CALL(motor, setVelocity(_))
        .WillByDefault([current](int v) { *current = v; });
    ON_CALL(motor, getBemf())
        .WillByDefault([current, stiction_pct, bemf_above]() -> int {
            return (std::abs(*current) >= stiction_pct) ? bemf_above : 0;
        });
    // resetPositionCounter / brake / off are no-ops by default (NiceMock).
    return {current};
}

StaticFrictionConfig makeFastConfig()
{
    StaticFrictionConfig cfg;
    cfg.start_pct        = 1;
    cfg.max_pct          = 15;
    cfg.step_pct         = 1;
    cfg.dwell_ms         = 1;
    cfg.samples_per_step = 2;
    cfg.motion_threshold = 5;
    return cfg;
}

} // namespace

TEST(StaticFrictionTest, MeasuresKsPositiveCorrectly)
{
    NiceMock<MockMotor> motor(/*port=*/0);
    libstp::foundation::MotorCalibration cal{};
    motor.setCalibration(cal);
    attachStictionModel(motor, /*stiction_pct=*/6);

    StaticFrictionMeasurer m({&motor});
    auto result = m.measureMotor(&motor, makeFastConfig());

    EXPECT_TRUE(result.measured);
    EXPECT_EQ(result.motor_port, 0);
    // The sweep advances pct by 1 starting at 1; first match is at 6.
    EXPECT_EQ(result.ks_positive_pct, 6);
    EXPECT_EQ(result.ks_negative_pct, 6);
    EXPECT_EQ(result.ks_avg_pct, 6);
}

TEST(StaticFrictionTest, MeasuresBothDirections)
{
    NiceMock<MockMotor> motor(/*port=*/1);
    libstp::foundation::MotorCalibration cal{};
    motor.setCalibration(cal);
    attachStictionModel(motor, /*stiction_pct=*/4);

    StaticFrictionMeasurer m({&motor});
    auto result = m.measureMotor(&motor, makeFastConfig());

    EXPECT_TRUE(result.measured);
    EXPECT_GT(result.ks_positive_pct, 0);
    EXPECT_GT(result.ks_negative_pct, 0);
}

TEST(StaticFrictionTest, MotorNeverRespondsGivesUnmeasured)
{
    NiceMock<MockMotor> motor(/*port=*/2);
    libstp::foundation::MotorCalibration cal{};
    motor.setCalibration(cal);
    // Always-zero BEMF — motor never crosses threshold.
    ON_CALL(motor, getBemf()).WillByDefault(Return(0));

    StaticFrictionMeasurer m({&motor});
    auto result = m.measureMotor(&motor, makeFastConfig());

    EXPECT_FALSE(result.measured);
    EXPECT_EQ(result.ks_positive_pct, 0);
    EXPECT_EQ(result.ks_negative_pct, 0);
    EXPECT_EQ(result.ks_avg_pct, 0);
}

TEST(StaticFrictionTest, NullMotorReturnsDefaultResult)
{
    StaticFrictionMeasurer m({});
    auto result = m.measureMotor(nullptr, makeFastConfig());
    EXPECT_FALSE(result.measured);
    EXPECT_EQ(result.motor_port, -1);
}

TEST(StaticFrictionTest, MultiMotorMeasuresIndependently)
{
    NiceMock<MockMotor> a(/*port=*/0);
    NiceMock<MockMotor> b(/*port=*/1);
    libstp::foundation::MotorCalibration cal{};
    a.setCalibration(cal);
    b.setCalibration(cal);
    attachStictionModel(a, /*stiction_pct=*/3);
    attachStictionModel(b, /*stiction_pct=*/8);

    StaticFrictionMeasurer m({&a, &b});
    auto results = m.measure(makeFastConfig());

    ASSERT_EQ(results.size(), 2u);
    EXPECT_TRUE(results[0].measured);
    EXPECT_TRUE(results[1].measured);
    EXPECT_EQ(results[0].ks_positive_pct, 3);
    EXPECT_EQ(results[1].ks_positive_pct, 8);
}
