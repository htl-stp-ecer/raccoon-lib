#include <gtest/gtest.h>
#include "drive/rate_limiter.hpp"
#include "test_support/test_fixtures.hpp"

using namespace libstp::drive;
using namespace libstp::test;

class RateLimiterTest : public AlgorithmTestFixture {};

TEST_F(RateLimiterTest, NoLimitWhenZeroMaxRate) {
    RateLimiter limiter(0.0);  // No limit

    double accel = 0.0;
    double output = limiter.step(100.0, 0.0, kDefaultDt, accel);
    EXPECT_NEAR(output, 100.0, kTolerance);  // Should pass through
}

TEST_F(RateLimiterTest, LimitsAcceleration) {
    RateLimiter limiter(10.0);  // Max rate of 10 units/second

    // Starting from 0, requesting 100 with dt=0.1
    // Max change = 10.0 * 0.1 = 1.0
    double accel = 0.0;
    double output = limiter.step(100.0, 0.0, 0.1, accel);
    EXPECT_NEAR(output, 1.0, kTolerance);  // Limited to max change
}

TEST_F(RateLimiterTest, LimitsDeceleration) {
    RateLimiter limiter(10.0);

    // Get to a high value first
    double current = 0.0;
    double accel = 0.0;
    for (int i = 0; i < 100; ++i) {
        current = limiter.step(100.0, current, 0.1, accel);
    }

    // Now request 0 - should decelerate at max rate
    double prev = current;
    double output = limiter.step(0.0, prev, 0.1, accel);

    EXPECT_LT(output, prev);  // Should be decreasing
    EXPECT_GE(output, prev - 1.0 - kTolerance);  // At most 1 unit decrease
}

TEST_F(RateLimiterTest, SetMaxRate) {
    RateLimiter limiter(10.0);

    limiter.setMaxRate(5.0);

    // Max change should now be 5.0 * 0.1 = 0.5
    double accel = 0.0;
    double output = limiter.step(100.0, 0.0, 0.1, accel);
    EXPECT_NEAR(output, 0.5, kTolerance);
}
