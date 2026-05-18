// Unit tests for libstp::autotune::FirmwarePidTuner.
//
// The tuner runs real wall-clock step responses (sleep_for) — we keep
// step_duration_s and sample_hz low so each test stays sub-second.
// We simulate a first-order BEMF response shared between setVelocity()
// (which resets t_start) and getBemf() (which samples the curve).
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <chrono>
#include <cmath>
#include <memory>

#include "autotune/firmware_pid_tune.hpp"
#include "autotune/types.hpp"
#include "test_support/mock_motor.hpp"

using libstp::autotune::FirmwarePidConfig;
using libstp::autotune::FirmwarePidTuner;
using libstp::test::MockMotor;
using testing::_;
using testing::NiceMock;
using testing::Return;

namespace
{

struct SimState
{
    std::chrono::steady_clock::time_point t_start{};
    double steady_state{0.0};
    double tau{0.02};
    bool   active{false};
};

// Attach a stateful first-order BEMF response model to a NiceMock<MockMotor>.
// setVelocity() restarts the response clock; getBemf() returns the sampled
// value. The shared_ptr keeps the state alive for the lifetime of the test.
std::shared_ptr<SimState> attachFirstOrder(NiceMock<MockMotor>& motor,
                                           double steady_state,
                                           double tau)
{
    auto state = std::make_shared<SimState>();
    state->steady_state = steady_state;
    state->tau          = (tau > 1e-6) ? tau : 1e-6;
    state->active       = true;

    ON_CALL(motor, setVelocity(_))
        .WillByDefault([state](int /*v*/) {
            state->t_start = std::chrono::steady_clock::now();
        });
    ON_CALL(motor, setSpeed(_))
        .WillByDefault([state](int /*pct*/) {
            state->t_start = std::chrono::steady_clock::now();
        });
    ON_CALL(motor, getBemf())
        .WillByDefault([state]() -> int {
            if (!state->active) return 0;
            const auto now = std::chrono::steady_clock::now();
            const double t = std::chrono::duration<double>(
                now - state->t_start).count();
            const double v = state->steady_state *
                             (1.0 - std::exp(-t / state->tau));
            return static_cast<int>(v);
        });
    return state;
}

FirmwarePidConfig makeFastConfig()
{
    FirmwarePidConfig cfg;
    cfg.step_duration_s   = 0.1;   // 100ms per response
    cfg.decel_duration_s  = 0.02;
    cfg.sample_hz         = 50;    // ~5 samples
    cfg.step_fraction     = 0.5;
    cfg.min_response_frac = 0.05;  // accept small responses for tests
    cfg.identification_trials = 1;
    cfg.validation_trials = 1;
    return cfg;
}

} // namespace

TEST(FirmwarePidTunerTest, NullMotorReturnsDefaultResult)
{
    FirmwarePidTuner tuner({});
    auto result = tuner.tuneMotor(nullptr, /*max_bemf=*/100, makeFastConfig());
    EXPECT_EQ(result.motor_port, -1);
    EXPECT_FALSE(result.accepted);
}

TEST(FirmwarePidTunerTest, MotorWithNoResponseRejected)
{
    NiceMock<MockMotor> motor(/*port=*/3);
    libstp::foundation::MotorCalibration cal{};
    motor.setCalibration(cal);
    ON_CALL(motor, getBemf()).WillByDefault(Return(0));

    FirmwarePidTuner tuner({&motor});
    auto result = tuner.tuneMotor(&motor, /*max_bemf=*/100, makeFastConfig());

    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.plant.method, "insufficient_response");
    EXPECT_EQ(result.motor_port, 3);
}

TEST(FirmwarePidTunerTest, MotorWithSimulatedResponseRunsWithoutCrash)
{
    NiceMock<MockMotor> motor(/*port=*/0);
    libstp::foundation::MotorCalibration cal{};
    motor.setCalibration(cal);
    auto sim = attachFirstOrder(motor, /*steady_state=*/45.0, /*tau=*/0.02);
    (void)sim;

    FirmwarePidTuner tuner({&motor});
    auto result = tuner.tuneMotor(&motor, /*max_bemf=*/100, makeFastConfig());

    EXPECT_EQ(result.motor_port, 0);
    // We don't make a hard claim on accepted/rejected — the deterministic
    // synthetic response may produce identical baseline/tuned ISE and end
    // up reverted. But the call must not crash and a method should be set.
    EXPECT_FALSE(result.plant.method.empty());
}

TEST(FirmwarePidTunerTest, TuneMultiMotorSkipsMissingPort)
{
    NiceMock<MockMotor> m0(/*port=*/0);
    NiceMock<MockMotor> m1(/*port=*/1);
    libstp::foundation::MotorCalibration cal{};
    m0.setCalibration(cal);
    m1.setCalibration(cal);
    ON_CALL(m0, getBemf()).WillByDefault(Return(0));
    ON_CALL(m1, getBemf()).WillByDefault(Return(0));

    FirmwarePidTuner tuner({&m0, &m1});
    std::map<int, int> max_bemf_speeds;
    max_bemf_speeds[0] = 100;  // only port 0
    auto results = tuner.tune(max_bemf_speeds, makeFastConfig());

    ASSERT_EQ(results.size(), 1u);
    EXPECT_TRUE(results.count(0));
    EXPECT_FALSE(results.count(1));
}

TEST(FirmwarePidTunerTest, GainsRevertedIfNotImproved)
{
    // With a deterministic synthetic response that depends only on time
    // since setVelocity(), baseline and tuned step responses are
    // identical → tuned_ise == baseline_ise → reject path runs and the
    // tuner calls setFirmwarePidGains with the starter values that were used
    // to get a measurable MAV-mode baseline.
    NiceMock<MockMotor> motor(/*port=*/0);
    libstp::foundation::MotorCalibration cal{};
    motor.setCalibration(cal);
    auto sim = attachFirstOrder(motor, /*steady_state=*/45.0, /*tau=*/0.02);
    (void)sim;

    FirmwarePidTuner tuner({&motor});
    auto result = tuner.tuneMotor(&motor, /*max_bemf=*/100, makeFastConfig());

    if (!result.accepted) {
        float kp = -1.0f, ki = -1.0f, kd = -1.0f;
        motor.getLastFirmwarePidGains(kp, ki, kd);
        EXPECT_FLOAT_EQ(kp, 0.3f);
        EXPECT_FLOAT_EQ(ki, 2.0f);
        EXPECT_FLOAT_EQ(kd, 0.0f);
    } else {
        // If by chance ISE improved, just verify the gains stored match
        // the tuner's reported gains.
        float kp = 0, ki = 0, kd = 0;
        motor.getLastFirmwarePidGains(kp, ki, kd);
        EXPECT_FLOAT_EQ(kp, result.kp);
        EXPECT_FLOAT_EQ(ki, result.ki);
        EXPECT_FLOAT_EQ(kd, result.kd);
    }
}
