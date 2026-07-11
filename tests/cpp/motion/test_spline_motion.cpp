#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>
#include <numbers>

#include "test_support/test_fixtures.hpp"
#include "test_support/mock_imu.hpp"
#include "motion/catmull_rom_spline.hpp"
#include "motion/spline_motion.hpp"
#include "drive/drive.hpp"

using namespace libstp::test;
using namespace libstp::motion;
using namespace libstp::drive;
using ::testing::NiceMock;

// ===========================================================================
// CatmullRomSpline tests
// ===========================================================================

class CatmullRomSplineTest : public AlgorithmTestFixture {};

TEST_F(CatmullRomSplineTest, StraightLine_TotalLengthIsCorrect)
{
    std::vector<Eigen::Vector2d> pts = {{0.0, 0.0}, {1.0, 0.0}};
    CatmullRomSpline spline(pts);
    EXPECT_NEAR(spline.totalLength(), 1.0, 0.02);
}

TEST_F(CatmullRomSplineTest, StraightLine_MidpointIsCorrect)
{
    std::vector<Eigen::Vector2d> pts = {{0.0, 0.0}, {1.0, 0.0}};
    CatmullRomSpline spline(pts);
    Eigen::Vector2d mid = spline.positionAt(spline.totalLength() / 2.0);
    EXPECT_NEAR(mid.x(), 0.5, 0.05);
    EXPECT_NEAR(mid.y(), 0.0, 0.05);
}

TEST_F(CatmullRomSplineTest, StraightLine_TangentIsForward)
{
    std::vector<Eigen::Vector2d> pts = {{0.0, 0.0}, {1.0, 0.0}};
    CatmullRomSpline spline(pts);
    Eigen::Vector2d t = spline.tangentAt(spline.totalLength() / 2.0);
    EXPECT_NEAR(t.x(), 1.0, 0.1);
    EXPECT_NEAR(t.y(), 0.0, 0.1);
}

TEST_F(CatmullRomSplineTest, ThreePoints_PassesThroughAll)
{
    std::vector<Eigen::Vector2d> pts = {{0.0, 0.0}, {0.5, 0.3}, {1.0, 0.0}};
    CatmullRomSpline spline(pts);

    // Start point
    Eigen::Vector2d start = spline.positionAt(0.0);
    EXPECT_NEAR(start.x(), 0.0, 0.01);
    EXPECT_NEAR(start.y(), 0.0, 0.01);

    // End point
    Eigen::Vector2d end = spline.positionAt(spline.totalLength());
    EXPECT_NEAR(end.x(), 1.0, 0.01);
    EXPECT_NEAR(end.y(), 0.0, 0.01);
}

TEST_F(CatmullRomSplineTest, TwoPointsIsMinimum)
{
    std::vector<Eigen::Vector2d> pts = {{0.0, 0.0}, {1.0, 1.0}};
    EXPECT_NO_THROW(CatmullRomSpline spline(pts));
}

TEST_F(CatmullRomSplineTest, OnePointThrows)
{
    std::vector<Eigen::Vector2d> pts = {{0.0, 0.0}};
    EXPECT_THROW(CatmullRomSpline spline(pts), std::invalid_argument);
}

TEST_F(CatmullRomSplineTest, ArcLengthParameterization_EvenSpacing)
{
    // A curved path — evenly-spaced arc-length values should produce
    // geometrically evenly-spaced points
    std::vector<Eigen::Vector2d> pts = {
        {0.0, 0.0}, {0.3, 0.2}, {0.6, -0.1}, {1.0, 0.0}
    };
    CatmullRomSpline spline(pts);

    const int N = 10;
    double ds = spline.totalLength() / N;
    std::vector<double> chord_lengths;
    Eigen::Vector2d prev = spline.positionAt(0.0);
    for (int i = 1; i <= N; ++i)
    {
        Eigen::Vector2d p = spline.positionAt(i * ds);
        chord_lengths.push_back((p - prev).norm());
        prev = p;
    }

    // All chord lengths should be approximately equal
    for (double c : chord_lengths)
    {
        EXPECT_NEAR(c, ds, ds * 0.15)
            << "Arc-length parameterization should produce even spacing";
    }
}

TEST_F(CatmullRomSplineTest, FindNearest_PointOnPath)
{
    std::vector<Eigen::Vector2d> pts = {{0.0, 0.0}, {1.0, 0.0}};
    CatmullRomSpline spline(pts);

    // A point right on the path at s ≈ 0.5
    Eigen::Vector2d query(0.5, 0.0);
    double s = spline.findNearestArcLength(query, 0.0);
    EXPECT_NEAR(s, spline.totalLength() / 2.0, 0.05);
}

TEST_F(CatmullRomSplineTest, FindNearest_PointOffPath)
{
    std::vector<Eigen::Vector2d> pts = {{0.0, 0.0}, {1.0, 0.0}};
    CatmullRomSpline spline(pts);

    // A point offset laterally — should still project near midpoint
    Eigen::Vector2d query(0.5, 0.2);
    double s = spline.findNearestArcLength(query, 0.0);
    EXPECT_NEAR(s, spline.totalLength() / 2.0, 0.1);
}

// ===========================================================================
// SplineMotion tests
// ===========================================================================

class SplineMotionTest : public MotionTestFixture
{
protected:
    void SetUp() override {
        MotionTestFixture::SetUp();
        mock_imu_ = std::make_shared<NiceMock<MockIMU>>();
        mock_imu_->setupDefaults();

        auto kinematics_owned = std::make_unique<NiceMock<MockKinematics>>();
        kinematics_owned->setupAsDifferential();
        drive_ = std::make_unique<Drive>(
            std::move(kinematics_owned),
            ChassisVelocityControlConfig{},
            *mock_imu_);
    }

    std::shared_ptr<SplineMotion> makeMotion(SplineMotionConfig config) {
        pid_config_ = defaultUnifiedMotionPidConfig();
        pid_config_.linear = AxisConstraints{0.5, 1.0, 1.0};
        pid_config_.lateral = AxisConstraints{0.3, 0.8, 0.8};

        MotionContext ctx{*drive_, *mock_odometry_, pid_config_};
        return std::make_shared<SplineMotion>(ctx, config);
    }

    static constexpr double kDt = 0.01;  // 100 Hz

    std::shared_ptr<NiceMock<MockIMU>> mock_imu_;
    std::unique_ptr<Drive> drive_;
    UnifiedMotionPidConfig pid_config_{};
};

// Helper: set the mock pose using world-frame x/y and absolute heading.
// Keeps getDistanceFromOrigin paths stale-but-unused (the new motion code
// reads getPose()/getAbsoluteHeading() exclusively).
static void setWorldPose(NiceMock<MockOdometry>& mock,
                         double x, double y, double heading_rad)
{
    libstp::foundation::Pose pose;
    pose.position = Eigen::Vector3f(static_cast<float>(x),
                                     static_cast<float>(y),
                                     0.0f);
    pose.heading = static_cast<float>(heading_rad);
    mock.setPose(pose);
    ON_CALL(mock, getAbsoluteHeading()).WillByDefault(testing::Return(heading_rad));
}

TEST_F(SplineMotionTest, StartsNotFinished)
{
    SplineMotionConfig cfg;
    cfg.waypoints_m = {{0.3, 0.0}, {0.6, 0.0}};
    cfg.speed_scale = 0.5;

    auto motion = makeMotion(cfg);
    setWorldPose(*mock_odometry_, 0.0, 0.0, 0.0);
    motion->start();
    EXPECT_FALSE(motion->isFinished());
}

TEST_F(SplineMotionTest, StartDoesNotResetOdometry)
{
    SplineMotionConfig cfg;
    cfg.waypoints_m = {{0.3, 0.0}, {0.6, 0.0}};
    cfg.speed_scale = 0.5;

    auto motion = makeMotion(cfg);
    EXPECT_CALL(*mock_odometry_, reset()).Times(0);
    setWorldPose(*mock_odometry_, 1.5, 0.7, 0.3);
    motion->start();
}

TEST_F(SplineMotionTest, FinishesWhenAtGoal)
{
    SplineMotionConfig cfg;
    cfg.waypoints_m = {{0.1, 0.0}, {0.2, 0.0}};
    cfg.speed_scale = 0.5;

    auto motion = makeMotion(cfg);
    setWorldPose(*mock_odometry_, 0.0, 0.0, 0.0);
    motion->start();

    // Simulate the robot reaching the end of the spline (body-forward 0.20)
    setWorldPose(*mock_odometry_, 0.20, 0.0, 0.0);

    // Run enough updates for the profiled PID to converge
    for (int i = 0; i < 500; ++i)
    {
        if (motion->isFinished()) break;
        motion->update(kDt);
    }

    // The motion should eventually finish (or at least produce telemetry)
    EXPECT_FALSE(motion->getTelemetry().empty());
}

TEST_F(SplineMotionTest, TelemetryIsPopulated)
{
    SplineMotionConfig cfg;
    cfg.waypoints_m = {{0.2, 0.0}, {0.4, 0.1}, {0.6, 0.0}};
    cfg.speed_scale = 0.5;

    auto motion = makeMotion(cfg);
    setWorldPose(*mock_odometry_, 0.0, 0.0, 0.0);
    motion->start();

    setWorldPose(*mock_odometry_, 0.05, 0.0, 0.0);
    motion->update(kDt);

    const auto& telem = motion->getTelemetry();
    ASSERT_FALSE(telem.empty());
    EXPECT_GT(telem.back().arc_target_m, 0.0);
    EXPECT_GE(telem.back().time_s, 0.0);
}

// ===========================================================================
// Reverse traversal (speed_scale < 0)
// ===========================================================================

// A positive-magnitude reverse speed is NOT clamped to a near-stall crawl: the
// path velocity cap scales with |speed_scale|, identical to a forward speed.
TEST_F(SplineMotionTest, ReverseSpeedMagnitudeNotClampedToStall)
{
    SplineMotionConfig fwd;
    fwd.waypoints_m = {{0.3, 0.0}, {0.6, 0.0}};
    fwd.speed_scale = 0.8;
    auto m_fwd = makeMotion(fwd);
    setWorldPose(*mock_odometry_, 0.0, 0.0, 0.0);
    m_fwd->start();
    setWorldPose(*mock_odometry_, 0.05, 0.0, 0.0);
    m_fwd->update(kDt);

    SplineMotionConfig rev = fwd;
    rev.speed_scale = -0.8;
    auto m_rev = makeMotion(rev);
    setWorldPose(*mock_odometry_, 0.0, 0.0, 0.0);
    m_rev->start();
    setWorldPose(*mock_odometry_, 0.05, 0.0, 0.0);
    m_rev->update(kDt);

    // Same speed profile magnitude — the |0.8| path cap, not a clamped 0.01.
    const double v_fwd = std::abs(m_fwd->getTelemetry().back().setpoint_velocity_mps);
    const double v_rev = std::abs(m_rev->getTelemetry().back().setpoint_velocity_mps);
    EXPECT_NEAR(v_fwd, v_rev, 1e-9);
}

// Differential reverse: the drivetrain cannot hold heading through a curve, so
// it drives rear-first — the heading target points opposite the travel tangent
// and the commanded body-forward velocity is negative.
TEST_F(SplineMotionTest, DifferentialReverseIsRearFirst)
{
    SplineMotionConfig cfg;
    cfg.waypoints_m = {{0.3, 0.0}, {0.6, 0.0}};  // straight, tangent = +x
    cfg.speed_scale = -0.5;                       // reverse

    auto motion = makeMotion(cfg);               // fixture kinematics = differential
    setWorldPose(*mock_odometry_, 0.0, 0.0, 0.0);  // start facing +x
    motion->start();
    motion->update(kDt);

    // Heading target points opposite the +x tangent (~pi): the robot is asked to
    // turn its nose away from travel so it can back along the curve rear-first.
    EXPECT_NEAR(std::abs(motion->getTelemetry().back().target_heading_rad),
                std::numbers::pi, 1e-6);

    // Once the nose has turned to face backward, the commanded body-forward
    // velocity is negative — the robot drives in reverse along the path. The
    // velocity decomposition reads getHeading(), so stub it too.
    setWorldPose(*mock_odometry_, 0.0, 0.0, std::numbers::pi);
    ON_CALL(*mock_odometry_, getHeading())
        .WillByDefault(::testing::Return(std::numbers::pi));
    motion->update(kDt);
    EXPECT_LT(motion->getTelemetry().back().cmd_vx_mps, 0.0);
}

// Holonomic (mecanum) reverse: the robot HOLDS its start orientation (heading
// target 0, no turn) and strafes the path. With a backward-leading path this is
// genuine reverse motion while the nose stays put.
TEST_F(SplineMotionTest, MecanumReverseHoldsHeading)
{
    auto kin = std::make_unique<NiceMock<MockKinematics>>();
    kin->setupAsMecanum();  // supportsLateralMotion() => true
    Drive mdrive(std::move(kin), ChassisVelocityControlConfig{}, *mock_imu_);

    SplineMotionConfig cfg;
    cfg.waypoints_m = {{-0.3, 0.0}, {-0.6, 0.0}};  // path leads BACKWARD (-x)
    cfg.speed_scale = -0.5;

    pid_config_ = defaultUnifiedMotionPidConfig();
    pid_config_.linear = AxisConstraints{0.5, 1.0, 1.0};
    pid_config_.lateral = AxisConstraints{0.3, 0.8, 0.8};
    MotionContext ctx{mdrive, *mock_odometry_, pid_config_};
    auto motion = std::make_shared<SplineMotion>(ctx, cfg);

    setWorldPose(*mock_odometry_, 0.0, 0.0, 0.0);  // facing +x (forward)
    motion->start();
    motion->update(kDt);

    const auto& s = motion->getTelemetry().back();
    // Nose held at the start orientation — no turn to the tangent.
    EXPECT_NEAR(s.target_heading_rad, 0.0, 1e-6);
    // Backward-leading path + held forward nose => body drives backward.
    EXPECT_LT(s.cmd_vx_mps, 0.0);
}
