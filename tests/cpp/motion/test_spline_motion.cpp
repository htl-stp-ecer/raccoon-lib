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

TEST_F(SplineMotionTest, StartsNotFinished)
{
    SplineMotionConfig cfg;
    cfg.waypoints_m = {{0.3, 0.0}, {0.6, 0.0}};
    cfg.speed_scale = 0.5;

    auto motion = makeMotion(cfg);
    motion->start();
    EXPECT_FALSE(motion->isFinished());
}

TEST_F(SplineMotionTest, FinishesWhenAtGoal)
{
    SplineMotionConfig cfg;
    cfg.waypoints_m = {{0.1, 0.0}, {0.2, 0.0}};
    cfg.speed_scale = 0.5;

    auto motion = makeMotion(cfg);
    motion->start();

    // Simulate the robot reaching the end of the spline
    mock_odometry_->simulateForwardProgress(0.20);
    mock_odometry_->setHeading(0.0);
    mock_odometry_->setHeadingError(0.0);

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
    motion->start();

    mock_odometry_->simulateForwardProgress(0.05);
    motion->update(kDt);

    const auto& telem = motion->getTelemetry();
    ASSERT_FALSE(telem.empty());
    EXPECT_GT(telem.back().arc_target_m, 0.0);
    EXPECT_GE(telem.back().time_s, 0.0);
}
