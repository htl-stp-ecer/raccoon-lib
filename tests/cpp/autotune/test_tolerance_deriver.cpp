// Unit tests for libstp::autotune::deriveTolerances — pure computation,
// no hardware interaction.
#include <gtest/gtest.h>

#include "autotune/tolerance_deriver.hpp"
#include "autotune/types.hpp"
#include "motion/motion_config.hpp"

using namespace libstp::autotune;

namespace
{
MotionTuneResult makeResult(const std::string& name, double final_score)
{
    MotionTuneResult r;
    r.param_name   = name;
    r.final_score  = final_score;
    return r;
}
} // namespace

TEST(ToleranceDeriverTest, EmptyResultsLeavesConfigUnchanged)
{
    std::map<std::string, MotionTuneResult> empty;
    libstp::motion::UnifiedMotionPidConfig config;
    const double init_dist  = config.distance_tolerance_m;
    const double init_angle = config.angle_tolerance_rad;

    MotionTuneConfig tune_cfg;
    ToleranceConfig  cfg;
    auto result = deriveTolerances(empty, tune_cfg, config, cfg);

    EXPECT_FALSE(result.distance_updated);
    EXPECT_FALSE(result.angle_updated);
    EXPECT_DOUBLE_EQ(config.distance_tolerance_m, init_dist);
    EXPECT_DOUBLE_EQ(config.angle_tolerance_rad, init_angle);
}

TEST(ToleranceDeriverTest, DistanceResultUpdatesTolerance)
{
    std::map<std::string, MotionTuneResult> results;
    // Use a moderate final_score so that scaled error sits inside the
    // configured [min, max] window (default min=0.005, max=0.050).
    // error_est = final_score / score_error_weight (default 5.0)
    //           = 0.10 / 5.0 = 0.02
    // scaled     = 0.02 * margin_factor(1.5) = 0.03  -> in range
    results["distance"] = makeResult("distance", 0.10);

    libstp::motion::UnifiedMotionPidConfig config;
    MotionTuneConfig tune_cfg;
    ToleranceConfig  cfg;
    auto result = deriveTolerances(results, tune_cfg, config, cfg);

    EXPECT_TRUE(result.distance_updated);
    EXPECT_FALSE(result.angle_updated);
    EXPECT_GE(config.distance_tolerance_m, cfg.min_distance_tolerance_m);
    EXPECT_LE(config.distance_tolerance_m, cfg.max_distance_tolerance_m);
    EXPECT_DOUBLE_EQ(config.distance_tolerance_m,
                     result.derived_distance_tolerance_m);
}

TEST(ToleranceDeriverTest, HeadingResultUpdatesAngleTolerance)
{
    std::map<std::string, MotionTuneResult> results;
    // error_est = 0.20 / 5.0 = 0.04 rad; scaled = 0.06 (within [0.017, 0.087])
    results["heading"] = makeResult("heading", 0.20);

    libstp::motion::UnifiedMotionPidConfig config;
    MotionTuneConfig tune_cfg;
    ToleranceConfig  cfg;
    auto result = deriveTolerances(results, tune_cfg, config, cfg);

    EXPECT_FALSE(result.distance_updated);
    EXPECT_TRUE(result.angle_updated);
    EXPECT_GE(config.angle_tolerance_rad, cfg.min_angle_tolerance_rad);
    EXPECT_LE(config.angle_tolerance_rad, cfg.max_angle_tolerance_rad);
    EXPECT_DOUBLE_EQ(config.angle_tolerance_rad,
                     result.derived_angle_tolerance_rad);
}

TEST(ToleranceDeriverTest, ToleranceClampedToBoundsLow)
{
    std::map<std::string, MotionTuneResult> results;
    // final_score = 0 -> error_est = 0 -> clamped to min.
    results["distance"] = makeResult("distance", 0.0);
    results["heading"]  = makeResult("heading", 0.0);

    libstp::motion::UnifiedMotionPidConfig config;
    MotionTuneConfig tune_cfg;
    ToleranceConfig  cfg;
    auto result = deriveTolerances(results, tune_cfg, config, cfg);

    EXPECT_TRUE(result.distance_updated);
    EXPECT_TRUE(result.angle_updated);
    EXPECT_DOUBLE_EQ(config.distance_tolerance_m, cfg.min_distance_tolerance_m);
    EXPECT_DOUBLE_EQ(config.angle_tolerance_rad, cfg.min_angle_tolerance_rad);
}

TEST(ToleranceDeriverTest, ToleranceClampedToBoundsHigh)
{
    std::map<std::string, MotionTuneResult> results;
    // Very large final_score → scaled error_est exceeds max, gets clamped.
    results["distance"] = makeResult("distance", 1e6);
    results["heading"]  = makeResult("heading", 1e6);

    libstp::motion::UnifiedMotionPidConfig config;
    MotionTuneConfig tune_cfg;
    ToleranceConfig  cfg;
    auto result = deriveTolerances(results, tune_cfg, config, cfg);

    EXPECT_TRUE(result.distance_updated);
    EXPECT_TRUE(result.angle_updated);
    EXPECT_DOUBLE_EQ(config.distance_tolerance_m, cfg.max_distance_tolerance_m);
    EXPECT_DOUBLE_EQ(config.angle_tolerance_rad, cfg.max_angle_tolerance_rad);
}

TEST(ToleranceDeriverTest, MarginFactorApplied)
{
    // Same input → larger margin_factor yields larger tolerance
    // (within clamp window).
    std::map<std::string, MotionTuneResult> results;
    results["distance"] = makeResult("distance", 0.05);

    MotionTuneConfig tune_cfg;
    ToleranceConfig  cfg_small;
    cfg_small.margin_factor = 1.0;
    ToleranceConfig cfg_large;
    cfg_large.margin_factor = 3.0;

    libstp::motion::UnifiedMotionPidConfig config_a;
    libstp::motion::UnifiedMotionPidConfig config_b;
    auto r_small = deriveTolerances(results, tune_cfg, config_a, cfg_small);
    auto r_large = deriveTolerances(results, tune_cfg, config_b, cfg_large);

    EXPECT_GE(r_large.derived_distance_tolerance_m,
              r_small.derived_distance_tolerance_m);
}

TEST(ToleranceDeriverTest, LateralKeyAlsoUpdatesDistance)
{
    // The deriver scans both "distance" and "lateral" keys for the
    // distance-tolerance field. Verify that "lateral" alone is enough.
    std::map<std::string, MotionTuneResult> results;
    results["lateral"] = makeResult("lateral", 0.10);

    libstp::motion::UnifiedMotionPidConfig config;
    MotionTuneConfig tune_cfg;
    ToleranceConfig  cfg;
    auto result = deriveTolerances(results, tune_cfg, config, cfg);

    EXPECT_TRUE(result.distance_updated);
}
