#include "autotune/motion_tune.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include "foundation/logging.hpp"
#include "foundation/types.hpp"
#include "motion/linear_motion.hpp"
#include "motion/motion.hpp"
#include "motion/turn_motion.hpp"

namespace libstp::autotune
{

// ============================================================================
// MotionTuner — constructor
// ============================================================================

MotionTuner::MotionTuner(drive::Drive&                   drive,
                         odometry::IOdometry&             odometry,
                         motion::UnifiedMotionPidConfig&  pid_config)
    : drive_(drive), odometry_(odometry), pid_config_(pid_config)
{
}

// ============================================================================
// returnToOrigin
// ============================================================================

void MotionTuner::returnLinear(motion::LinearAxis      axis,
                               double                  home_heading_rad,
                               const MotionTuneConfig& cfg) const
{
    using Clock = std::chrono::steady_clock;

    // Simply retrace the SAME axis: the trial drove +axis, so drive −axis back
    // to the start, with heading-hold correction ON the whole time so it stays
    // straight. Pure straight motion (ω ≈ 0) keeps the offset PAA lever arm out
    // of the picture. Stop at the closest approach to the origin (straight-line
    // distance reaches its minimum, then starts to grow → we've passed it) or
    // on tolerance/timeout. Fixed gentle gains, not the gains being tuned.
    constexpr double kRevSpeed   = 0.10;   // m/s
    constexpr double kKpHead     = 2.0;
    constexpr double kMaxW       = 1.0;    // rad/s
    constexpr double kPosTolM    = 0.03;   // m
    constexpr double kTimeoutS   = 6.0;
    const double dt = 1.0 / static_cast<double>(cfg.sample_hz);
    const auto   clampAbs = [](double v, double lim) { return std::clamp(v, -lim, lim); };

    double min_dist = std::numeric_limits<double>::max();
    auto   t0       = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - t0).count() < kTimeoutS)
    {
        const double dist = odometry_.getDistanceFromOrigin().straight_line;
        if (dist < kPosTolM)
            break;
        // Passed the origin (distance bottomed out and is climbing again).
        if (dist > min_dist + 0.01)
            break;
        min_dist = std::min(min_dist, dist);

        const double head_err =
            std::remainder(home_heading_rad - odometry_.getHeading(), 2.0 * M_PI);

        foundation::ChassisVelocity cmd{0.0, 0.0, clampAbs(kKpHead * head_err, kMaxW)};
        if (axis == motion::LinearAxis::Lateral)
            cmd.vy = -kRevSpeed;
        else
            cmd.vx = -kRevSpeed;

        drive_.setVelocity(cmd);
        drive_.update(dt);
        std::this_thread::sleep_for(std::chrono::microseconds(1'000'000 / cfg.sample_hz));
    }

    drive_.hardStop();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(cfg.settle_s * 1000.0)));
}

void MotionTuner::returnTurn(double home_heading_rad, const MotionTuneConfig& cfg) const
{
    using Clock = std::chrono::steady_clock;

    // Rotate back to the home heading (gyro only — the offset PAA position is
    // never used while rotating). Fixed gentle gains.
    constexpr double kKpHead     = 2.0;
    constexpr double kMaxW       = 1.0;    // rad/s
    constexpr double kHeadTolRad = 0.02;   // ~1.1°
    constexpr double kTimeoutS   = 5.0;
    const double dt = 1.0 / static_cast<double>(cfg.sample_hz);
    const auto   clampAbs = [](double v, double lim) { return std::clamp(v, -lim, lim); };

    auto t0 = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - t0).count() < kTimeoutS)
    {
        const double head_err =
            std::remainder(home_heading_rad - odometry_.getHeading(), 2.0 * M_PI);
        if (std::abs(head_err) < kHeadTolRad)
            break;
        drive_.setVelocity(
            foundation::ChassisVelocity{0.0, 0.0, clampAbs(kKpHead * head_err, kMaxW)});
        drive_.update(dt);
        std::this_thread::sleep_for(std::chrono::microseconds(1'000'000 / cfg.sample_hz));
    }

    drive_.hardStop();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(cfg.settle_s * 1000.0)));
}

// ============================================================================
// linearTargetDistance
// ============================================================================

double MotionTuner::linearTargetDistance(motion::LinearAxis       axis,
                                         const MotionTuneConfig&  cfg) const
{
    const motion::AxisConstraints& constraints =
        (axis == motion::LinearAxis::Lateral) ? pid_config_.lateral : pid_config_.linear;

    double max_vel = std::max(constraints.max_velocity, 0.0);
    if (max_vel < 1e-6)
        return cfg.linear_test_distance_m;

    double dist = max_vel * 0.8;
    return std::clamp(dist, cfg.min_linear_distance_m, cfg.max_linear_distance_m);
}

// ============================================================================
// trialTimeoutS
// ============================================================================

double MotionTuner::trialTimeoutS(double                  distance_m,
                                  double                  max_velocity,
                                  double                  speed_scale,
                                  const MotionTuneConfig& cfg) const
{
    double effective = std::max(max_velocity * speed_scale, 0.05);
    double expected  = std::abs(distance_m) / effective;
    return std::clamp(expected * 3.0 + 1.0, cfg.min_timeout_s, cfg.motion_timeout_s);
}

// ============================================================================
// runLinearTrial
// ============================================================================

std::tuple<double, double, double> MotionTuner::runLinearTrial(
    motion::LinearAxis      axis,
    double                  distance_m,
    double                  speed_scale,
    const MotionTuneConfig& cfg) const
{
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;

    const Micros period{1'000'000 / cfg.sample_hz};

    // Capture world heading as the heading target for this trial.
    double target_heading_rad = odometry_.getAbsoluteHeading();

    odometry_.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Home pose for the post-trial return (origin is here after the reset).
    const double home_heading_rad = odometry_.getHeading();

    // Determine max_velocity for timeout calculation.
    const motion::AxisConstraints& constraints =
        (axis == motion::LinearAxis::Lateral) ? pid_config_.lateral : pid_config_.linear;
    double max_vel = std::max(constraints.max_velocity, 0.05);

    double timeout_s = trialTimeoutS(distance_m, max_vel, speed_scale, cfg);

    motion::LinearMotionConfig lm_cfg;
    lm_cfg.axis               = axis;
    lm_cfg.distance_m         = distance_m;
    lm_cfg.speed_scale        = speed_scale;
    lm_cfg.target_heading_rad = target_heading_rad;

    motion::LinearMotion motion(
        motion::MotionContext{drive_, odometry_, pid_config_}, lm_cfg);
    motion.start();

    auto   t0       = Clock::now();
    auto   next     = t0;
    double prev_now = 0.0;

    while (true)
    {
        auto   now_tp  = Clock::now();
        double elapsed = std::chrono::duration<double>(now_tp - t0).count();

        // Timeout check.
        if (elapsed >= timeout_s)
        {
            drive_.hardStop();
            // Frame-independent: .forward/.lateral project onto the odometry
            // origin frame (the calib board's fixed axes), not the robot's
            // heading, so they under-read whenever the robot is not axis-aligned.
            // For a pure single-axis trial the straight-line distance IS the
            // travelled distance. (Scoring uses the motion's own telemetry,
            // which is already correct — this only feeds timeout/stuck logic.)
            double progress = odometry_.getDistanceFromOrigin().straight_line;
            double remaining = std::abs(distance_m) - progress;
            LIBSTP_LOG_WARN("[MotionTuner] Linear trial timed out after {:.2f}s", elapsed);
            returnLinear(axis, home_heading_rad, cfg);
            return {cfg.score_timeout_penalty, 0.0, remaining};
        }

        // Progress for stuck detection (frame-independent; see note above).
        double progress = odometry_.getDistanceFromOrigin().straight_line;

        if (elapsed >= cfg.stuck_timeout_s && progress < cfg.stuck_linear_progress_m)
        {
            drive_.hardStop();
            double remaining = std::abs(distance_m) - progress;
            LIBSTP_LOG_WARN("[MotionTuner] Linear trial stuck at t={:.2f}s, progress={:.4f}m",
                            elapsed, progress);
            returnLinear(axis, home_heading_rad, cfg);
            return {cfg.score_timeout_penalty, 0.0, remaining};
        }

        double dt = elapsed - prev_now;
        if (dt < 1e-9)
            dt = 1.0 / static_cast<double>(cfg.sample_hz);
        prev_now = elapsed;

        motion.update(dt);

        if (motion.isFinished())
            break;

        next += period;
        auto now2 = Clock::now();
        if (next > now2)
            std::this_thread::sleep_until(next);
        else
            next = now2;
    }

    drive_.hardStop();

    // Compute metrics from telemetry.
    const auto& telemetry = motion.getTelemetry();

    double settle_time = 0.0;
    double overshoot   = 0.0;
    double final_error = 0.0;

    if (!telemetry.empty())
    {
        settle_time = telemetry.back().time_s;
        final_error = std::abs(telemetry.back().actual_error_m);

        // Overshoot = max distance beyond target.
        for (const auto& s : telemetry)
        {
            double excess = std::abs(s.position_m) - std::abs(distance_m);
            if (excess > overshoot)
                overshoot = excess;
        }
    }

    // Retrace the same axis back to the start (heading-corrected), so repeated
    // Hooke-Jeeves trials keep the robot in the same place.
    returnLinear(axis, home_heading_rad, cfg);

    return {settle_time, overshoot, final_error};
}

// ============================================================================
// runTurnTrial
// ============================================================================

std::tuple<double, double, double> MotionTuner::runTurnTrial(
    double                  angle_rad,
    double                  speed_scale,
    const MotionTuneConfig& cfg) const
{
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;

    const Micros period{1'000'000 / cfg.sample_hz};

    odometry_.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Home heading for the post-trial return (so the robot rotates back instead
    // of accumulating the trial angle across many trials).
    const double home_heading_rad = odometry_.getHeading();

    double max_vel   = std::max(pid_config_.angular.max_velocity, 0.1);
    double timeout_s = trialTimeoutS(std::abs(angle_rad), max_vel, speed_scale, cfg);

    const double stuck_rad = cfg.stuck_angular_progress_deg * (M_PI / 180.0);

    motion::TurnConfig turn_cfg;
    turn_cfg.target_angle_rad = angle_rad;
    turn_cfg.speed_scale      = speed_scale;

    motion::TurnMotion motion(
        motion::MotionContext{drive_, odometry_, pid_config_}, turn_cfg);
    motion.start();

    auto   t0       = Clock::now();
    auto   next     = t0;
    double prev_now = 0.0;

    double min_heading = 0.0;
    double max_heading = 0.0;
    double last_heading = 0.0;

    while (true)
    {
        auto   now_tp  = Clock::now();
        double elapsed = std::chrono::duration<double>(now_tp - t0).count();

        // Timeout check.
        if (elapsed >= timeout_s)
        {
            drive_.hardStop();
            double remaining = std::abs(angle_rad) - std::abs(last_heading);
            LIBSTP_LOG_WARN("[MotionTuner] Turn trial timed out after {:.2f}s", elapsed);
            returnTurn(home_heading_rad, cfg);
            return {cfg.score_timeout_penalty, 0.0, std::max(0.0, remaining)};
        }

        double heading = odometry_.getHeading();
        last_heading   = heading;
        if (heading < min_heading) min_heading = heading;
        if (heading > max_heading) max_heading = heading;

        // Stuck check.
        if (elapsed >= cfg.stuck_timeout_s && std::abs(heading) < stuck_rad)
        {
            drive_.hardStop();
            double remaining = std::abs(angle_rad) - std::abs(heading);
            LIBSTP_LOG_WARN("[MotionTuner] Turn trial stuck at t={:.2f}s, heading={:.4f}rad",
                            elapsed, heading);
            returnTurn(home_heading_rad, cfg);
            return {cfg.score_timeout_penalty, 0.0, std::max(0.0, remaining)};
        }

        double dt = elapsed - prev_now;
        if (dt < 1e-9)
            dt = 1.0 / static_cast<double>(cfg.sample_hz);
        prev_now = elapsed;

        motion.update(dt);

        if (motion.isFinished())
            break;

        next += period;
        auto now2 = Clock::now();
        if (next > now2)
            std::this_thread::sleep_until(next);
        else
            next = now2;
    }

    drive_.hardStop();

    double final_error = std::abs(angle_rad - last_heading);

    double overshoot = 0.0;
    if (angle_rad > 0.0)
        overshoot = std::max(0.0, max_heading - angle_rad);
    else
        overshoot = std::max(0.0, angle_rad - min_heading);

    double settle_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

    // Rotate back to the start heading so trials don't accumulate rotation.
    returnTurn(home_heading_rad, cfg);

    return {settle_time, overshoot, final_error};
}

// ============================================================================
// scoreMotionTrial
// ============================================================================

double MotionTuner::scoreMotionTrial(const std::string&      param_name,
                                     double                   settle_time,
                                     double                   overshoot,
                                     double                   final_error,
                                     const MotionTuneConfig&  cfg) const
{
    double base = cfg.score_settle_weight    * settle_time
                + cfg.score_overshoot_weight * overshoot
                + cfg.score_error_weight     * final_error;

    double overshoot_soft;
    double error_soft;
    double overshoot_breach_rate;
    double error_breach_rate;

    if (param_name == "distance" || param_name == "lateral")
    {
        overshoot_soft        = cfg.linear_overshoot_soft_m;
        error_soft            = cfg.linear_final_error_soft_m;
        overshoot_breach_rate = cfg.score_linear_overshoot_breach_per_m;
        error_breach_rate     = cfg.score_linear_error_breach_per_m;
    }
    else  // "heading"
    {
        overshoot_soft        = cfg.turn_overshoot_soft_rad;
        error_soft            = cfg.turn_final_error_soft_rad;
        overshoot_breach_rate = cfg.score_turn_overshoot_breach_per_rad;
        error_breach_rate     = cfg.score_turn_error_breach_per_rad;
    }

    if (overshoot > overshoot_soft)
        base += cfg.score_constraint_breach_base
                + overshoot_breach_rate * (overshoot - overshoot_soft);

    if (final_error > error_soft)
        base += cfg.score_constraint_breach_base
                + error_breach_rate * (final_error - error_soft);

    return base;
}

// ============================================================================
// evaluateGains
// ============================================================================

double MotionTuner::evaluateGains(const std::string&      param_name,
                                  double                   kp,
                                  double                   kd,
                                  int                      trial_idx,
                                  const MotionTuneConfig&  cfg) const
{
    // Select the appropriate PidConfig reference.
    foundation::PidConfig& pid_cfg =
        (param_name == "heading") ? pid_config_.heading : pid_config_.distance;

    // Apply gains in-place; motion objects read the live config when constructed.
    pid_cfg.kp = kp;
    pid_cfg.kd = kd;

    // Alternate direction each trial.
    double sign = (trial_idx % 2 == 0) ? 1.0 : -1.0;

    double settle, overshoot, final_error;

    if (param_name == "heading")
    {
        double angle_rad = sign * cfg.turn_test_angle_deg * (M_PI / 180.0);
        auto [s, o, e] = runTurnTrial(angle_rad, cfg.primary_speed_scale, cfg);
        settle = s; overshoot = o; final_error = e;
    }
    else
    {
        motion::LinearAxis axis = (param_name == "lateral")
                                      ? motion::LinearAxis::Lateral
                                      : motion::LinearAxis::Forward;
        double distance_m = sign * linearTargetDistance(axis, cfg);
        auto [s, o, e] = runLinearTrial(axis, distance_m, cfg.primary_speed_scale, cfg);
        settle = s; overshoot = o; final_error = e;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(cfg.settle_s * 1000.0)));

    return scoreMotionTrial(param_name, settle, overshoot, final_error, cfg);
}

// ============================================================================
// tuneParam — Hooke-Jeeves coordinate descent
// ============================================================================

MotionTuneResult MotionTuner::tuneParam(const std::string&      param_name,
                                        const MotionTuneConfig&  cfg) const
{
    LIBSTP_LOG_INFO("[MotionTuner] ====================================");
    LIBSTP_LOG_INFO("[MotionTuner] Tuning param '{}'", param_name);

    foundation::PidConfig& pid_cfg =
        (param_name == "heading") ? pid_config_.heading : pid_config_.distance;

    double kp = pid_cfg.kp;
    double kd = pid_cfg.kd;

    double delta_kp = std::max(kp * cfg.initial_delta_frac, cfg.min_delta);
    double delta_kd = std::max(kd * cfg.initial_delta_frac, cfg.min_delta);

    MotionTuneResult result;
    result.param_name   = param_name;
    result.initial_kp   = kp;
    result.initial_kd   = kd;

    int trial_idx = 0;

    // Seed evaluation.
    double best_score = evaluateGains(param_name, kp, kd, trial_idx++, cfg);
    result.initial_score = best_score;

    LIBSTP_LOG_INFO("[MotionTuner] [{}] Seed: kp={:.4f}, kd={:.4f}, score={:.4f}",
                    param_name, kp, kd, best_score);

    for (int iter = 0; iter < cfg.max_iterations; ++iter)
    {
        bool improved = false;

        // --- Probe kp ---
        {
            double candidates[2] = {kp + delta_kp, kp - delta_kp};
            for (double candidate : candidates)
            {
                if (candidate <= 0.0)
                    continue;
                double score = evaluateGains(param_name, candidate, kd, trial_idx++, cfg);
                if (score < best_score)
                {
                    best_score = score;
                    kp         = candidate;
                    improved   = true;
                    LIBSTP_LOG_INFO("[MotionTuner] [{}] iter={}, kp improved → {:.4f}, score={:.4f}",
                                    param_name, iter, kp, best_score);
                    break;
                }
            }
        }

        // --- Probe kd ---
        {
            double candidates[2] = {kd + delta_kd, kd - delta_kd};
            for (double candidate : candidates)
            {
                if (candidate < 0.0)
                    continue;
                double score = evaluateGains(param_name, kp, candidate, trial_idx++, cfg);
                if (score < best_score)
                {
                    best_score = score;
                    kd         = candidate;
                    improved   = true;
                    LIBSTP_LOG_INFO("[MotionTuner] [{}] iter={}, kd improved → {:.4f}, score={:.4f}",
                                    param_name, iter, kd, best_score);
                    break;
                }
            }
        }

        if (!improved)
        {
            delta_kp *= cfg.delta_shrink;
            delta_kd *= cfg.delta_shrink;
            LIBSTP_LOG_INFO("[MotionTuner] [{}] iter={}, no improvement — shrink deltas: "
                            "delta_kp={:.4f}, delta_kd={:.4f}",
                            param_name, iter, delta_kp, delta_kd);
        }

        if (delta_kp < cfg.min_delta && delta_kd < cfg.min_delta)
        {
            LIBSTP_LOG_INFO("[MotionTuner] [{}] Converged after {} iterations", param_name, iter + 1);
            result.iterations = iter + 1;
            break;
        }

        result.iterations = iter + 1;
    }

    // Write best gains into live config.
    pid_cfg.kp = kp;
    pid_cfg.kd = kd;

    result.final_kp    = kp;
    result.final_kd    = kd;
    result.final_score = best_score;

    LIBSTP_LOG_INFO("[MotionTuner] [{}] Done: kp {:.4f}→{:.4f}, kd {:.4f}→{:.4f}, "
                    "score {:.4f}→{:.4f} (iters={})",
                    param_name,
                    result.initial_kp, result.final_kp,
                    result.initial_kd, result.final_kd,
                    result.initial_score, result.final_score,
                    result.iterations);

    return result;
}

// ============================================================================
// tune — multi-param
// ============================================================================

std::map<std::string, MotionTuneResult> MotionTuner::tune(
    const std::vector<std::string>& params,
    const MotionTuneConfig&         cfg) const
{
    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  MOTION TUNING");
    {
        std::string ps;
        for (const auto& p : params) { if (!ps.empty()) ps += ", "; ps += p; }
        LIBSTP_LOG_INFO("  Params: {}", ps);
    }
    LIBSTP_LOG_INFO("============================================================");

    std::map<std::string, MotionTuneResult> results;

    for (const auto& param : params)
    {
        if (param != "distance" && param != "lateral" && param != "heading")
        {
            LIBSTP_LOG_WARN("[MotionTuner] Unknown param '{}' — skipping", param);
            continue;
        }

        results[param] = tuneParam(param, cfg);

        // Settle between params.
        drive_.hardStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  MOTION TUNING RESULTS");
    LIBSTP_LOG_INFO("============================================================");
    for (const auto& [p, r] : results)
    {
        LIBSTP_LOG_INFO("  {}: kp {:.4f}→{:.4f}, kd {:.4f}→{:.4f}, "
                        "score {:.4f}→{:.4f}, iters={}",
                        p, r.initial_kp, r.final_kp, r.initial_kd, r.final_kd,
                        r.initial_score, r.final_score, r.iterations);
    }
    LIBSTP_LOG_INFO("============================================================");

    return results;
}

} // namespace libstp::autotune
