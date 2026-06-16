#include "autotune/characterize_drive.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "foundation/logging.hpp"
#include "foundation/types.hpp"

namespace libstp::autotune
{

// ============================================================================
// Algorithm constants (mirror Python characterize_drive.py)
// ============================================================================

static constexpr double kVelEmaAlpha          = 0.3;
static constexpr double kPlateauDerivThresh   = 0.005; // m/s per cycle
static constexpr int    kPlateauHoldCycles    = 20;
static constexpr double kPlateauMinPeakFrac   = 0.70;
static constexpr double kStoppedThreshMps     = 0.005;
static constexpr double kRampLowFrac          = 0.10;
static constexpr double kRampHighFrac         = 0.90;

// Initial odometry settle after reset (ms).
static constexpr int    kOdometrySettleMs     = 50;

// ============================================================================
// Internal sample type
// ============================================================================

struct Sample
{
    double time_s{0.0};
    double position{0.0};
    double velocity{0.0};
};

// ============================================================================
// Velocity estimation: EMA over finite differences
// ============================================================================

static void computeVelocities(std::vector<Sample>& samples)
{
    if (samples.size() < 2)
        return;

    // Seed from the first finite difference so the decel phase starts at the
    // actual velocity rather than zero.
    double dt0 = samples[1].time_s - samples[0].time_s;
    double seed = (dt0 > 1e-6)
                  ? (samples[1].position - samples[0].position) / dt0
                  : 0.0;
    samples[0].velocity = seed;
    double prev = seed;

    for (std::size_t i = 1; i < samples.size(); ++i)
    {
        double dt = samples[i].time_s - samples[i - 1].time_s;
        if (dt < 1e-6)
        {
            samples[i].velocity = prev;
            continue;
        }
        double raw      = (samples[i].position - samples[i - 1].position) / dt;
        double filtered = kVelEmaAlpha * raw + (1.0 - kVelEmaAlpha) * prev;
        samples[i].velocity = filtered;
        prev = filtered;
    }
}

// ============================================================================
// Analysis helpers
// ============================================================================

/// Compute the median of a sorted vector (does NOT sort for you).
static double medianOfSorted(std::vector<double>& sorted)
{
    if (sorted.empty())
        return 0.0;
    std::size_t n = sorted.size();
    if (n % 2 == 1)
        return sorted[n / 2];
    return 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
}

/// Find the index where the velocity plateaus; returns -1 if none found.
static int findPlateauStart(const std::vector<Sample>& samples)
{
    if (static_cast<int>(samples.size()) < kPlateauHoldCycles + 2)
        return -1;

    double peak = 0.0;
    for (const auto& s : samples)
        peak = std::max(peak, std::abs(s.velocity));

    double minPlateauVel = std::max(kStoppedThreshMps, peak * kPlateauMinPeakFrac);
    int hold = 0;

    for (std::size_t i = 2; i < samples.size(); ++i)
    {
        double dt = samples[i].time_s - samples[i - 1].time_s;
        if (dt < 1e-6)
            continue;
        double deriv = std::abs(samples[i].velocity - samples[i - 1].velocity) / dt;
        double vel   = std::abs(samples[i].velocity);

        if (vel >= minPlateauVel && deriv < kPlateauDerivThresh)
        {
            ++hold;
            if (hold >= kPlateauHoldCycles)
                return static_cast<int>(i) - kPlateauHoldCycles + 1;
        }
        else
        {
            hold = 0;
        }
    }
    return -1;
}

/// Median velocity in the plateau region [plateau_start, end).
static double analyzeMaxVelocity(const std::vector<Sample>& samples, int plateau_start)
{
    std::vector<double> vels;
    vels.reserve(samples.size() - static_cast<std::size_t>(plateau_start));
    for (std::size_t i = static_cast<std::size_t>(plateau_start); i < samples.size(); ++i)
        vels.push_back(std::abs(samples[i].velocity));

    if (vels.empty())
        return 0.0;
    std::sort(vels.begin(), vels.end());
    return medianOfSorted(vels);
}

/// Fallback: median of the top 20% of |v| values (robust when no plateau).
static double fallbackMaxVelocity(const std::vector<Sample>& samples)
{
    std::vector<double> vels;
    for (const auto& s : samples)
        if (std::abs(s.velocity) > kStoppedThreshMps)
            vels.push_back(std::abs(s.velocity));

    if (vels.empty())
        return 0.0;

    std::sort(vels.begin(), vels.end());
    std::size_t top_start = static_cast<std::size_t>(
        static_cast<double>(vels.size()) * 0.80);
    std::vector<double> top(vels.begin() + static_cast<std::ptrdiff_t>(top_start),
                            vels.end());
    return medianOfSorted(top);
}

/// Acceleration during ramp-up.
///
/// Least-squares linear fit of |v(t)| over the 10%–90% window of the measured
/// peak. Same approach as analyzeDecel for consistency and noise robustness.
static double analyzeAccel(const std::vector<Sample>& samples, double max_vel)
{
    if (max_vel < kStoppedThreshMps)
        return 0.0;

    const double low  = max_vel * kRampLowFrac;
    const double high = max_vel * kRampHighFrac;

    // Take samples in the rising window — only up to the first peak crossing
    // to avoid including the plateau or any post-peak dip.
    std::vector<double> ts;
    std::vector<double> vs;
    ts.reserve(samples.size());
    vs.reserve(samples.size());
    bool reached_high = false;
    for (const auto& s : samples)
    {
        const double v = std::abs(s.velocity);
        if (v >= high)
            reached_high = true;
        if (reached_high)
            break;
        if (v >= low)
        {
            ts.push_back(s.time_s);
            vs.push_back(v);
        }
    }

    if (ts.size() < 3)
    {
        // Fallback: max_vel / time_to_peak.
        double peak_time = 0.0;
        double peak_vel  = 0.0;
        for (const auto& s : samples)
        {
            if (std::abs(s.velocity) > peak_vel)
            {
                peak_vel  = std::abs(s.velocity);
                peak_time = s.time_s;
            }
        }
        if (peak_time <= 1e-6)
            return 0.0;
        return max_vel / peak_time;
    }

    const double n = static_cast<double>(ts.size());
    double sum_t = 0.0, sum_v = 0.0;
    for (std::size_t i = 0; i < ts.size(); ++i) { sum_t += ts[i]; sum_v += vs[i]; }
    const double mean_t = sum_t / n;
    const double mean_v = sum_v / n;
    double cov = 0.0, var = 0.0;
    for (std::size_t i = 0; i < ts.size(); ++i)
    {
        const double dt = ts[i] - mean_t;
        cov += dt * (vs[i] - mean_v);
        var += dt * dt;
    }
    if (var <= 1e-12)
        return 0.0;
    return cov / var; // positive (accelerating)
}

/// Deceleration during coast-down.
///
/// Uses a least-squares linear fit of |v(t)| over the 10%–90% window of the
/// known peak velocity (from the accel phase). The slope of that fit is the
/// average deceleration. A linear fit is robust to sample noise compared to
/// picking two crossing points, and matches what motion planners actually
/// want (a single effective deceleration rate).
static double analyzeDecel(const std::vector<Sample>& samples, double known_peak)
{
    if (samples.size() < 3 || known_peak < kStoppedThreshMps)
        return 0.0;

    const double high = known_peak * kRampHighFrac;
    const double low  = known_peak * kRampLowFrac;

    std::vector<double> ts;
    std::vector<double> vs;
    ts.reserve(samples.size());
    vs.reserve(samples.size());

    for (const auto& s : samples)
    {
        double v = std::abs(s.velocity);
        if (v <= high && v >= low)
        {
            ts.push_back(s.time_s);
            vs.push_back(v);
        }
    }

    if (ts.size() < 3)
    {
        LIBSTP_LOG_WARN(
            "    Decel: only {} samples in [10%,90%] window (peak={:.4f}); "
            "cannot fit", ts.size(), known_peak);
        return 0.0;
    }

    // Least-squares slope of v vs t: slope = cov(t,v) / var(t).
    const double n = static_cast<double>(ts.size());
    double sum_t = 0.0, sum_v = 0.0;
    for (std::size_t i = 0; i < ts.size(); ++i) { sum_t += ts[i]; sum_v += vs[i]; }
    const double mean_t = sum_t / n;
    const double mean_v = sum_v / n;
    double cov = 0.0, var = 0.0;
    for (std::size_t i = 0; i < ts.size(); ++i)
    {
        const double dt = ts[i] - mean_t;
        cov += dt * (vs[i] - mean_v);
        var += dt * dt;
    }
    if (var <= 1e-12)
        return 0.0;
    const double slope = cov / var; // dv/dt (negative — decelerating)
    return -slope;                  // return positive deceleration magnitude
}

// ============================================================================
// Sampling loop
// ============================================================================

using Clock  = std::chrono::steady_clock;
using Micros = std::chrono::microseconds;

/**
 * Record position samples at the configured Hz until either:
 *  - the timeout expires, OR
 *  - the live EMA velocity has been within the plateau band for
 *    kPlateauHoldCycles consecutive cycles (early exit during accel phase).
 *
 * @param getPos      Callable returning the current scalar position.
 * @param timeout_s   Maximum recording time in seconds.
 * @param sample_hz   Target sampling rate.
 * @param early_exit  If true, stop when a velocity plateau is detected.
 * @return            Recorded samples (time_s relative to start, position, velocity=0).
 */
template <typename GetPosFn>
static std::vector<Sample> recordSamples(
    GetPosFn getPos,
    double   timeout_s,
    int      sample_hz,
    bool     early_exit)
{
    const Micros period{1'000'000 / sample_hz};
    auto next = Clock::now();
    auto t0   = Clock::now();

    std::vector<Sample> samples;
    samples.reserve(static_cast<std::size_t>(timeout_s * sample_hz * 1.1));

    // Live EMA state for early-exit plateau detection.
    //
    // Plateau heuristic: track the running max of filtered |v|. As long as |v|
    // keeps growing (within tolerance), reset the "stable" timer. Once it
    // stops growing for kPlateauHoldSeconds AND we're above a minimum speed
    // (so we don't trigger on the initial stationary samples or after a wall
    // collision), declare the plateau reached.
    double prev_vel       = 0.0;
    double max_filtered   = 0.0;
    double last_growth_t  = 0.0;
    bool   seeded         = false;
    constexpr double kPlateauHoldSeconds = 0.10; // 100 ms of stable max
    constexpr double kPlateauGrowthFrac  = 1.01; // count as "growing" if > 1% above max

    while (true)
    {
        auto now     = Clock::now();
        double elapsed = std::chrono::duration<double>(now - t0).count();
        if (elapsed >= timeout_s)
            break;

        double pos = getPos();
        samples.push_back({elapsed, pos, 0.0});

        if (early_exit && samples.size() >= 2)
        {
            std::size_t n  = samples.size();
            double dt      = samples[n - 1].time_s - samples[n - 2].time_s;
            double raw_vel = (dt > 1e-6)
                             ? (samples[n - 1].position - samples[n - 2].position) / dt
                             : prev_vel;

            if (!seeded)
            {
                prev_vel       = raw_vel;
                max_filtered   = std::abs(raw_vel);
                last_growth_t  = elapsed;
                seeded         = true;
            }

            double filtered = kVelEmaAlpha * raw_vel + (1.0 - kVelEmaAlpha) * prev_vel;
            samples.back().velocity = filtered;
            const double absv = std::abs(filtered);

            if (absv > max_filtered * kPlateauGrowthFrac)
            {
                max_filtered  = absv;
                last_growth_t = elapsed;
            }

            // Plateau iff we've been at >70% of max for kPlateauHoldSeconds
            // without further growth. This catches both real plateaus and
            // wall-collision plateaus (which we want to stop on too — the
            // recorded data still has the valid peak in the middle).
            const bool above_min = absv >= max_filtered * 0.70;
            const bool stable    = (elapsed - last_growth_t) >= kPlateauHoldSeconds;
            if (above_min && stable && max_filtered > kStoppedThreshMps)
                break;

            prev_vel = filtered;
        }

        next += period;
        auto now2 = Clock::now();
        if (next > now2)
            std::this_thread::sleep_until(next);
        else
            next = now2; // reset if we fell behind
    }

    return samples;
}

// ============================================================================
// Per-trial logic
// ============================================================================

static AxisResult runSingleTrial(
    drive::Drive&            drive,
    odometry::IOdometry&     odometry,
    const std::string&       axis,
    const CharacterizeConfig& cfg)
{
    AxisResult result{};

    // ---- Reset odometry and settle ------------------------------------------
    odometry.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(kOdometrySettleMs));

    // ---- Accel phase ---------------------------------------------------------
    // Command full power.
    foundation::ChassisVelocity dir{};
    if (axis == "forward")       dir = {1.0, 0.0, 0.0};
    else if (axis == "lateral")  dir = {0.0, 1.0, 0.0};
    else                         dir = {0.0, 0.0, 1.0}; // angular

    drive.applyPowerCommand(dir, cfg.power_percent);

    // Angular axis: getHeading() returns a wrapped value in [-π, π], which
    // produces spurious ±2π velocity spikes when the robot crosses ±π. Track
    // the accumulated (unwrapped) heading so velocity = Δpos/Δt stays valid
    // across multiple rotations.
    double angular_unwrapped = 0.0;
    double angular_prev_raw  = odometry.getHeading();
    auto getPos = [&]() -> double {
        if (axis == "angular")
        {
            const double raw = odometry.getHeading();
            double delta = raw - angular_prev_raw;
            if (delta > 3.14159265358979) delta -= 2.0 * 3.14159265358979;
            else if (delta < -3.14159265358979) delta += 2.0 * 3.14159265358979;
            angular_unwrapped += delta;
            angular_prev_raw   = raw;
            return angular_unwrapped;
        }
        // Use the frame-independent straight-line distance, NOT .forward /
        // .lateral. getDistanceFromOrigin() projects onto the origin frame
        // (origin_heading_ == 0), i.e. the calib board's fixed x/y axes — which
        // are not the robot's forward/lateral unless it happens to start axis
        // aligned. For a pure single-axis trial the robot travels in a straight
        // line, so the Euclidean distance from the origin IS the distance along
        // that axis, regardless of the board's orientation. (This was the
        // long-standing "characterize aliases calib-board position" bug.)
        return odometry.getDistanceFromOrigin().straight_line;
    };

    auto accel_samples = recordSamples(getPos, cfg.accel_timeout, cfg.sample_hz,
                                       /*early_exit=*/true);

    // Immediately brake and record the coast-down in the same pass. This
    // avoids a second ramp (which would crash the robot into the table wall
    // for the forward axis) and gives a guaranteed-valid coast-down
    // measurement, since the wheels are at the just-measured peak velocity.
    for (auto* m : drive.getMotors())
        m->brake();

    auto decel_samples = recordSamples(getPos, cfg.decel_timeout, cfg.sample_hz,
                                       /*early_exit=*/false);

    // Re-compute velocities from scratch for accurate analysis.
    computeVelocities(accel_samples);

    if (static_cast<int>(accel_samples.size()) < 10)
    {
        LIBSTP_LOG_WARN("    Too few samples ({}) in accel phase", accel_samples.size());
        return result;
    }

    // Find max velocity.
    int plateau_idx = findPlateauStart(accel_samples);
    if (plateau_idx >= 0)
    {
        result.max_velocity = analyzeMaxVelocity(accel_samples, plateau_idx);
    }
    else
    {
        result.max_velocity = fallbackMaxVelocity(accel_samples);

        // Collect the actual sorted max for the warning.
        std::vector<double> all_vels;
        all_vels.reserve(accel_samples.size());
        for (const auto& s : accel_samples)
            all_vels.push_back(std::abs(s.velocity));
        std::sort(all_vels.begin(), all_vels.end());
        double raw_max = all_vels.empty() ? 0.0 : all_vels.back();

        LIBSTP_LOG_WARN(
            "    No velocity plateau detected — using top-window median "
            "({:.4f}) instead of max ({:.4f})",
            result.max_velocity, raw_max);
    }

    result.acceleration = analyzeAccel(accel_samples, result.max_velocity);

    // decel_samples were already recorded right after brake (above).
    computeVelocities(decel_samples);
    result.deceleration = analyzeDecel(decel_samples, result.max_velocity);

    // CSV dump for diagnosis (enabled via env var LIBSTP_AUTOTUNE_CSV).
    if (const char* dir = std::getenv("LIBSTP_AUTOTUNE_CSV"))
    {
        std::string path = std::string(dir) + "/decel_" + axis + ".csv";
        std::ofstream f(path);
        if (f)
        {
            f << "t,pos,vel\n";
            for (const auto& s : decel_samples)
                f << s.time_s << "," << s.position << "," << s.velocity << "\n";
            LIBSTP_LOG_INFO("    Decel CSV written: {} ({} samples)", path, decel_samples.size());
        }

        std::string accel_path = std::string(dir) + "/accel_" + axis + ".csv";
        std::ofstream af(accel_path);
        if (af)
        {
            af << "t,pos,vel\n";
            for (const auto& s : accel_samples)
                af << s.time_s << "," << s.position << "," << s.velocity << "\n";
        }
    }

    return result;
}

// ============================================================================
// DriveCharacterizer
// ============================================================================

DriveCharacterizer::DriveCharacterizer(drive::Drive&        drive,
                                       odometry::IOdometry& odometry)
    : drive_(drive), odometry_(odometry)
{
}

void DriveCharacterizer::brakeMotors() const
{
    for (auto* m : drive_.getMotors())
        m->brake();
}

AxisResult DriveCharacterizer::characterizeAxis(const std::string& axis,
                                                const CharacterizeConfig& cfg) const
{
    const bool is_angular = (axis == "angular");
    const char* unit       = is_angular ? "rad/s" : "m/s";
    const char* accel_unit = is_angular ? "rad/s²" : "m/s²";

    if (axis != "forward" && axis != "lateral" && axis != "angular")
    {
        LIBSTP_LOG_WARN("Unknown axis '{}' — skipping", axis);
        return {};
    }

    LIBSTP_LOG_INFO("--- {} axis ---", axis);

    std::vector<AxisResult> trial_results;
    trial_results.reserve(static_cast<std::size_t>(cfg.trials));

    for (int t = 1; t <= cfg.trials; ++t)
    {
        LIBSTP_LOG_INFO("  [{}] Trial {}/{} (power={}%)", axis, t, cfg.trials,
                        cfg.power_percent);

        AxisResult r = runSingleTrial(drive_, odometry_, axis, cfg);
        trial_results.push_back(r);

        LIBSTP_LOG_INFO("    max_vel={:.4f} {}, accel={:.4f} {}, decel={:.4f} {}",
                        r.max_velocity, unit, r.acceleration, accel_unit,
                        r.deceleration, accel_unit);

        // Settle between trials.
        brakeMotors();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Filter out failed (zero-velocity) trials.
    std::vector<AxisResult> valid;
    for (const auto& r : trial_results)
        if (r.max_velocity > 0.0)
            valid.push_back(r);

    if (valid.empty())
    {
        LIBSTP_LOG_WARN("  [{}] All {} trials failed", axis, cfg.trials);
        return {};
    }

    // Compute per-metric medians independently.
    auto median_of = [](std::vector<AxisResult>& v, auto field) -> double {
        std::vector<double> vals;
        vals.reserve(v.size());
        for (const auto& r : v)
            vals.push_back(field(r));
        std::sort(vals.begin(), vals.end());
        return medianOfSorted(vals);
    };

    AxisResult final_result{
        median_of(valid, [](const AxisResult& r) { return r.max_velocity; }),
        median_of(valid, [](const AxisResult& r) { return r.acceleration; }),
        median_of(valid, [](const AxisResult& r) { return r.deceleration; })
    };

    if (valid.size() >= 2)
    {
        double vmax = 0.0, vmin = std::numeric_limits<double>::max();
        for (const auto& r : valid)
        {
            vmax = std::max(vmax, r.max_velocity);
            vmin = std::min(vmin, r.max_velocity);
        }
        LIBSTP_LOG_INFO("  [{}] Median of {} trials (vel spread={:.4f} {})",
                        axis, valid.size(), vmax - vmin, unit);
    }

    return final_result;
}

std::map<std::string, AxisResult>
DriveCharacterizer::characterize(const std::vector<std::string>& axes,
                                 const CharacterizeConfig& cfg) const
{
    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  DRIVE CHARACTERIZATION (raw motor power)");
    LIBSTP_LOG_INFO("  Trials: {}, Power: {}%", cfg.trials, cfg.power_percent);
    LIBSTP_LOG_INFO("============================================================");

    std::map<std::string, AxisResult> results;

    for (const auto& axis : axes)
    {
        results[axis] = characterizeAxis(axis, cfg);

        // Stop and settle between axes.
        brakeMotors();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Summary.
    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  RESULTS");
    LIBSTP_LOG_INFO("============================================================");
    for (const auto& [ax, r] : results)
    {
        bool ang = (ax == "angular");
        LIBSTP_LOG_INFO(
            "  {}: max_vel={:.4f} {}, accel={:.4f} {}, decel={:.4f} {}",
            ax,
            r.max_velocity, ang ? "rad/s" : "m/s",
            r.acceleration, ang ? "rad/s²" : "m/s²",
            r.deceleration, ang ? "rad/s²" : "m/s²");
    }
    LIBSTP_LOG_INFO("============================================================");

    return results;
}

} // namespace libstp::autotune
