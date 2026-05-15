#include "autotune/characterize_drive.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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

// Pause before the decel ramp (seconds).
static constexpr double kInterPhausePauseMs   = 300;   // 0.3 s expressed in ms
// Duration to ramp up to full speed before cutting power for decel test.
static constexpr double kDecelRampDurationMs  = 1500;  // 1.5 s
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

/// Acceleration from 10%→90% crossing times.
static double analyzeAccel(const std::vector<Sample>& samples, double max_vel)
{
    double low  = max_vel * kRampLowFrac;
    double high = max_vel * kRampHighFrac;

    double t_low  = -1.0;
    double t_high = -1.0;

    for (const auto& s : samples)
    {
        double v = std::abs(s.velocity);
        if (t_low < 0.0 && v >= low)
            t_low = s.time_s;
        if (t_low >= 0.0 && t_high < 0.0 && v >= high)
        {
            t_high = s.time_s;
            break;
        }
    }

    if (t_low < 0.0 || t_high < 0.0 || t_high <= t_low)
    {
        // Fallback: max_vel / time_to_peak
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
        if (peak_time <= 1e-6 || max_vel <= 0.0)
            return 0.0;
        return max_vel / peak_time;
    }

    return (high - low) / (t_high - t_low);
}

/// Deceleration from 90%→10% crossing times during coast-down.
static double analyzeDecel(const std::vector<Sample>& samples)
{
    if (samples.size() < 3)
        return 0.0;

    double peak = 0.0;
    for (const auto& s : samples)
        peak = std::max(peak, std::abs(s.velocity));

    if (peak < kStoppedThreshMps)
        return 0.0;

    double high = peak * kRampHighFrac;
    double low  = peak * kRampLowFrac;

    double t_high = -1.0;
    double t_low  = -1.0;

    for (const auto& s : samples)
    {
        double v = std::abs(s.velocity);
        // During coast-down velocity drops: first crossing below 90%, then below 10%.
        if (t_high < 0.0 && v <= high)
            t_high = s.time_s;
        if (t_high >= 0.0 && t_low < 0.0 && v <= low)
        {
            t_low = s.time_s;
            break;
        }
    }

    if (t_high < 0.0 || t_low < 0.0 || t_low <= t_high)
        return 0.0;

    return (high - low) / (t_low - t_high);
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
    double prev_vel   = 0.0;
    int    hold       = 0;
    bool   seeded     = false;

    while (true)
    {
        auto now     = Clock::now();
        double elapsed = std::chrono::duration<double>(now - t0).count();
        if (elapsed >= timeout_s)
            break;

        double pos = getPos();
        samples.push_back({elapsed, pos, 0.0});

        // Live EMA velocity for early exit (accel phase only).
        if (early_exit && samples.size() >= 2)
        {
            std::size_t n  = samples.size();
            double dt      = samples[n - 1].time_s - samples[n - 2].time_s;
            double raw_vel = (dt > 1e-6)
                             ? (samples[n - 1].position - samples[n - 2].position) / dt
                             : prev_vel;

            if (!seeded)
            {
                // Seed from first finite difference rather than zero.
                // This gives faster EMA convergence than Python's live loop
                // (which seeds at zero). The offline computeVelocities() pass
                // overwrites the velocity field anyway, so this only affects
                // the early-exit plateau timing, not the final analysis.
                prev_vel = raw_vel;
                seeded   = true;
            }

            double filtered = kVelEmaAlpha * raw_vel + (1.0 - kVelEmaAlpha) * prev_vel;
            samples.back().velocity = filtered;

            if (samples.size() >= 3)
            {
                double deriv = (dt > 1e-6)
                               ? std::abs(filtered - samples[n - 2].velocity) / dt
                               : 0.0;
                if (deriv < kPlateauDerivThresh)
                    ++hold;
                else
                    hold = 0;

                if (hold >= kPlateauHoldCycles)
                    break;
            }
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

    auto getPos = [&]() -> double {
        if (axis == "angular")
            return odometry.getHeading();
        auto d = odometry.getDistanceFromOrigin();
        return (axis == "forward") ? d.forward : d.lateral;
    };

    auto accel_samples = recordSamples(getPos, cfg.accel_timeout, cfg.sample_hz,
                                       /*early_exit=*/true);

    // Brake all motors.
    for (auto* m : drive.getMotors())
        m->brake();

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

    // ---- Brief pause --------------------------------------------------------
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<long long>(kInterPhausePauseMs)));

    // ---- Decel phase setup: reset odometry ----------------------------------
    odometry.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(kOdometrySettleMs));

    // Ramp up to full speed for 1.5 s then cut.
    drive.applyPowerCommand(dir, cfg.power_percent);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<long long>(kDecelRampDurationMs)));

    // Brake — record coast-down.
    for (auto* m : drive.getMotors())
        m->brake();

    auto decel_samples = recordSamples(getPos, cfg.decel_timeout, cfg.sample_hz,
                                       /*early_exit=*/false);

    computeVelocities(decel_samples);
    result.deceleration = analyzeDecel(decel_samples);

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
