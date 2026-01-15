#include "calibration/motion/drive_straight_autotune.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <numbers>
#include <thread>

#include "foundation/logging.hpp"
#include "foundation/types.hpp"
#include "odometry/angle_utils.hpp"

namespace libstp::calibration
{
    namespace
    {
        struct Extremum
        {
            double value;
            double time;
        };

        double averagePeriod(const std::vector<Extremum>& extrema)
        {
            if (extrema.size() < 2) return 0.0;

            double sum = 0.0;
            for (std::size_t i = 1; i < extrema.size(); ++i)
            {
                sum += extrema[i].time - extrema[i - 1].time;
            }
            return sum / static_cast<double>(extrema.size() - 1);
        }

        double averageValue(const std::vector<Extremum>& extrema)
        {
            if (extrema.empty()) return 0.0;
            const double sum = std::accumulate(extrema.begin(), extrema.end(), 0.0,
                                               [](double acc, const Extremum& e) { return acc + e.value; });
            return sum / static_cast<double>(extrema.size());
        }
    }

    DriveStraightAutotuner::DriveStraightAutotuner(drive::Drive& drive,
                                                   odometry::IOdometry& odom,
                                                   MotionCalibrationConfig cfg)
        : drive_(drive)
        , odom_(odom)
        , cfg_(std::move(cfg))
    {
    }

    DriveStraightAutotuneResult DriveStraightAutotuner::autotuneHeading()
    {
        DriveStraightAutotuneResult result;

        const double dt = (cfg_.control_rate_hz > 1e-6) ? (1.0 / cfg_.control_rate_hz) : 0.01;
        if (dt <= 0.0)
        {
            result.error = "Invalid control rate";
            return result;
        }

        if (cfg_.relay_yaw_rate <= 0.0)
        {
            result.error = "relay_yaw_rate must be positive";
            return result;
        }

        const double forward_speed = std::abs(cfg_.forward_speed_mps);

        LIBSTP_LOG_INFO(
            "DriveStraightAutotune: relay test start (relay_yaw_rate={:.3f} rad/s, forward_speed={:.3f} m/s, dt={:.4f}s)",
            cfg_.relay_yaw_rate,
            forward_speed,
            dt);

        drive_.softStop();
        odom_.reset();

        std::vector<double> headings;
        std::vector<double> times;
        headings.reserve(2000);
        times.reserve(2000);

        auto start_time = std::chrono::steady_clock::now();
        auto next_tick = start_time;

        int zero_crossings = 0;
        bool have_last_error = false;
        double last_error = 0.0;
        const int required_cycles = cfg_.min_heading_cycles + cfg_.settle_skip_cycles;

        while (true)
        {
            const auto now = std::chrono::steady_clock::now();
            const double t = std::chrono::duration<double>(now - start_time).count();
            if (t >= cfg_.max_heading_autotune_time) break;

            odom_.update(dt);

            const double heading = odom_.getHeading();
            const double error = odometry::wrapAngle(-heading);  // target heading = 0 rad
            const double yaw_cmd = (error >= 0.0) ? cfg_.relay_yaw_rate : -cfg_.relay_yaw_rate;

            drive_.setVelocity(foundation::ChassisVelocity{forward_speed, 0.0, yaw_cmd});
            [[maybe_unused]] const auto cmd = drive_.update(dt);

            headings.push_back(heading);
            times.push_back(t);

            if (have_last_error)
            {
                if ((error >= 0.0 && last_error < 0.0) || (error <= 0.0 && last_error > 0.0))
                {
                    zero_crossings++;
                    if (zero_crossings / 2 >= required_cycles && t > 2.0)
                    {
                        break;  // Enough oscillations captured
                    }
                }
            }

            have_last_error = true;
            last_error = error;

            next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(dt));
            std::this_thread::sleep_until(next_tick);
        }

        // Stop motion
        drive_.setVelocity(foundation::ChassisVelocity{0.0, 0.0, 0.0});
        [[maybe_unused]] const auto stop_cmd = drive_.update(dt);

        result.relay_output = cfg_.relay_yaw_rate;

        if (times.size() < 10)
        {
            result.error = "Insufficient samples collected for autotune";
            return result;
        }

        // Peak/trough detection
        std::vector<Extremum> peaks;
        std::vector<Extremum> troughs;
        double last_peak_time = -1e9;
        double last_trough_time = -1e9;

        for (std::size_t i = 1; i + 1 < headings.size(); ++i)
        {
            const double prev = headings[i - 1];
            const double cur = headings[i];
            const double next = headings[i + 1];
            const double t = times[i];

            if (cur > prev && cur > next && (t - last_peak_time) >= cfg_.min_peak_separation)
            {
                peaks.push_back({cur, t});
                last_peak_time = t;
            }
            else if (cur < prev && cur < next && (t - last_trough_time) >= cfg_.min_peak_separation)
            {
                troughs.push_back({cur, t});
                last_trough_time = t;
            }
        }

        if (peaks.empty() || troughs.empty())
        {
            result.error = "No oscillation peaks detected";
            return result;
        }

        // Drop initial settling cycles
        const std::size_t drop = static_cast<std::size_t>(std::clamp(cfg_.settle_skip_cycles, 0, 5));
        const auto dropPeaks = std::min(drop, peaks.size());
        const auto dropTroughs = std::min(drop, troughs.size());
        if (dropPeaks > 0) peaks.erase(peaks.begin(), peaks.begin() + static_cast<std::ptrdiff_t>(dropPeaks));
        if (dropTroughs > 0) troughs.erase(troughs.begin(), troughs.begin() + static_cast<std::ptrdiff_t>(dropTroughs));

        const std::size_t cycles = std::min(peaks.size(), troughs.size());
        result.cycles = static_cast<int>(cycles);

        if (cycles < static_cast<std::size_t>(cfg_.min_heading_cycles))
        {
            result.error = "Not enough stable oscillation cycles detected";
            return result;
        }

        const double mean_peak = averageValue(peaks);
        const double mean_trough = averageValue(troughs);
        const double amplitude = (mean_peak - mean_trough) * 0.5;
        const double amplitude_abs = std::abs(amplitude);

        if (amplitude_abs < cfg_.min_heading_amplitude)
        {
            result.error = "Oscillation amplitude too small for reliable tuning";
            return result;
        }

        const double Tu_peaks = averagePeriod(peaks);
        const double Tu_troughs = averagePeriod(troughs);
        const double Tu = (Tu_peaks > 0.0) ? Tu_peaks : Tu_troughs;

        if (Tu <= 0.0)
        {
            result.error = "Unable to compute oscillation period";
            return result;
        }

        const double Ku = (4.0 * cfg_.relay_yaw_rate) / (std::numbers::pi * amplitude_abs);

        // Conservative Ziegler–Nichols-style tuning
        const double Kp = 0.45 * Ku;
        const double Ti = 0.6 * Tu;
        const double Td = 0.125 * Tu;

        result.Ku = Ku;
        result.Tu = Tu;
        result.amplitude = amplitude_abs;
        result.kp = Kp;
        result.ki = (Ti > 1e-6) ? (Kp / Ti) : 0.0;
        result.kd = Kp * Td;
        result.success = true;

        LIBSTP_LOG_INFO(
            "Relay autotune results: Ku={:.3f}, Tu={:.3f}s, amplitude={:.4f} rad -> kp={:.4f}, ki={:.4f}, kd={:.4f} (cycles={})",
            Ku,
            Tu,
            amplitude,
            result.kp,
            result.ki,
            result.kd,
            result.cycles);

        return result;
    }
}
