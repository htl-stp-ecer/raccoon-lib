#include "autotune/encoder_calibrator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <thread>
#include <vector>

#include "foundation/logging.hpp"
#include "foundation/types.hpp"

namespace libstp::autotune
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

/// Shortest signed delta between two wrapped angles in [-π, π].
double wrappedDelta(double next, double prev)
{
    double d = next - prev;
    while (d >  kPi) d -= 2.0 * kPi;
    while (d < -kPi) d += 2.0 * kPi;
    return d;
}

} // namespace

EncoderCalibrator::EncoderCalibrator(drive::Drive&        drive,
                                     odometry::IOdometry& odometry)
    : drive_(drive), odometry_(odometry)
{
}

EncoderCalResult EncoderCalibrator::calibrate(const EncoderCalConfig& cfg) const
{
    EncoderCalResult result;

    auto motors = drive_.getMotors();
    if (motors.empty())
    {
        result.failure_reason = "no motors on drive";
        LIBSTP_LOG_WARN("[EncoderCalibrator] {}", result.failure_reason);
        return result;
    }
    if (motors.size() > 4)
    {
        LIBSTP_LOG_WARN("[EncoderCalibrator] More than 4 motors ({}) — only first 4 will be reported",
                        motors.size());
    }

    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  ENCODER CALIBRATION (IMU as ground truth)");
    LIBSTP_LOG_INFO("============================================================");

    // ---- Reset state ------------------------------------------------------
    odometry_.reset();
    std::vector<long long> ticks_start(motors.size(), 0);
    for (std::size_t i = 0; i < motors.size(); ++i)
    {
        auto* m = motors[i];
        if (m == nullptr)
            continue;
        m->resetPositionCounter();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    for (std::size_t i = 0; i < motors.size(); ++i)
    {
        auto* m = motors[i];
        if (m != nullptr)
            ticks_start[i] = m->getPosition();
    }

    const double imu_start = odometry_.getAbsoluteHeading();
    LIBSTP_LOG_INFO("[EncoderCalibrator] imu_start={:.4f}rad", imu_start);

    // ---- Command rotation -------------------------------------------------
    const double w_cmd = cfg.angular_velocity_rad_s;
    drive_.setVelocity(foundation::ChassisVelocity{0.0, 0.0, w_cmd});

    // ---- Sample loop: integrate IMU heading and odometry heading ----------
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;

    const int sample_hz = std::max(1, cfg.sample_hz);
    const Micros period{1'000'000 / sample_hz};

    double imu_accum  = 0.0;
    double odom_accum = 0.0;

    double imu_prev_raw  = imu_start;
    double odom_prev_raw = odometry_.getHeading();

    auto t0   = Clock::now();
    auto next = t0;

    const double timeout_s =
        std::max(2.0, 1.5 * std::abs(cfg.total_angle_rad / std::max(1e-3, std::abs(w_cmd))));

    while (true)
    {
        const auto now     = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - t0).count();

        // Drive the chassis controllers forward.
        const double dt = (elapsed > 0.0)
                          ? std::min(0.05, 1.0 / static_cast<double>(sample_hz))
                          : 1.0 / static_cast<double>(sample_hz);
        drive_.update(dt);
        odometry_.update(dt);

        // Accumulate unwrapped IMU heading.
        const double imu_raw = odometry_.getAbsoluteHeading();
        imu_accum += wrappedDelta(imu_raw, imu_prev_raw);
        imu_prev_raw = imu_raw;

        // Accumulate odometry-integrated heading.
        const double odom_raw = odometry_.getHeading();
        odom_accum += wrappedDelta(odom_raw, odom_prev_raw);
        odom_prev_raw = odom_raw;

        if (std::abs(imu_accum) >= std::abs(cfg.total_angle_rad))
            break;
        if (elapsed > timeout_s)
        {
            LIBSTP_LOG_WARN("[EncoderCalibrator] Timeout after {:.2f}s — "
                            "imu_accum={:.4f}rad", elapsed, imu_accum);
            break;
        }

        next += period;
        const auto now2 = Clock::now();
        if (next > now2)
            std::this_thread::sleep_until(next);
        else
            next = now2;
    }

    // ---- Stop and settle --------------------------------------------------
    drive_.hardStop();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(cfg.settle_s * 1000.0)));

    result.imu_total_angle_rad  = imu_accum;
    result.odom_total_angle_rad = odom_accum;

    LIBSTP_LOG_INFO("[EncoderCalibrator] imu_total={:.4f}rad odom_total={:.4f}rad",
                    imu_accum, odom_accum);

    if (std::abs(imu_accum) < cfg.min_required_angle_rad)
    {
        result.failure_reason = "insufficient rotation";
        LIBSTP_LOG_WARN("[EncoderCalibrator] {} (|imu|={:.4f} < {:.4f})",
                        result.failure_reason, std::abs(imu_accum),
                        cfg.min_required_angle_rad);
        return result;
    }
    if (std::abs(odom_accum) < 1e-6)
    {
        result.failure_reason = "odometry did not move";
        LIBSTP_LOG_WARN("[EncoderCalibrator] {}", result.failure_reason);
        return result;
    }

    // ---- Compute scale and rewrite per-motor calibration ------------------
    const double scale = imu_accum / odom_accum;
    LIBSTP_LOG_INFO("[EncoderCalibrator] scale = imu/odom = {:.6f}", scale);

    const std::size_t reported = std::min<std::size_t>(motors.size(), 4);
    for (std::size_t i = 0; i < motors.size(); ++i)
    {
        auto* m = motors[i];
        if (m == nullptr)
            continue;

        const auto old_cal = m->getCalibration();
        foundation::MotorCalibration new_cal = old_cal;
        new_cal.ticks_to_rad = old_cal.ticks_to_rad * scale;
        m->setCalibration(new_cal);

        if (i < reported)
        {
            result.ticks_to_rad[i]  = new_cal.ticks_to_rad;
            result.scale_factors[i] = scale;
        }

        LIBSTP_LOG_INFO("[EncoderCalibrator] motor port={} ticks_to_rad: {:.8f} -> {:.8f}",
                        m->getPort(), old_cal.ticks_to_rad, new_cal.ticks_to_rad);

        // Touch ticks_start to silence -Wunused-but-set after refactor.
        (void)ticks_start;
    }

    result.success = true;
    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  ENCODER CALIBRATION COMPLETE");
    LIBSTP_LOG_INFO("  Caller must re-publish kinematics config to STM32!");
    LIBSTP_LOG_INFO("============================================================");

    return result;
}

} // namespace libstp::autotune
