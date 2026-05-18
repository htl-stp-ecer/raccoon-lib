#include "autotune/auto_tuner.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

#include "autotune/characterize_drive.hpp"
#include "autotune/encoder_calibrator.hpp"
#include "autotune/firmware_pid_tune.hpp"
#include "autotune/motion_tune.hpp"
#include "autotune/static_friction.hpp"
#include "autotune/tolerance_deriver.hpp"
#include "autotune/vel_lpf_tune.hpp"
#include "autotune/velocity_tune.hpp"
#include "foundation/logging.hpp"

namespace libstp::autotune
{

namespace
{
    constexpr int    kBemfProbeSpeedPct      = 80;
    constexpr int    kBemfProbeDurationMs    = 500;
    constexpr int    kBemfProbeSettleMs      = 100;
    constexpr int    kBemfMinPeakForUsable   = 5;
    constexpr double kDefaultAxisMaxLinear   = 0.3;
    constexpr double kDefaultAxisMaxAngular  = 2.0;

    double defaultMaxVelocity(const std::string& axis)
    {
        if (axis == "wz") return kDefaultAxisMaxAngular;
        return kDefaultAxisMaxLinear;
    }
}

AutoTuner::AutoTuner(drive::Drive&                   drive,
                     odometry::IOdometry&            odometry,
                     motion::UnifiedMotionPidConfig& motion_pid_config)
    : drive_(drive)
    , odometry_(odometry)
    , motion_pid_config_(motion_pid_config)
{
}

void AutoTuner::setConfirmCallback(ConfirmCallback cb)
{
    confirm_cb_ = std::move(cb);
}

void AutoTuner::confirm(const std::string& phase_key)
{
    if (confirm_cb_) confirm_cb_(phase_key);
}

std::vector<hal::motor::IMotor*> AutoTuner::motors()
{
    return drive_.getMotors();
}

bool AutoTuner::bemfAvailable()
{
    auto ms = motors();
    if (ms.empty()) return false;

    for (auto* m : ms) m->setSpeed(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    int peak = 0;
    for (auto* m : ms) peak = std::max(peak, std::abs(m->getBemf()));

    for (auto* m : ms) m->brake();
    std::this_thread::sleep_for(std::chrono::milliseconds(kBemfProbeSettleMs));

    if (peak < kBemfMinPeakForUsable)
    {
        LIBSTP_LOG_WARN(
            "[AutoTuner] BEMF not responding (peak={}) — BEMF-dependent phase will be skipped",
            peak);
        return false;
    }
    return true;
}

std::optional<std::map<int, int>> AutoTuner::estimateMaxBemfSpeeds()
{
    auto ms = motors();
    if (ms.empty()) return std::nullopt;

    std::map<int, int> result;
    for (auto* m : ms)
    {
        m->setSpeed(kBemfProbeSpeedPct);
        std::this_thread::sleep_for(std::chrono::milliseconds(kBemfProbeDurationMs));
        const int peak = std::abs(m->getBemf());
        m->brake();
        std::this_thread::sleep_for(std::chrono::milliseconds(kBemfProbeSettleMs));

        if (peak < kBemfMinPeakForUsable)
        {
            LIBSTP_LOG_WARN(
                "[AutoTuner] motor port={} BEMF not responding (peak={}) — "
                "firmware PID tune unavailable",
                m->getPort(), peak);
            return std::nullopt;
        }
        result[m->getPort()] = peak;
        LIBSTP_LOG_INFO("[AutoTuner] motor port={} estimated max BEMF={}",
                        m->getPort(), peak);
    }
    return result;
}

// ============================================================================
// Individual phases
// ============================================================================

std::map<int, VelLpfResult>
AutoTuner::runVelLpfPhase(const VelLpfConfig& cfg)
{
    auto ms = motors();
    if (ms.empty())
    {
        LIBSTP_LOG_WARN("[AutoTuner] vel_lpf: no drive motors — skipping");
        return {};
    }

    // VelLpfTuner already writes the tuned alpha back via motor->setCalibration().
    VelLpfTuner tuner(ms);
    return tuner.tune(cfg);
}

std::map<int, StaticFrictionResult>
AutoTuner::runStaticFrictionPhase(const StaticFrictionConfig& cfg)
{
    auto ms = motors();
    if (ms.empty())
    {
        LIBSTP_LOG_WARN("[AutoTuner] static_friction: no drive motors — skipping");
        return {};
    }

    StaticFrictionMeasurer measurer(ms);
    return measurer.measure(cfg);
}

std::map<int, FirmwarePidResult>
AutoTuner::runFirmwarePidPhase(const FirmwarePidConfig& cfg,
                                std::optional<std::map<int, int>> max_bemf_speeds)
{
    auto ms = motors();
    if (ms.empty())
    {
        LIBSTP_LOG_WARN("[AutoTuner] firmware_pid: no drive motors — skipping");
        return {};
    }

    auto speeds = max_bemf_speeds.has_value()
                      ? std::move(*max_bemf_speeds)
                      : std::map<int, int>{};

    FirmwarePidTuner tuner(ms, &drive_.getImu());
    // FirmwarePidTuner pushes accepted gains to the STM32 via IMotor::setFirmwarePidGains().
    // When `speeds` is empty, the tuner identifies max BEMF from its own raw
    // all-motor PWM step response instead of using a sequential pre-probe.
    return tuner.tune(speeds, cfg);
}

EncoderCalResult
AutoTuner::runEncoderCalibrationPhase(const EncoderCalConfig& cfg)
{
    EncoderCalibrator calibrator(drive_, odometry_);
    const auto result = calibrator.calibrate(cfg);

    // The calibrator mutates each motor's ticks_to_rad in process. Re-publish
    // the kinematics config so the STM32 odometry uses the new scale factor.
    if (result.success)
    {
        LIBSTP_LOG_INFO(
            "[AutoTuner] encoder_cal: republishing kinematics + resetting odometry");
        odometry_.reset();
    }
    else
    {
        LIBSTP_LOG_WARN("[AutoTuner] encoder_cal failed: {}", result.failure_reason);
    }
    return result;
}

std::map<std::string, AxisResult>
AutoTuner::runCharacterizePhase(const std::vector<std::string>& axes,
                                 const CharacterizeConfig& cfg)
{
    if (axes.empty())
    {
        LIBSTP_LOG_WARN("[AutoTuner] characterize: no axes requested — skipping");
        return {};
    }

    DriveCharacterizer characterizer(drive_, odometry_);
    return characterizer.characterize(axes, cfg);
}

std::map<std::string, VelocityTuneResult>
AutoTuner::runVelocityPhase(const std::vector<std::string>& axes,
                             const std::map<std::string, double>& max_velocities,
                             const VelocityTuneConfig& cfg)
{
    if (axes.empty())
    {
        LIBSTP_LOG_WARN("[AutoTuner] velocity: no axes requested — skipping");
        return {};
    }

    VelocityTuner tuner(drive_, odometry_);
    // VelocityTuner.tuneAxis() already calls drive_.setVelocityControlConfig()
    // for accepted axes; nothing else needed here.
    return tuner.tune(axes, max_velocities, cfg);
}

std::map<std::string, MotionTuneResult>
AutoTuner::runMotionPhase(const std::vector<std::string>& params,
                           const MotionTuneConfig& cfg)
{
    if (params.empty())
    {
        LIBSTP_LOG_WARN("[AutoTuner] motion: no parameters requested — skipping");
        return {};
    }

    MotionTuner tuner(drive_, odometry_, motion_pid_config_);
    // MotionTuner mutates motion_pid_config_ in place with the best gains.
    return tuner.tune(params, cfg);
}

ToleranceResult
AutoTuner::runTolerancesPhase(const std::map<std::string, MotionTuneResult>& motion_results,
                               const MotionTuneConfig& motion_cfg,
                               const ToleranceConfig& cfg)
{
    // deriveTolerances() mutates motion_pid_config_ in place.
    return deriveTolerances(motion_results, motion_cfg, motion_pid_config_, cfg);
}

// ============================================================================
// Full pipeline
// ============================================================================

AutoTuneResult AutoTuner::tuneAll(const AutoTuneConfig& cfg)
{
    AutoTuneResult out;

    const bool has_lateral = drive_.getKinematics().supportsLateralMotion();

    // Resolve axis lists from `cfg`, falling back to kinematics-driven defaults.
    auto characterize_axes = cfg.characterize_axes;
    if (characterize_axes.empty())
    {
        characterize_axes = has_lateral
            ? std::vector<std::string>{"forward", "lateral", "angular"}
            : std::vector<std::string>{"forward", "angular"};
    }

    auto velocity_axes = cfg.velocity_axes;
    if (velocity_axes.empty())
    {
        velocity_axes = has_lateral
            ? std::vector<std::string>{"vx", "vy", "wz"}
            : std::vector<std::string>{"vx", "wz"};
    }

    auto motion_params = cfg.motion_params;
    if (motion_params.empty())
    {
        motion_params = has_lateral
            ? std::vector<std::string>{"distance", "lateral", "heading"}
            : std::vector<std::string>{"distance", "heading"};
    }

    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  AUTO-TUNE PIPELINE");
    LIBSTP_LOG_INFO("============================================================");

    // ---- Phase 1: vel_lpf ----
    if (cfg.tune_vel_lpf)
    {
        confirm("vel_lpf");
        if (bemfAvailable())
        {
            out.vel_lpf = runVelLpfPhase(cfg.vel_lpf_cfg);
            out.vel_lpf_ran = true;
        }
        else
        {
            LIBSTP_LOG_WARN("[AutoTuner] vel_lpf: BEMF unavailable — skipped");
        }
    }

    // ---- Phase 2: static_friction ----
    if (cfg.tune_static_friction)
    {
        confirm("static_friction");
        out.static_friction = runStaticFrictionPhase(cfg.static_friction_cfg);
        out.static_friction_ran = true;
    }

    // ---- Phase 3: firmware_pid ----
    if (cfg.tune_firmware_pid)
    {
        confirm("firmware_pid");
        out.firmware_pid = runFirmwarePidPhase(cfg.firmware_pid_cfg, cfg.max_bemf_speeds);
        out.firmware_pid_ran = !out.firmware_pid.empty();
    }

    // ---- Phase 4: encoder_cal (ticks_to_rad) ----
    if (cfg.tune_encoder_cal)
    {
        confirm("encoder_cal");
        out.encoder_cal = runEncoderCalibrationPhase(cfg.encoder_cal_cfg);
        out.encoder_cal_ran = true;
    }

    // ---- Phase 5: characterize ----
    if (cfg.tune_characterize)
    {
        for (const auto& axis : characterize_axes)
        {
            confirm("characterize:" + axis);
            auto r = runCharacterizePhase({axis}, cfg.characterize_cfg);
            if (auto it = r.find(axis); it != r.end())
                out.characterize[axis] = it->second;
        }
        out.characterize_ran = true;
    }

    // ---- Phase 6: velocity ----
    if (cfg.tune_velocity)
    {
        // Build the max_velocities map: prefer caller-supplied, then characterize
        // results, then sensible per-axis defaults.
        std::map<std::string, double> max_velocities = cfg.max_velocities;

        auto resolveMax = [&](const std::string& axis) {
            if (auto it = max_velocities.find(axis); it != max_velocities.end())
                return it->second;
            if (axis == "vx")
            {
                if (auto it = out.characterize.find("forward"); it != out.characterize.end())
                    return it->second.max_velocity;
            }
            else if (axis == "vy")
            {
                if (auto it = out.characterize.find("lateral"); it != out.characterize.end())
                    return it->second.max_velocity;
            }
            else if (axis == "wz")
            {
                if (auto it = out.characterize.find("angular"); it != out.characterize.end())
                    return it->second.max_velocity;
            }
            return defaultMaxVelocity(axis);
        };

        for (const auto& axis : velocity_axes)
        {
            const double mv = resolveMax(axis);
            if (mv <= 0.0)
            {
                LIBSTP_LOG_WARN("[AutoTuner] velocity[{}]: no positive max — skipping",
                                axis);
                continue;
            }
            max_velocities[axis] = mv;

            confirm("velocity:" + axis);
            auto r = runVelocityPhase({axis}, max_velocities, cfg.velocity_cfg);
            if (auto it = r.find(axis); it != r.end())
                out.velocity[axis] = it->second;
        }
        out.velocity_ran = true;
    }

    // ---- Phase 7: motion ----
    if (cfg.tune_motion)
    {
        for (const auto& param : motion_params)
        {
            confirm("motion:" + param);
            auto r = runMotionPhase({param}, cfg.motion_cfg);
            if (auto it = r.find(param); it != r.end())
                out.motion[param] = it->second;
        }
        out.motion_ran = true;
    }

    // ---- Phase 8: tolerances ----
    if (cfg.tune_tolerances && !out.motion.empty())
    {
        confirm("tolerances");
        out.tolerances = runTolerancesPhase(out.motion, cfg.motion_cfg, cfg.tolerance_cfg);
        out.tolerances_ran = true;
    }

    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  AUTO-TUNE COMPLETE");
    LIBSTP_LOG_INFO("============================================================");
    return out;
}

} // namespace libstp::autotune
