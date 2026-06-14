#include "autotune/firmware_pid_tune.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "autotune/control/control_elements.hpp"
#include "autotune/control/control_loop.hpp"
#include "autotune/control/control_path.hpp"
#include "autotune/control/control_path_param_estimation.hpp"
#include "autotune/control/controller_optimization.hpp"
#include "foundation/logging.hpp"

namespace libstp::autotune
{

// ============================================================================
// Algorithm constants (mirror those used by VelocityTuner — keep identical to
// reuse the same FOPDT identification and CHR derivation semantics).
// ============================================================================

static constexpr int    kRegressionHalfWindow = 3;
static constexpr double kSettlePauseMs        = 300.0;
static constexpr double kResetSettleMs        = 100.0;
static constexpr double kTailFrac             = 0.20;
static constexpr double kSteadyStateFrac      = 0.10;
static constexpr double kRiseLowFrac          = 0.10;
static constexpr double kRiseHighFrac         = 0.63;

// ----------------------------------------------------------------------------
// Local helpers (copies of VelocityTuner internals — same math, file-scoped).
// ----------------------------------------------------------------------------

namespace
{
struct RawPwmTrial
{
    std::vector<StepResponseData> accel;
    std::vector<StepResponseData> decel;
};

std::vector<StepResponseData> sampleMotors(
    const std::vector<hal::motor::IMotor*>& motors,
    double                                  command,
    double                                  duration_s,
    int                                     sample_hz)
{
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;

    const std::size_t n = motors.size();
    std::vector<StepResponseData> out(n);
    if (n == 0) return out;

    const Micros period{1'000'000 / std::max(1, sample_hz)};
    const std::size_t reserve = static_cast<std::size_t>(duration_s * sample_hz * 1.1);
    for (auto& data : out)
    {
        data.times.reserve(reserve);
        data.commanded.reserve(reserve);
        data.measured.reserve(reserve);
    }

    auto t0 = Clock::now();
    auto next = t0;
    while (true)
    {
        const auto now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - t0).count();
        if (elapsed >= duration_s)
            break;

        for (std::size_t i = 0; i < n; ++i)
        {
            const double measured = motors[i] != nullptr
                ? static_cast<double>(motors[i]->getBemf())
                : 0.0;
            out[i].times.push_back(elapsed);
            out[i].commanded.push_back(command);
            out[i].measured.push_back(measured);
        }

        next += period;
        const auto now2 = Clock::now();
        if (next > now2)
            std::this_thread::sleep_until(next);
        else
            next = now2;
    }

    return out;
}

RawPwmTrial runRawPwmTrial(const std::vector<hal::motor::IMotor*>& motors,
                           int                                     pwm_percent,
                           double                                  step_duration_s,
                           double                                  decel_duration_s,
                           int                                     sample_hz)
{
    for (auto* motor : motors)
    {
        if (motor != nullptr) motor->setSpeed(pwm_percent);
    }

    RawPwmTrial trial;
    trial.accel = sampleMotors(motors, static_cast<double>(pwm_percent),
                               step_duration_s, sample_hz);

    // Zero raw PWM and keep sampling so logs/results include the coast-down
    // behavior. Brake only after the decel window has been captured.
    for (auto* motor : motors)
    {
        if (motor != nullptr) motor->setSpeed(0);
    }
    trial.decel = sampleMotors(motors, 0.0, decel_duration_s, sample_hz);

    for (auto* motor : motors)
    {
        if (motor != nullptr) motor->brake();
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(kSettlePauseMs)));

    return trial;
}

std::pair<double, double> quadraticDerivatives(
    const std::vector<double>& times,
    const std::vector<double>& values,
    int                        centre,
    int                        half_w)
{
    const int n  = static_cast<int>(times.size());
    const int lo = std::max(0, centre - half_w);
    const int hi = std::min(n - 1, centre + half_w);

    if (hi - lo < 2)
        return {0.0, 0.0};

    double tc = times[static_cast<std::size_t>(centre)];

    double s00 = 0, s10 = 0, s20 = 0, s30 = 0, s40 = 0;
    double r0  = 0, r1  = 0, r2  = 0;

    for (int k = lo; k <= hi; ++k)
    {
        double dt  = times[static_cast<std::size_t>(k)] - tc;
        double dt2 = dt * dt;
        double y   = values[static_cast<std::size_t>(k)];

        s00 += 1.0;
        s10 += dt;
        s20 += dt2;
        s30 += dt2 * dt;
        s40 += dt2 * dt2;

        r0 += y;
        r1 += y * dt;
        r2 += y * dt2;
    }

    double det = s00 * (s20 * s40 - s30 * s30)
               - s10 * (s10 * s40 - s30 * s20)
               + s20 * (s10 * s30 - s20 * s20);

    if (std::abs(det) < 1e-30)
        return {0.0, 0.0};

    double det_b = s00 * (r1 * s40 - s30 * r2)
                 - s10 * (r0 * s40 - s20 * r2)
                 + s20 * (r0 * s30 - s20 * r1);
    double b = det_b / det;

    double det_c = s00 * (s20 * r2  - r1  * s30)
                 - s10 * (s10 * r2  - r0  * s30)
                 + s20 * (s10 * r1  - r0  * s20);
    double c = det_c / det;

    return {b, 2.0 * c};
}

std::optional<PlantParams> identifyPlantInflection(const StepResponseData& data,
                                                   double                  command)
{
    const int n = static_cast<int>(data.times.size());
    if (n < 2 * kRegressionHalfWindow + 3)
        return std::nullopt;

    int ss_start = std::max(0, static_cast<int>(n * (1.0 - kSteadyStateFrac)));
    double y_ss = 0.0;
    const int ss_count = n - ss_start;
    for (int i = ss_start; i < n; ++i)
        y_ss += data.measured[static_cast<std::size_t>(i)];
    y_ss /= static_cast<double>(ss_count);

    std::vector<double> d1(static_cast<std::size_t>(n), 0.0);
    std::vector<double> d2(static_cast<std::size_t>(n), 0.0);

    for (int i = kRegressionHalfWindow; i < n - kRegressionHalfWindow; ++i)
    {
        auto [b, two_c] = quadraticDerivatives(data.times, data.measured,
                                               i, kRegressionHalfWindow);
        d1[static_cast<std::size_t>(i)] = b;
        d2[static_cast<std::size_t>(i)] = two_c;
    }

    int infl_idx = -1;
    double best_d1 = 0.0;
    for (int i = kRegressionHalfWindow + 1; i < n - kRegressionHalfWindow; ++i)
    {
        std::size_t im1 = static_cast<std::size_t>(i - 1);
        std::size_t ii  = static_cast<std::size_t>(i);
        if (d2[im1] * d2[ii] < 0.0)
        {
            double abs_d1 = std::abs(d1[ii]);
            if (abs_d1 > best_d1)
            {
                best_d1  = abs_d1;
                infl_idx = i;
            }
        }
    }
    if (infl_idx < 0)
        return std::nullopt;

    double t_infl = data.times[static_cast<std::size_t>(infl_idx)];
    double y_infl = data.measured[static_cast<std::size_t>(infl_idx)];
    double slope  = d1[static_cast<std::size_t>(infl_idx)];

    if (std::abs(slope) < 1e-12)
        return std::nullopt;

    double t_zero  = t_infl - y_infl / slope;
    double t_ss    = t_infl + (y_ss - y_infl) / slope;
    double t_start = data.times.front();
    double Tu = std::max(t_zero - t_start, 0.001);
    double Tg = std::max(t_ss   - t_zero,  0.001);
    double Ks = (std::abs(command) > 1e-9) ? (y_ss / command) : 0.0;

    if (Ks <= 0.0)
        return std::nullopt;

    return PlantParams{Ks, Tu, Tg, "inflection"};
}

PlantParams identifyPlantRiseTime(const StepResponseData& data,
                                  double                  command)
{
    const int n = static_cast<int>(data.times.size());

    int ss_start = std::max(0, static_cast<int>(n * (1.0 - kSteadyStateFrac)));
    double y_ss = 0.0;
    for (int i = ss_start; i < n; ++i)
        y_ss += data.measured[static_cast<std::size_t>(i)];
    y_ss /= static_cast<double>(std::max(1, n - ss_start));

    double Ks = (std::abs(command) > 1e-9) ? (y_ss / command) : 0.0;

    double low_thresh  = std::abs(y_ss) * kRiseLowFrac;
    double high_thresh = std::abs(y_ss) * kRiseHighFrac;

    double t_start = data.times.empty() ? 0.0 : data.times.front();
    double t_low  = -1.0;
    double t_high = -1.0;

    for (int i = 0; i < n; ++i)
    {
        double abs_m = std::abs(data.measured[static_cast<std::size_t>(i)]);
        double t     = data.times[static_cast<std::size_t>(i)];
        if (t_low < 0.0 && abs_m >= low_thresh)
            t_low = t;
        if (t_low >= 0.0 && t_high < 0.0 && abs_m >= high_thresh)
        {
            t_high = t;
            break;
        }
    }

    double Tu = (t_low >= 0.0) ? std::max(t_low - t_start, 0.01) : 0.01;
    double Tg = (t_high >= 0.0 && t_low >= 0.0 && t_high > t_low)
                ? std::max(t_high - t_low, 0.01)
                : 0.01;

    return PlantParams{Ks, Tu, Tg, "rise_time"};
}

foundation::PidGains computeChrGains(const PlantParams&       plant,
                                     const FirmwarePidConfig& cfg)
{
    if (plant.Ks <= 0.0 || plant.Tu <= 0.0 || plant.Tg <= 0.0)
        return {0.0, 0.0, 0.0};

    double kp = cfg.chr_kp_scale * plant.Tg / (plant.Ks * plant.Tu);
    double ki = cfg.chr_ki_scale * kp / plant.Tg;
    double kd = cfg.chr_kd_scale * plant.Tg / plant.Ks;

    if (kp <= 0.0)
        return {0.0, 0.0, 0.0};

    return {kp, ki, kd};
}

double computeIse(const StepResponseData& data)
{
    const std::size_t n = data.times.size();
    if (n < 2)
        return 0.0;

    double ise = 0.0;
    for (std::size_t i = 1; i < n; ++i)
    {
        double dt = data.times[i] - data.times[i - 1];
        double e0 = data.commanded[i - 1] - data.measured[i - 1];
        double e1 = data.commanded[i]     - data.measured[i];
        ise += 0.5 * (e0 * e0 + e1 * e1) * dt;
    }
    return ise;
}

double tailMeanAbs(const StepResponseData& data)
{
    const std::size_t n = data.measured.size();
    if (n == 0)
        return 0.0;

    std::size_t tail_start = static_cast<std::size_t>(
        static_cast<double>(n) * (1.0 - kTailFrac));
    tail_start = std::min(tail_start, n - 1);

    double sum = 0.0;
    std::size_t count = n - tail_start;
    for (std::size_t i = tail_start; i < n; ++i)
        sum += std::abs(data.measured[i]);
    return sum / static_cast<double>(count);
}

int trialCount(int requested)
{
    return std::max(1, requested);
}

StepResponseData averageResponses(const std::vector<StepResponseData>& trials)
{
    StepResponseData avg;
    if (trials.empty()) return avg;

    std::size_t n = trials.front().times.size();
    for (const auto& trial : trials)
    {
        n = std::min(n, trial.times.size());
        n = std::min(n, trial.commanded.size());
        n = std::min(n, trial.measured.size());
    }
    if (n == 0) return avg;

    avg.times.assign(n, 0.0);
    avg.commanded.assign(n, 0.0);
    avg.measured.assign(n, 0.0);

    for (const auto& trial : trials)
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            avg.times[i]     += trial.times[i];
            avg.commanded[i] += trial.commanded[i];
            avg.measured[i]  += trial.measured[i];
        }
    }

    const double denom = static_cast<double>(trials.size());
    for (std::size_t i = 0; i < n; ++i)
    {
        avg.times[i]     /= denom;
        avg.commanded[i] /= denom;
        avg.measured[i]  /= denom;
    }
    return avg;
}

double averageIse(const std::vector<StepResponseData>& trials)
{
    if (trials.empty()) return 0.0;

    double sum = 0.0;
    for (const auto& trial : trials)
        sum += computeIse(trial);
    return sum / static_cast<double>(trials.size());
}

// ----------------------------------------------------------------------------
// ControlTheory differential-evolution gain computation.
//
// (1) Fit a PT1 plant (K, T) to the raw-PWM step response via DE system
//     identification, then (2) DE-optimize physical firmware kp/ki/kd against
//     the simulated closed loop using the dt-EXPLICIT ControlElementPID. The
//     simulated PID matches the firmware pid.c form exactly so the optimized
//     gains transfer directly.
// ----------------------------------------------------------------------------
struct CtPidResult
{
    bool   ok{false};
    double kp{0};
    double ki{0};
    double kd{0};
    double plantK{0};
    double plantT{0};
};

CtPidResult optimizePidControlTheory(const StepResponseData&  raw_avg,
                                     double                   command,
                                     double                   stepHeight,
                                     const FirmwarePidConfig& cfg)
{
    using namespace libstp::autotune::control;

    CtPidResult out;

    const std::size_t n = raw_avg.times.size();
    if (n < 10 || raw_avg.measured.size() != n)
        return out;

    std::vector<std::pair<double, double>> meas;
    meas.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
        meas.emplace_back(raw_avg.times[i], raw_avg.measured[i]);

    // ---- Plant fit (PT1) ----
    // K0 = steady-state output / command, estimated from the last 10% tail.
    if (!(command > 0.0))
        return out;
    const std::size_t tail_start = static_cast<std::size_t>(
        static_cast<double>(n) * 0.9);
    double tail_sum = 0.0;
    std::size_t tail_count = n - tail_start;
    for (std::size_t i = tail_start; i < n; ++i)
        tail_sum += raw_avg.measured[i];
    const double tail_mean = tail_sum / static_cast<double>(std::max<std::size_t>(1, tail_count));
    const double K0 = tail_mean / command;
    if (!(K0 > 0.0))
        return out;
    const double T0 = 0.1;

    ControlPath plant;
    plant.add(std::make_unique<ControlElementPT1>(K0, T0));
    ControlPathParamEstimation est(plant, meas);
    auto p = est.optimize(cfg.de_plant_min, cfg.de_plant_max, true,
                          cfg.de_plant_steps, command, 0.0, cfg.de_seed);
    if (p.size() < 2)
        return out;
    const double plantK = p[0];
    const double plantT = p[1];
    if (!std::isfinite(plantK) || !std::isfinite(plantT) || plantK <= 0.0 || plantT <= 0.0)
        return out;

    // ---- PID optimize against the closed loop ----
    ControlPath loopPlant;
    loopPlant.add(std::make_unique<ControlElementPT1>(plantK, plantT));
    ControlElementPID pid(0, 0, 0);
    ControlLoop       loop(pid, loopPlant);
    ControllerOptimizationDifferentialEvolution opt(loop, pid);

    const double endTime = cfg.de_sim_horizon_s;  // sim dt = endTime / 5000.
    auto g = opt.optimize(0.0, cfg.de_gain_max, true, cfg.de_pid_steps,
                          stepHeight, 0.0, endTime, cfg.de_max_overshoot,
                          cfg.de_weight_ctrl, cfg.de_weight_overshoot, cfg.de_seed);
    if (g.size() < 3)
        return out;
    const double kp = g[0];
    const double ki = g[1];
    const double kd = g[2];
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) || kp <= 0.0)
        return out;

    out.ok     = true;
    out.kp     = kp;
    out.ki     = ki;
    out.kd     = kd;
    out.plantK = plantK;
    out.plantT = plantT;
    return out;
}

} // namespace

// ============================================================================
// FirmwarePidTuner
// ============================================================================

FirmwarePidTuner::FirmwarePidTuner(const std::vector<hal::motor::IMotor*>& motors,
                                    hal::imu::IIMU*                         imu)
    : motors_(motors), imu_(imu)
{
}

StepResponseData FirmwarePidTuner::runStepResponse(
    hal::motor::IMotor* motor,
    int                 bemf_target,
    double              duration_s,
    int                 sample_hz,
    const std::string&  csv_path) const
{
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;

    const Micros period{1'000'000 / std::max(1, sample_hz)};
    const std::size_t reserve = static_cast<std::size_t>(duration_s * sample_hz * 1.1);

    // Per-sample raw recording buffer for the optional CSV dump. We keep the
    // hot loop free of file I/O — accumulate in memory, flush after braking.
    struct Sample
    {
        double             t;
        int                target;
        double             this_motor_bemf;
        std::vector<int>   all_bemf;        // one per entry in motors_
        std::vector<long long> all_position; // one per entry in motors_
        float              imu_heading{0.0f};
        float              imu_gyro[3]{0.0f, 0.0f, 0.0f};
        float              imu_accel[3]{0.0f, 0.0f, 0.0f};
    };
    std::vector<Sample> raw;
    const bool record_csv = !csv_path.empty();
    if (record_csv) raw.reserve(reserve);

    motor->setVelocity(bemf_target);

    StepResponseData data;
    data.times.reserve(reserve);
    data.commanded.reserve(reserve);
    data.measured.reserve(reserve);

    const double commanded = static_cast<double>(bemf_target);

    auto t0   = Clock::now();
    auto next = t0;

    while (true)
    {
        auto   now     = Clock::now();
        double elapsed = std::chrono::duration<double>(now - t0).count();
        if (elapsed >= duration_s)
            break;

        const double measured = static_cast<double>(motor->getBemf());

        data.times.push_back(elapsed);
        data.commanded.push_back(commanded);
        data.measured.push_back(measured);

        if (record_csv)
        {
            Sample s;
            s.t = elapsed;
            s.target = bemf_target;
            s.this_motor_bemf = measured;
            s.all_bemf.reserve(motors_.size());
            s.all_position.reserve(motors_.size());
            for (auto* m : motors_)
            {
                s.all_bemf.push_back(m != nullptr ? m->getBemf() : 0);
                s.all_position.push_back(m != nullptr ? m->getPosition() : 0);
            }
            if (imu_ != nullptr)
            {
                s.imu_heading = imu_->getHeading();
                float accel_buf[3]{};
                float gyro_buf[3]{};
                float mag_buf[3]{};
                imu_->read(accel_buf, gyro_buf, mag_buf);
                s.imu_accel[0] = accel_buf[0];
                s.imu_accel[1] = accel_buf[1];
                s.imu_accel[2] = accel_buf[2];
                s.imu_gyro[0] = gyro_buf[0];
                s.imu_gyro[1] = gyro_buf[1];
                s.imu_gyro[2] = gyro_buf[2];
            }
            raw.push_back(std::move(s));
        }

        next += period;
        auto now2 = Clock::now();
        if (next > now2)
            std::this_thread::sleep_until(next);
        else
            next = now2;
    }

    motor->brake();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(kSettlePauseMs)));

    if (record_csv && !raw.empty())
    {
        std::error_code ec;
        const auto parent = std::filesystem::path(csv_path).parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent, ec);

        std::ofstream out(csv_path);
        if (out.is_open())
        {
            // Capture firmware PID gains active during this step response.
            float kp_active = 0.0f, ki_active = 0.0f, kd_active = 0.0f;
            motor->getLastFirmwarePidGains(kp_active, ki_active, kd_active);

            // Header — metadata as comments, then CSV column row.
            out << "# motor_port=" << motor->getPort()
                << " bemf_target=" << bemf_target
                << " duration_s=" << duration_s
                << " sample_hz=" << sample_hz
                << " kp_active=" << kp_active
                << " ki_active=" << ki_active
                << " kd_active=" << kd_active
                << " has_imu=" << (imu_ != nullptr ? 1 : 0) << "\n";

            out << "t,target,this_motor_bemf";
            for (std::size_t i = 0; i < motors_.size(); ++i)
            {
                const int p = motors_[i] != nullptr ? motors_[i]->getPort() : -1;
                out << ",bemf_port" << p << ",pos_port" << p;
            }
            if (imu_ != nullptr)
            {
                out << ",imu_heading,imu_gx,imu_gy,imu_gz,imu_ax,imu_ay,imu_az";
            }
            out << "\n";

            for (const auto& s : raw)
            {
                out << s.t << "," << s.target << "," << s.this_motor_bemf;
                for (std::size_t i = 0; i < motors_.size(); ++i)
                {
                    out << "," << s.all_bemf[i] << "," << s.all_position[i];
                }
                if (imu_ != nullptr)
                {
                    out << "," << s.imu_heading
                        << "," << s.imu_gyro[0] << "," << s.imu_gyro[1] << "," << s.imu_gyro[2]
                        << "," << s.imu_accel[0] << "," << s.imu_accel[1] << "," << s.imu_accel[2];
                }
                out << "\n";
            }
            LIBSTP_LOG_INFO("[FirmwarePidTuner] wrote CSV ({} samples) → {}",
                            raw.size(), csv_path);
        }
        else
        {
            LIBSTP_LOG_WARN("[FirmwarePidTuner] could not open CSV for writing: {}",
                            csv_path);
        }
    }

    return data;
}

std::vector<StepResponseData> FirmwarePidTuner::runParallelStepResponse(
    const std::vector<hal::motor::IMotor*>& motors,
    const std::vector<int>&                 bemf_targets,
    double                                  duration_s,
    int                                     sample_hz) const
{
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;

    const std::size_t n = motors.size();
    std::vector<StepResponseData> out(n);
    if (n == 0) return out;

    const Micros period{1'000'000 / std::max(1, sample_hz)};
    const std::size_t reserve = static_cast<std::size_t>(duration_s * sample_hz * 1.1);

    // Pre-allocate buffers and cache per-motor command values.
    std::vector<double> commanded(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        out[i].times.reserve(reserve);
        out[i].commanded.reserve(reserve);
        out[i].measured.reserve(reserve);
        commanded[i] = static_cast<double>(bemf_targets[i]);
    }

    // Command all motors simultaneously so they spin up together.  This is
    // important: tuning each motor alone uses a different DC-bus loading and
    // mechanical coupling regime than the real operating condition.
    for (std::size_t i = 0; i < n; ++i)
    {
        if (motors[i] != nullptr)
            motors[i]->setVelocity(bemf_targets[i]);
    }

    auto t0   = Clock::now();
    auto next = t0;

    while (true)
    {
        const auto now     = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - t0).count();
        if (elapsed >= duration_s)
            break;

        // Sample every motor's BEMF in a tight burst before sleeping.
        for (std::size_t i = 0; i < n; ++i)
        {
            const double measured = (motors[i] != nullptr)
                                        ? static_cast<double>(motors[i]->getBemf())
                                        : 0.0;
            out[i].times.push_back(elapsed);
            out[i].commanded.push_back(commanded[i]);
            out[i].measured.push_back(measured);
        }

        next += period;
        const auto now2 = Clock::now();
        if (next > now2)
            std::this_thread::sleep_until(next);
        else
            next = now2;
    }

    // Brake all motors together and settle.
    for (auto* m : motors)
    {
        if (m != nullptr) m->brake();
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(kSettlePauseMs)));

    return out;
}

FirmwarePidResult FirmwarePidTuner::tuneMotor(
    hal::motor::IMotor*      motor,
    int                      max_bemf_speed,
    const FirmwarePidConfig& cfg) const
{
    FirmwarePidResult result;
    if (motor == nullptr)
    {
        LIBSTP_LOG_WARN("[FirmwarePidTuner] Null motor pointer — skipping");
        return result;
    }
    result.motor_port = motor->getPort();

    // Keep the public single-motor API on the same raw-PWM identification path
    // as the normal multi-motor tune(). It still only commands the one motor
    // supplied here, while tune() commands the whole drive motor set.
    FirmwarePidTuner single({motor}, imu_);
    auto results = single.tune({{result.motor_port, max_bemf_speed}}, cfg);
    auto it = results.find(result.motor_port);
    return it != results.end() ? it->second : result;

    // Save the gains that were configured before we touched anything so we can
    // revert if tuning makes things worse.
    float saved_kp = 0.0f, saved_ki = 0.0f, saved_kd = 0.0f;
    motor->getLastFirmwarePidGains(saved_kp, saved_ki, saved_kd);

    // If the firmware has no usable gains yet, push sensible starters so the
    // baseline step response below actually produces motion (otherwise MAV-mode
    // with kp=ki=kd=0 leaves the motor stationary, BEMF stays at 0, and we
    // bail out with "insufficient response").  The starters also become the
    // floor we revert to on rejection — keeping a tunable robot rather than
    // dropping back to 0/0/0 where no MAV-mode motion is possible.
    constexpr float kStarterKp = 0.3f;
    constexpr float kStarterKi = 2.0f;
    constexpr float kStarterKd = 0.0f;
    const bool used_starters =
        (std::abs(saved_kp) < 0.01f && std::abs(saved_ki) < 0.01f && std::abs(saved_kd) < 0.01f);
    if (used_starters)
    {
        LIBSTP_LOG_INFO(
            "[FirmwarePidTuner] port={} no prior firmware PID gains — applying "
            "starters kp={} ki={} kd={}",
            result.motor_port, kStarterKp, kStarterKi, kStarterKd);
        motor->setFirmwarePidGains(kStarterKp, kStarterKi, kStarterKd);
        // Update saved_* so the revert path keeps the starters (not 0/0/0)
        // when tuning is rejected.
        saved_kp = kStarterKp;
        saved_ki = kStarterKi;
        saved_kd = kStarterKd;
        // Brief settle so the firmware actually picks up the new gains.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // ---- Step 1: Quiet the motor and reset position ----
    motor->resetPositionCounter();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(kResetSettleMs)));

    const int bemf_target = static_cast<int>(
        std::lround(static_cast<double>(max_bemf_speed) * cfg.step_fraction));
    const double command = static_cast<double>(bemf_target);

    LIBSTP_LOG_INFO("[FirmwarePidTuner] port={} Baseline step response "
                    "(bemf_target={})", result.motor_port, bemf_target);

    const std::string baseline_csv = cfg.csv_dir.empty()
        ? std::string{}
        : cfg.csv_dir + "/firmware_pid_port"
              + std::to_string(result.motor_port) + "_baseline.csv";

    // ---- Step 2: Baseline step response ----
    StepResponseData baseline = runStepResponse(motor, bemf_target,
                                                cfg.step_duration_s,
                                                cfg.sample_hz,
                                                baseline_csv);
    result.baseline_ise = computeIse(baseline);

    LIBSTP_LOG_INFO("[FirmwarePidTuner] port={} baseline ISE={:.6f}",
                    result.motor_port, result.baseline_ise);

    // ---- Step 3: Reject if tail too small ----
    const double tail = tailMeanAbs(baseline);
    if (tail < std::abs(command) * cfg.min_response_frac)
    {
        LIBSTP_LOG_WARN(
            "[FirmwarePidTuner] port={} insufficient response: tail={:.2f} < {:.2f}",
            result.motor_port, tail, std::abs(command) * cfg.min_response_frac);
        result.plant.method = "insufficient_response";
        result.accepted     = false;
        return result;
    }

    // ---- Step 4: FOPDT identification ----
    std::optional<PlantParams> opt_plant = identifyPlantInflection(baseline, command);
    if (opt_plant.has_value())
    {
        result.plant = *opt_plant;
        LIBSTP_LOG_INFO("[FirmwarePidTuner] port={} plant (inflection): "
                        "Ks={:.4f}, Tu={:.4f}s, Tg={:.4f}s",
                        result.motor_port, result.plant.Ks,
                        result.plant.Tu, result.plant.Tg);
    }
    else
    {
        result.plant = identifyPlantRiseTime(baseline, command);
        LIBSTP_LOG_INFO("[FirmwarePidTuner] port={} plant (rise_time): "
                        "Ks={:.4f}, Tu={:.4f}s, Tg={:.4f}s",
                        result.motor_port, result.plant.Ks,
                        result.plant.Tu, result.plant.Tg);
    }

    if (result.plant.Ks <= 0.0)
    {
        LIBSTP_LOG_WARN("[FirmwarePidTuner] port={} Ks<=0 — rejecting",
                        result.motor_port);
        return result;
    }

    // ---- Step 5: Compute CHR gains ----
    const foundation::PidGains gains = computeChrGains(result.plant, cfg);
    if (gains.kp <= 0.0)
    {
        LIBSTP_LOG_WARN("[FirmwarePidTuner] port={} CHR kp<=0 — rejecting",
                        result.motor_port);
        return result;
    }
    result.kp = static_cast<float>(gains.kp);
    result.ki = static_cast<float>(gains.ki);
    result.kd = static_cast<float>(gains.kd);

    LIBSTP_LOG_INFO(
        "[FirmwarePidTuner] port={} CHR gains: kp={:.4f}, ki={:.4f}, kd={:.4f}",
        result.motor_port, result.kp, result.ki, result.kd);

    // ---- Step 6: Apply candidate gains to firmware ----
    motor->setFirmwarePidGains(result.kp, result.ki, result.kd);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(kResetSettleMs)));

    const std::string tuned_csv = cfg.csv_dir.empty()
        ? std::string{}
        : cfg.csv_dir + "/firmware_pid_port"
              + std::to_string(result.motor_port) + "_tuned.csv";

    // ---- Step 7: Tuned step response ----
    StepResponseData tuned = runStepResponse(motor, bemf_target,
                                             cfg.step_duration_s,
                                             cfg.sample_hz,
                                             tuned_csv);
    result.tuned_ise = computeIse(tuned);

    LIBSTP_LOG_INFO("[FirmwarePidTuner] port={} tuned ISE={:.6f} "
                    "(baseline={:.6f})",
                    result.motor_port, result.tuned_ise, result.baseline_ise);

    // ---- Step 8: Accept / revert ----
    if (result.tuned_ise < result.baseline_ise)
    {
        result.accepted = true;
        LIBSTP_LOG_INFO("[FirmwarePidTuner] port={} gains ACCEPTED "
                        "({:.2f}%% ISE improvement)",
                        result.motor_port,
                        100.0 * (result.baseline_ise - result.tuned_ise) /
                            (result.baseline_ise + 1e-12));
    }
    else
    {
        result.accepted = false;
        LIBSTP_LOG_WARN("[FirmwarePidTuner] port={} gains REJECTED — reverting",
                        result.motor_port);
        motor->setFirmwarePidGains(saved_kp, saved_ki, saved_kd);
    }

    return result;
}

std::map<int, FirmwarePidResult> FirmwarePidTuner::tune(
    const std::map<int, int>& max_bemf_speeds,
    const FirmwarePidConfig&  cfg) const
{
    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  FIRMWARE PID TUNING (parallel step response)");
    LIBSTP_LOG_INFO("============================================================");

    std::map<int, FirmwarePidResult> results;

    std::vector<hal::motor::IMotor*> active_motors;
    std::vector<int> bemf_targets;
    active_motors.reserve(motors_.size());
    bemf_targets.reserve(motors_.size());

    for (auto* motor : motors_)
    {
        if (motor == nullptr) continue;

        const int port = motor->getPort();
        if (!max_bemf_speeds.empty() && max_bemf_speeds.find(port) == max_bemf_speeds.end())
        {
            LIBSTP_LOG_WARN("[FirmwarePidTuner] port={} no max_bemf_speed — skipping",
                            port);
            continue;
        }

        FirmwarePidResult result;
        result.motor_port = port;
        results[port] = result;

        active_motors.push_back(motor);
        bemf_targets.push_back(0);
    }

    if (active_motors.empty())
    {
        LIBSTP_LOG_WARN("[FirmwarePidTuner] no motors with usable max-BEMF speeds");
        return results;
    }

    std::vector<std::array<float, 3>> saved_gains(active_motors.size());
    for (std::size_t i = 0; i < active_motors.size(); ++i)
    {
        auto* motor = active_motors[i];
        auto& saved = saved_gains[i];
        motor->getLastFirmwarePidGains(saved[0], saved[1], saved[2]);

        constexpr float kStarterKp = 0.3f;
        constexpr float kStarterKi = 2.0f;
        constexpr float kStarterKd = 0.0f;
        const bool needs_starters =
            std::abs(saved[0]) < 0.01f && std::abs(saved[1]) < 0.01f &&
            std::abs(saved[2]) < 0.01f;
        if (needs_starters)
        {
            LIBSTP_LOG_INFO(
                "[FirmwarePidTuner] port={} no prior firmware PID gains — "
                "applying starters kp={} ki={} kd={}",
                motor->getPort(), kStarterKp, kStarterKi, kStarterKd);
            motor->setFirmwarePidGains(kStarterKp, kStarterKi, kStarterKd);
            saved = {kStarterKp, kStarterKi, kStarterKd};
        }
        motor->resetPositionCounter();
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(kResetSettleMs)));

    const int raw_pwm = std::clamp(cfg.raw_pwm_percent, -100, 100);
    if (raw_pwm == 0)
    {
        LIBSTP_LOG_WARN("[FirmwarePidTuner] raw_pwm_percent is zero — skipping");
        return results;
    }

    const int id_trials = trialCount(cfg.identification_trials);
    std::vector<std::vector<StepResponseData>> raw_accel_by_motor(active_motors.size());
    std::vector<std::vector<StepResponseData>> raw_decel_by_motor(active_motors.size());
    for (int trial = 0; trial < id_trials; ++trial)
    {
        LIBSTP_LOG_INFO(
            "[FirmwarePidTuner] raw PWM plant trial {}/{} (all motors, pwm={}%%, {:.2f}s)",
            trial + 1, id_trials, raw_pwm, cfg.step_duration_s);
        auto trial_data = runRawPwmTrial(active_motors, raw_pwm,
                                         cfg.step_duration_s,
                                         cfg.decel_duration_s,
                                         cfg.sample_hz);
        for (std::size_t i = 0; i < trial_data.accel.size(); ++i)
            raw_accel_by_motor[i].push_back(std::move(trial_data.accel[i]));
        for (std::size_t i = 0; i < trial_data.decel.size(); ++i)
            raw_decel_by_motor[i].push_back(std::move(trial_data.decel[i]));
    }

    std::vector<std::array<float, 3>> candidate_gains(active_motors.size());
    std::vector<bool> has_candidate(active_motors.size(), false);
    std::vector<bool> has_validation_target(active_motors.size(), false);

    for (std::size_t i = 0; i < active_motors.size(); ++i)
    {
        auto* motor = active_motors[i];
        const int port = motor->getPort();
        auto& result = results[port];
        const double command = static_cast<double>(raw_pwm);

        const StepResponseData raw_avg = averageResponses(raw_accel_by_motor[i]);

        const double tail = tailMeanAbs(raw_avg);
        if (tail < std::abs(command) * cfg.min_response_frac)
        {
            LIBSTP_LOG_WARN(
                "[FirmwarePidTuner] port={} insufficient raw PWM response: tail={:.2f} < {:.2f}",
                port, tail, std::abs(command) * cfg.min_response_frac);
            result.plant.method = "insufficient_response";
            continue;
        }

        std::optional<PlantParams> opt_plant =
            identifyPlantInflection(raw_avg, command);
        result.plant = opt_plant.has_value()
            ? *opt_plant
            : identifyPlantRiseTime(raw_avg, command);

        LIBSTP_LOG_INFO(
            "[FirmwarePidTuner] port={} raw PWM plant ({}): Ks={:.4f} BEMF/%%PWM, Tu={:.4f}s, Tg={:.4f}s, max_bemf={:.2f}",
            port, result.plant.method, result.plant.Ks,
            result.plant.Tu, result.plant.Tg, tail);

        if (result.plant.Ks <= 0.0)
        {
            LIBSTP_LOG_WARN("[FirmwarePidTuner] port={} Ks<=0 — rejecting",
                            port);
            continue;
        }

        // Validation target (BEMF setpoint for the MAV step) is needed both as
        // the DE step height and for the downstream baseline/tuned responses.
        // Compute it FIRST so the DE optimizer can target it.
        const int validation_target = static_cast<int>(
            std::lround(std::abs(tail) * cfg.step_fraction));

        bool have_gains = false;

        if (cfg.use_control_theory)
        {
            const CtPidResult ct = optimizePidControlTheory(
                raw_avg, command, static_cast<double>(validation_target), cfg);
            if (ct.ok)
            {
                result.kp         = static_cast<float>(ct.kp);
                result.ki         = static_cast<float>(ct.ki);
                result.kd         = static_cast<float>(ct.kd);
                result.pid_method = "control_theory_de";
                result.plant_K    = ct.plantK;
                result.plant_T    = ct.plantT;
                have_gains        = true;
                LIBSTP_LOG_INFO(
                    "[FirmwarePidTuner] port={} ControlTheory DE gains: "
                    "kp={:.4f}, ki={:.4f}, kd={:.4f} (plant_K={:.4f}, plant_T={:.4f}s)",
                    port, result.kp, result.ki, result.kd,
                    result.plant_K, result.plant_T);
            }
            else
            {
                LIBSTP_LOG_WARN(
                    "[FirmwarePidTuner] port={} ControlTheory DE failed — "
                    "falling back to CHR heuristic",
                    port);
            }
        }

        if (!have_gains)
        {
            const foundation::PidGains gains = computeChrGains(result.plant, cfg);
            if (gains.kp <= 0.0)
            {
                LIBSTP_LOG_WARN("[FirmwarePidTuner] port={} CHR kp<=0 — rejecting",
                                port);
                continue;
            }
            result.kp         = static_cast<float>(gains.kp);
            result.ki         = static_cast<float>(gains.ki);
            result.kd         = static_cast<float>(gains.kd);
            result.pid_method = "chr_fallback";
            LIBSTP_LOG_INFO(
                "[FirmwarePidTuner] port={} CHR fallback gains: "
                "kp={:.4f}, ki={:.4f}, kd={:.4f}",
                port, result.kp, result.ki, result.kd);
        }

        candidate_gains[i] = {result.kp, result.ki, result.kd};
        has_candidate[i] = true;

        if (validation_target > 0)
        {
            bemf_targets[i] = validation_target;
            has_validation_target[i] = true;
        }

        LIBSTP_LOG_INFO(
            "[FirmwarePidTuner] port={} candidate gains ({}): "
            "kp={:.4f}, ki={:.4f}, kd={:.4f}, validation_target={}",
            port, result.pid_method, result.kp, result.ki, result.kd,
            bemf_targets[i]);
    }

    const int val_trials = trialCount(cfg.validation_trials);
    std::vector<std::vector<StepResponseData>> baseline_by_motor(active_motors.size());
    if (std::any_of(has_validation_target.begin(), has_validation_target.end(),
                    [](bool v) { return v; }))
    {
        for (int trial = 0; trial < val_trials; ++trial)
        {
            LIBSTP_LOG_INFO("[FirmwarePidTuner] MAV baseline validation trial {}/{}",
                            trial + 1, val_trials);
            auto responses = runParallelStepResponse(active_motors, bemf_targets,
                                                     cfg.step_duration_s,
                                                     cfg.sample_hz);
            for (std::size_t i = 0; i < responses.size(); ++i)
            {
                if (has_validation_target[i])
                    baseline_by_motor[i].push_back(std::move(responses[i]));
            }
        }

        for (std::size_t i = 0; i < active_motors.size(); ++i)
        {
            if (has_validation_target[i])
                results[active_motors[i]->getPort()].baseline_ise =
                    averageIse(baseline_by_motor[i]);
        }
    }

    bool any_candidate = false;
    for (std::size_t i = 0; i < active_motors.size(); ++i)
    {
        if (!has_candidate[i] || !has_validation_target[i]) continue;
        any_candidate = true;
        active_motors[i]->setFirmwarePidGains(candidate_gains[i][0],
                                              candidate_gains[i][1],
                                              candidate_gains[i][2]);
        active_motors[i]->resetPositionCounter();
    }

    if (!any_candidate)
    {
        for (std::size_t i = 0; i < active_motors.size(); ++i)
            active_motors[i]->setFirmwarePidGains(saved_gains[i][0],
                                                  saved_gains[i][1],
                                                  saved_gains[i][2]);
    }
    else
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<long long>(kResetSettleMs)));

        std::vector<std::vector<StepResponseData>> tuned_by_motor(active_motors.size());
        for (int trial = 0; trial < val_trials; ++trial)
        {
            LIBSTP_LOG_INFO("[FirmwarePidTuner] MAV tuned validation trial {}/{}",
                            trial + 1, val_trials);
            auto responses = runParallelStepResponse(active_motors, bemf_targets,
                                                     cfg.step_duration_s,
                                                     cfg.sample_hz);
            for (std::size_t i = 0; i < responses.size(); ++i)
            {
                if (has_validation_target[i])
                    tuned_by_motor[i].push_back(std::move(responses[i]));
            }
        }

        for (std::size_t i = 0; i < active_motors.size(); ++i)
        {
            auto* motor = active_motors[i];
            const int port = motor->getPort();
            auto& result = results[port];

            if (!has_candidate[i] || !has_validation_target[i])
            {
                motor->setFirmwarePidGains(saved_gains[i][0],
                                           saved_gains[i][1],
                                           saved_gains[i][2]);
                continue;
            }

            result.tuned_ise = averageIse(tuned_by_motor[i]);
            const double accept_threshold =
                result.baseline_ise * (1.0 - cfg.accept_improvement_frac);
            if (result.tuned_ise < accept_threshold)
            {
                result.accepted = true;
                LIBSTP_LOG_INFO(
                    "[FirmwarePidTuner] port={} gains ACCEPTED ({:.2f}%% mean ISE improvement)",
                    port,
                    100.0 * (result.baseline_ise - result.tuned_ise) /
                        (result.baseline_ise + 1e-12));
            }
            else
            {
                result.accepted = false;
                LIBSTP_LOG_WARN(
                    "[FirmwarePidTuner] port={} gains REJECTED — reverting "
                    "(baseline_ise={:.6f}, tuned_ise={:.6f})",
                    port, result.baseline_ise, result.tuned_ise);
                motor->setFirmwarePidGains(saved_gains[i][0],
                                           saved_gains[i][1],
                                           saved_gains[i][2]);
            }
        }
    }

    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  FIRMWARE PID TUNING RESULTS");
    LIBSTP_LOG_INFO("============================================================");
    for (const auto& [port, r] : results)
    {
        LIBSTP_LOG_INFO("  port={}: accepted={}, Ks={:.4f}, Tu={:.4f}s, Tg={:.4f}s, "
                        "kp={:.4f}, ki={:.4f}, kd={:.4f}, "
                        "baseline_ise={:.6f}, tuned_ise={:.6f}",
                        port, r.accepted ? "yes" : "no",
                        r.plant.Ks, r.plant.Tu, r.plant.Tg,
                        r.kp, r.ki, r.kd,
                        r.baseline_ise, r.tuned_ise);
    }
    LIBSTP_LOG_INFO("============================================================");

    // Optional summary CSV — one row per tuned motor with plant fit + gains +
    // ISE for the offline analysis tooling to consume alongside the
    // per-sample dumps.
    if (!cfg.csv_dir.empty() && !results.empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(cfg.csv_dir, ec);
        const std::string summary_path = cfg.csv_dir + "/firmware_pid_summary.csv";
        std::ofstream out(summary_path);
        if (out.is_open())
        {
            out << "port,accepted,plant_method,pid_method,plant_K,plant_T,"
                   "Ks,Tu,Tg,kp,ki,kd,baseline_ise,tuned_ise\n";
            for (const auto& [port, r] : results)
            {
                out << port
                    << "," << (r.accepted ? 1 : 0)
                    << "," << r.plant.method
                    << "," << r.pid_method
                    << "," << r.plant_K
                    << "," << r.plant_T
                    << "," << r.plant.Ks
                    << "," << r.plant.Tu
                    << "," << r.plant.Tg
                    << "," << r.kp
                    << "," << r.ki
                    << "," << r.kd
                    << "," << r.baseline_ise
                    << "," << r.tuned_ise
                    << "\n";
            }
            LIBSTP_LOG_INFO("[FirmwarePidTuner] wrote summary CSV → {}",
                            summary_path);
        }
        else
        {
            LIBSTP_LOG_WARN("[FirmwarePidTuner] could not open summary CSV: {}",
                            summary_path);
        }
    }

    return results;
}

} // namespace libstp::autotune
