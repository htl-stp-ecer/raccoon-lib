#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "autotune/auto_tuner.hpp"
#include "autotune/bemf_velocity_tune.hpp"
#include "autotune/characterize_drive.hpp"
#include "autotune/encoder_calibrator.hpp"
#include "autotune/firmware_pid_tune.hpp"
#include "autotune/motion_tune.hpp"
#include "autotune/static_friction.hpp"
#include "autotune/tolerance_deriver.hpp"
#include "autotune/types.hpp"
#include "autotune/vel_lpf_tune.hpp"
#include "autotune/velocity_tune.hpp"
#include "hal/IMotor.hpp"
#include "motion/motion_config.hpp"

namespace py = pybind11;

PYBIND11_MODULE(autotune, m)
{
    m.doc() = "Python bindings for libstp-autotune (DriveCharacterizer, VelocityTuner)";

    using namespace libstp::autotune;

    // ------------------------------------------------------------------
    // AxisResult — read-only from Python
    // ------------------------------------------------------------------
    py::class_<AxisResult>(m, "AxisResult")
        .def(py::init<>())
        .def_readonly("max_velocity", &AxisResult::max_velocity,
                      "Peak velocity at full power (m/s or rad/s).")
        .def_readonly("acceleration", &AxisResult::acceleration,
                      "Acceleration from 10%%–90%% of max velocity (m/s² or rad/s²).")
        .def_readonly("deceleration", &AxisResult::deceleration,
                      "Deceleration from 90%%–10%% during coast-down (m/s² or rad/s²).")
        .def("__repr__", [](const AxisResult& r) {
            std::ostringstream oss;
            oss << "AxisResult(max_velocity=" << r.max_velocity
                << ", acceleration=" << r.acceleration
                << ", deceleration=" << r.deceleration << ")";
            return oss.str();
        });

    // ------------------------------------------------------------------
    // CharacterizeConfig — read-write from Python
    // ------------------------------------------------------------------
    py::class_<CharacterizeConfig>(m, "CharacterizeConfig")
        .def(py::init<>())
        .def_readwrite("power_percent", &CharacterizeConfig::power_percent,
                       "Raw PWM percentage (1–100).")
        .def_readwrite("trials", &CharacterizeConfig::trials,
                       "Number of trials per axis; median is taken.")
        .def_readwrite("accel_timeout", &CharacterizeConfig::accel_timeout,
                       "Maximum seconds to wait for the acceleration phase.")
        .def_readwrite("decel_timeout", &CharacterizeConfig::decel_timeout,
                       "Maximum seconds to record the coast-down phase.")
        .def_readwrite("sample_hz", &CharacterizeConfig::sample_hz,
                       "Position sampling rate in Hz (default 500).");

    // ------------------------------------------------------------------
    // DriveCharacterizer
    // ------------------------------------------------------------------
    py::class_<DriveCharacterizer>(m, "DriveCharacterizer")
        .def(py::init([](libstp::drive::Drive&        drive,
                         libstp::odometry::IOdometry& odometry)
             {
                 return std::make_unique<DriveCharacterizer>(drive, odometry);
             }),
             py::arg("drive"),
             py::arg("odometry"),
             py::keep_alive<1, 2>(),  // keep drive alive while characterizer is alive
             py::keep_alive<1, 3>(),  // keep odometry alive while characterizer is alive
             "Construct a DriveCharacterizer.\n\n"
             "Parameters\n----------\n"
             "drive : Drive\n    Chassis drive controller.\n"
             "odometry : IOdometry\n    Odometry source.\n")

        .def("characterize_axis",
             &DriveCharacterizer::characterizeAxis,
             py::arg("axis"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Characterize a single axis (\"forward\", \"lateral\", or \"angular\").\n\n"
             "Releases the GIL for the full sampling loop.\n\n"
             "Returns\n-------\n"
             "AxisResult\n    Median max_velocity, acceleration, and deceleration.")

        .def("characterize",
             &DriveCharacterizer::characterize,
             py::arg("axes"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Characterize multiple axes in sequence.\n\n"
             "Releases the GIL for the full sampling loop.\n\n"
             "Returns\n-------\n"
             "dict[str, AxisResult]\n    Map from axis name to AxisResult.");

    // ------------------------------------------------------------------
    // Phase 2 — VelocityTuner types
    // ------------------------------------------------------------------

    // StepResponseData — read-only from Python
    py::class_<StepResponseData>(m, "StepResponseData",
                                 "Raw step-response recording for one velocity axis.")
        .def(py::init<>())
        .def_readonly("times",     &StepResponseData::times,
                      "Elapsed time (s) for each sample.")
        .def_readonly("commanded", &StepResponseData::commanded,
                      "Commanded velocity at each sample (m/s or rad/s).")
        .def_readonly("measured",  &StepResponseData::measured,
                      "Measured velocity at each sample (m/s or rad/s).")
        .def("__repr__", [](const StepResponseData& d) {
            std::ostringstream oss;
            oss << "StepResponseData(n_samples=" << d.times.size() << ")";
            return oss.str();
        });

    // PlantParams
    py::class_<PlantParams>(m, "PlantParams",
                            "First-order-plus-dead-time plant parameters.")
        .def(py::init<>())
        .def_readonly("Ks",     &PlantParams::Ks,
                      "Static gain (steady-state output / step input).")
        .def_readonly("Tu",     &PlantParams::Tu,
                      "Dead time in seconds.")
        .def_readonly("Tg",     &PlantParams::Tg,
                      "Time constant (lag) in seconds.")
        .def_readonly("method", &PlantParams::method,
                      "Identification method: \"inflection\", \"rise_time\", or "
                      "\"insufficient_response\".")
        .def("__repr__", [](const PlantParams& p) {
            std::ostringstream oss;
            oss << "PlantParams(Ks=" << p.Ks
                << ", Tu=" << p.Tu
                << ", Tg=" << p.Tg
                << ", method=\"" << p.method << "\")";
            return oss.str();
        });

    // VelocityTuneResult
    py::class_<VelocityTuneResult>(m, "VelocityTuneResult",
                                   "Result of tuning a single velocity axis.")
        .def(py::init<>())
        .def_readonly("axis",         &VelocityTuneResult::axis,
                      "Axis name: \"vx\", \"vy\", or \"wz\".")
        .def_readonly("plant",        &VelocityTuneResult::plant,
                      "Identified FOPDT plant parameters.")
        .def_readonly("pid",          &VelocityTuneResult::pid,
                      "CHR PID gains (PidGains).")
        .def_readonly("ff",           &VelocityTuneResult::ff,
                      "Feedforward config (Feedforward).")
        .def_readonly("baseline_ise", &VelocityTuneResult::baseline_ise,
                      "ISE of the baseline (pre-tune) step response.")
        .def_readonly("tuned_ise",    &VelocityTuneResult::tuned_ise,
                      "ISE of the tuned step response (0 if not run).")
        .def_readonly("accepted",     &VelocityTuneResult::accepted,
                      "True if the tuned gains were applied and accepted.")
        .def_readonly("baseline_response", &VelocityTuneResult::baseline_response,
                      "Raw baseline step response (StepResponseData).")
        .def_readonly("tuned_response",    &VelocityTuneResult::tuned_response,
                      "Raw tuned step response (StepResponseData; empty if skipped).")
        .def("__repr__", [](const VelocityTuneResult& r) {
            std::ostringstream oss;
            oss << "VelocityTuneResult(axis=\"" << r.axis
                << "\", accepted=" << (r.accepted ? "True" : "False")
                << ", kp=" << r.pid.kp
                << ", ki=" << r.pid.ki
                << ", kd=" << r.pid.kd
                << ", baseline_ise=" << r.baseline_ise
                << ", tuned_ise=" << r.tuned_ise << ")";
            return oss.str();
        });

    // VelocityTuneConfig — read-write from Python
    py::class_<VelocityTuneConfig>(m, "VelocityTuneConfig",
                                   "Configuration for the VelocityTuner.")
        .def(py::init<>())
        .def_readwrite("step_duration_s",      &VelocityTuneConfig::step_duration_s,
                       "Duration of each step-response recording (s).")
        .def_readwrite("step_command_frac",    &VelocityTuneConfig::step_command_frac,
                       "Fraction of max_velocity used as step command (0–1).")
        .def_readwrite("sample_hz",            &VelocityTuneConfig::sample_hz,
                       "Step-response sampling rate (Hz).")
        .def_readwrite("chr_kp_scale",         &VelocityTuneConfig::chr_kp_scale,
                       "CHR kp scaling factor.")
        .def_readwrite("chr_ki_scale",         &VelocityTuneConfig::chr_ki_scale,
                       "CHR ki scaling factor.")
        .def_readwrite("chr_kd_scale",         &VelocityTuneConfig::chr_kd_scale,
                       "CHR kd scaling factor.")
        .def_readwrite("min_response_frac",    &VelocityTuneConfig::min_response_frac,
                       "Tail/|command| threshold — below this, tuning is skipped.")
        .def_readwrite("low_dead_time_ratio",  &VelocityTuneConfig::low_dead_time_ratio,
                       "Tu/Tg below this triggers early-return without CHR.")
        .def_readwrite("low_dead_time_gain_tol",
                       &VelocityTuneConfig::low_dead_time_gain_tol,
                       "|Ks-1| tolerance accompanying the low-dead-time check.");

    // ------------------------------------------------------------------
    // VelocityTuner
    // ------------------------------------------------------------------
    py::class_<VelocityTuner>(m, "VelocityTuner",
                              "Step-response-based velocity PID tuner.")
        .def(py::init([](libstp::drive::Drive&        drive,
                          libstp::odometry::IOdometry& odometry)
             {
                 return std::make_unique<VelocityTuner>(drive, odometry);
             }),
             py::arg("drive"),
             py::arg("odometry"),
             py::keep_alive<1, 2>(),  // keep drive alive while tuner is alive
             py::keep_alive<1, 3>(),  // keep odometry alive while tuner is alive
             "Construct a VelocityTuner.\n\n"
             "Parameters\n----------\n"
             "drive : Drive\n    Chassis drive controller.\n"
             "odometry : IOdometry\n    Odometry source.\n")

        .def("tune_axis",
             &VelocityTuner::tuneAxis,
             py::arg("axis"),
             py::arg("max_velocity"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Tune a single velocity axis (\"vx\", \"vy\", or \"wz\").\n\n"
             "Releases the GIL for the full sampling loop.\n\n"
             "Returns\n-------\n"
             "VelocityTuneResult\n    Plant params, gains, ISE metrics, and acceptance flag.")

        .def("tune",
             &VelocityTuner::tune,
             py::arg("axes"),
             py::arg("max_velocities"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Tune multiple axes in sequence.\n\n"
             "Releases the GIL for the full sampling loop.\n\n"
             "Parameters\n----------\n"
             "axes : list[str]\n    Axes to tune.\n"
             "max_velocities : dict[str, float]\n"
             "    Maximum physical velocity per axis.\n"
             "cfg : VelocityTuneConfig\n    Shared tuning configuration.\n\n"
             "Returns\n-------\n"
             "dict[str, VelocityTuneResult]\n    Map from axis name to result.");

    // ------------------------------------------------------------------
    // Phase 3 — MotionTuner types
    // ------------------------------------------------------------------

    // MotionTuneResult — read-only from Python
    py::class_<MotionTuneResult>(m, "MotionTuneResult",
                                 "Result of tuning a single motion PID parameter.")
        .def(py::init<>())
        .def_readonly("param_name",     &MotionTuneResult::param_name,
                      "Parameter name: \"distance\", \"lateral\", or \"heading\".")
        .def_readonly("initial_kp",     &MotionTuneResult::initial_kp,
                      "Proportional gain at the start of tuning.")
        .def_readonly("initial_kd",     &MotionTuneResult::initial_kd,
                      "Derivative gain at the start of tuning.")
        .def_readonly("final_kp",       &MotionTuneResult::final_kp,
                      "Best-found proportional gain.")
        .def_readonly("final_kd",       &MotionTuneResult::final_kd,
                      "Best-found derivative gain.")
        .def_readonly("initial_score",  &MotionTuneResult::initial_score,
                      "Composite score of the initial (seed) trial.")
        .def_readonly("final_score",    &MotionTuneResult::final_score,
                      "Composite score of the best-found gains.")
        .def_readonly("iterations",     &MotionTuneResult::iterations,
                      "Number of Hooke-Jeeves iterations performed.")
        .def("__repr__", [](const MotionTuneResult& r) {
            std::ostringstream oss;
            oss << "MotionTuneResult(param=\"" << r.param_name
                << "\", kp=" << r.initial_kp << "->" << r.final_kp
                << ", kd=" << r.initial_kd << "->" << r.final_kd
                << ", score=" << r.initial_score << "->" << r.final_score
                << ", iters=" << r.iterations << ")";
            return oss.str();
        });

    // MotionTuneConfig — read-write from Python
    py::class_<MotionTuneConfig>(m, "MotionTuneConfig",
                                 "Configuration for the MotionTuner.")
        .def(py::init<>())
        .def_readwrite("sample_hz",                              &MotionTuneConfig::sample_hz,
                       "Update loop rate (Hz).")
        .def_readwrite("settle_s",                               &MotionTuneConfig::settle_s,
                       "Post-trial settle time (s).")
        .def_readwrite("max_iterations",                         &MotionTuneConfig::max_iterations,
                       "Maximum Hooke-Jeeves iterations.")
        .def_readwrite("initial_delta_frac",                     &MotionTuneConfig::initial_delta_frac,
                       "Fraction of initial gain for first delta.")
        .def_readwrite("min_delta",                              &MotionTuneConfig::min_delta,
                       "Minimum delta before stopping.")
        .def_readwrite("delta_shrink",                           &MotionTuneConfig::delta_shrink,
                       "Factor to shrink deltas on no improvement.")
        .def_readwrite("linear_test_distance_m",                 &MotionTuneConfig::linear_test_distance_m,
                       "Fallback linear test distance (m).")
        .def_readwrite("turn_test_angle_deg",                    &MotionTuneConfig::turn_test_angle_deg,
                       "Turn test angle (degrees).")
        .def_readwrite("motion_timeout_s",                       &MotionTuneConfig::motion_timeout_s,
                       "Maximum trial duration (s).")
        .def_readwrite("min_timeout_s",                          &MotionTuneConfig::min_timeout_s,
                       "Minimum computed timeout (s).")
        .def_readwrite("stuck_timeout_s",                        &MotionTuneConfig::stuck_timeout_s,
                       "Time without progress to declare stuck (s).")
        .def_readwrite("stuck_linear_progress_m",                &MotionTuneConfig::stuck_linear_progress_m,
                       "Linear progress threshold for stuck detection (m).")
        .def_readwrite("stuck_angular_progress_deg",             &MotionTuneConfig::stuck_angular_progress_deg,
                       "Angular progress threshold for stuck detection (degrees).")
        .def_readwrite("primary_speed_scale",                    &MotionTuneConfig::primary_speed_scale,
                       "Speed scale for trial motions (0–1).")
        .def_readwrite("min_linear_distance_m",                  &MotionTuneConfig::min_linear_distance_m,
                       "Minimum clamped test distance (m).")
        .def_readwrite("max_linear_distance_m",                  &MotionTuneConfig::max_linear_distance_m,
                       "Maximum clamped test distance (m).")
        .def_readwrite("score_settle_weight",                    &MotionTuneConfig::score_settle_weight,
                       "Weight on settle time in the composite score.")
        .def_readwrite("score_overshoot_weight",                 &MotionTuneConfig::score_overshoot_weight,
                       "Weight on overshoot in the composite score.")
        .def_readwrite("score_error_weight",                     &MotionTuneConfig::score_error_weight,
                       "Weight on final error in the composite score.")
        .def_readwrite("score_timeout_penalty",                  &MotionTuneConfig::score_timeout_penalty,
                       "Score returned on timeout or stuck trial.")
        .def_readwrite("score_constraint_breach_base",           &MotionTuneConfig::score_constraint_breach_base,
                       "Base penalty added when a soft limit is breached.")
        .def_readwrite("linear_overshoot_soft_m",                &MotionTuneConfig::linear_overshoot_soft_m,
                       "Soft overshoot limit for linear trials (m).")
        .def_readwrite("linear_final_error_soft_m",              &MotionTuneConfig::linear_final_error_soft_m,
                       "Soft final-error limit for linear trials (m).")
        .def_readwrite("turn_overshoot_soft_rad",                &MotionTuneConfig::turn_overshoot_soft_rad,
                       "Soft overshoot limit for turn trials (rad).")
        .def_readwrite("turn_final_error_soft_rad",              &MotionTuneConfig::turn_final_error_soft_rad,
                       "Soft final-error limit for turn trials (rad).")
        .def_readwrite("score_linear_overshoot_breach_per_m",    &MotionTuneConfig::score_linear_overshoot_breach_per_m,
                       "Penalty rate for linear overshoot breach (score/m).")
        .def_readwrite("score_linear_error_breach_per_m",        &MotionTuneConfig::score_linear_error_breach_per_m,
                       "Penalty rate for linear error breach (score/m).")
        .def_readwrite("score_turn_overshoot_breach_per_rad",    &MotionTuneConfig::score_turn_overshoot_breach_per_rad,
                       "Penalty rate for turn overshoot breach (score/rad).")
        .def_readwrite("score_turn_error_breach_per_rad",        &MotionTuneConfig::score_turn_error_breach_per_rad,
                       "Penalty rate for turn error breach (score/rad).");

    // ------------------------------------------------------------------
    // MotionTuner
    // ------------------------------------------------------------------
    py::class_<MotionTuner>(m, "MotionTuner",
                            "Hooke-Jeeves coordinate-descent motion PID tuner.")
        .def(py::init([](libstp::drive::Drive&                    drive,
                          libstp::odometry::IOdometry&             odometry,
                          libstp::motion::UnifiedMotionPidConfig&  pid_config)
             {
                 return std::make_unique<MotionTuner>(drive, odometry, pid_config);
             }),
             py::arg("drive"),
             py::arg("odometry"),
             py::arg("pid_config"),
             py::keep_alive<1, 2>(),  // keep drive alive while tuner is alive
             py::keep_alive<1, 3>(),  // keep odometry alive while tuner is alive
             py::keep_alive<1, 4>(),  // keep pid_config alive while tuner is alive
             "Construct a MotionTuner.\n\n"
             "Parameters\n----------\n"
             "drive : Drive\n    Chassis drive controller.\n"
             "odometry : IOdometry\n    Odometry source.\n"
             "pid_config : UnifiedMotionPidConfig\n"
             "    Motion PID config to tune in place.\n")

        .def("tune_param",
             &MotionTuner::tuneParam,
             py::arg("param_name"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Tune a single motion PID parameter.\n\n"
             "Valid param names: \"distance\", \"lateral\", \"heading\".\n"
             "Updates pid_config in place with the best-found gains.\n\n"
             "Releases the GIL for the full tuning loop.\n\n"
             "Returns\n-------\n"
             "MotionTuneResult\n    Initial/final gains and scores.")

        .def("tune",
             &MotionTuner::tune,
             py::arg("params"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Tune multiple motion PID parameters in sequence.\n\n"
             "Valid param names: \"distance\", \"lateral\", \"heading\".\n"
             "Updates pid_config in place with best-found gains for each.\n\n"
             "Releases the GIL for the full tuning loop.\n\n"
             "Returns\n-------\n"
             "dict[str, MotionTuneResult]\n    Map from param name to result.");

    // ------------------------------------------------------------------
    // Phase 4 — FirmwarePidTuner
    // ------------------------------------------------------------------
    py::class_<FirmwarePidConfig>(m, "FirmwarePidConfig",
                                  "Configuration for the FirmwarePidTuner.")
        .def(py::init<>())
        .def_readwrite("step_fraction",      &FirmwarePidConfig::step_fraction,
                       "Fraction of max BEMF speed used as step command (0-1).")
        .def_readwrite("step_duration_s",    &FirmwarePidConfig::step_duration_s,
                       "Duration of each step-response recording (s).")
        .def_readwrite("sample_hz",          &FirmwarePidConfig::sample_hz,
                       "Step-response sampling rate (Hz).")
        .def_readwrite("chr_kp_scale",       &FirmwarePidConfig::chr_kp_scale,
                       "CHR kp scaling factor.")
        .def_readwrite("chr_ki_scale",       &FirmwarePidConfig::chr_ki_scale,
                       "CHR ki scaling factor.")
        .def_readwrite("chr_kd_scale",       &FirmwarePidConfig::chr_kd_scale,
                       "CHR kd scaling factor.")
        .def_readwrite("min_response_frac",  &FirmwarePidConfig::min_response_frac,
                       "Tail/|command| threshold for accepting the recording.")
        .def_readwrite("csv_dir",            &FirmwarePidConfig::csv_dir,
                       "Directory for per-sample CSV dump (empty = disabled).");

    py::class_<FirmwarePidResult>(m, "FirmwarePidResult",
                                  "Result of tuning a single motor's firmware PID.")
        .def(py::init<>())
        .def_readonly("motor_port",   &FirmwarePidResult::motor_port)
        .def_readonly("plant",        &FirmwarePidResult::plant)
        .def_readonly("kp",           &FirmwarePidResult::kp)
        .def_readonly("ki",           &FirmwarePidResult::ki)
        .def_readonly("kd",           &FirmwarePidResult::kd)
        .def_readonly("baseline_ise", &FirmwarePidResult::baseline_ise)
        .def_readonly("tuned_ise",    &FirmwarePidResult::tuned_ise)
        .def_readonly("accepted",     &FirmwarePidResult::accepted)
        .def("__repr__", [](const FirmwarePidResult& r) {
            std::ostringstream oss;
            oss << "FirmwarePidResult(port=" << r.motor_port
                << ", accepted=" << (r.accepted ? "True" : "False")
                << ", kp=" << r.kp << ", ki=" << r.ki << ", kd=" << r.kd
                << ", baseline_ise=" << r.baseline_ise
                << ", tuned_ise=" << r.tuned_ise << ")";
            return oss.str();
        });

    py::class_<FirmwarePidTuner>(m, "FirmwarePidTuner",
                                 "Per-motor firmware velocity-PID tuner.")
        .def(py::init([](const std::vector<libstp::hal::motor::IMotor*>& motors) {
                 return std::make_unique<FirmwarePidTuner>(motors);
             }),
             py::arg("motors"),
             py::keep_alive<1, 2>(),
             "Construct a FirmwarePidTuner from a list of motor references.")

        .def("tune_motor",
             [](const FirmwarePidTuner& t,
                libstp::hal::motor::IMotor* motor,
                int max_bemf_speed,
                const FirmwarePidConfig& cfg) {
                 return t.tuneMotor(motor, max_bemf_speed, cfg);
             },
             py::arg("motor"),
             py::arg("max_bemf_speed"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Tune a single motor's firmware velocity PID.")

        .def("tune",
             &FirmwarePidTuner::tune,
             py::arg("max_bemf_speeds"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Tune every motor; max_bemf_speeds is a dict[int, int] keyed by port.");

    // ------------------------------------------------------------------
    // Phase 5 — StaticFrictionMeasurer
    // ------------------------------------------------------------------
    py::class_<StaticFrictionConfig>(m, "StaticFrictionConfig",
                                     "Configuration for the StaticFrictionMeasurer.")
        .def(py::init<>())
        .def_readwrite("start_pct",        &StaticFrictionConfig::start_pct)
        .def_readwrite("max_pct",          &StaticFrictionConfig::max_pct)
        .def_readwrite("step_pct",         &StaticFrictionConfig::step_pct)
        .def_readwrite("dwell_ms",         &StaticFrictionConfig::dwell_ms)
        .def_readwrite("samples_per_step", &StaticFrictionConfig::samples_per_step)
        .def_readwrite("motion_threshold", &StaticFrictionConfig::motion_threshold);

    py::class_<StaticFrictionSample>(m, "StaticFrictionSample",
                                     "One PWM-vs-BEMF sample from a kS sweep.")
        .def(py::init<>())
        .def_readonly("pwm_pct",     &StaticFrictionSample::pwm_pct,
                      "Signed PWM percent (+ forward, - reverse).")
        .def_readonly("median_bemf", &StaticFrictionSample::median_bemf,
                      "Median |BEMF| (ADC counts) at this step.");

    py::class_<StaticFrictionResult>(m, "StaticFrictionResult",
                                     "Per-motor kS measurement result.")
        .def(py::init<>())
        .def_readonly("motor_port",      &StaticFrictionResult::motor_port)
        .def_readonly("ks_positive_pct", &StaticFrictionResult::ks_positive_pct)
        .def_readonly("ks_negative_pct", &StaticFrictionResult::ks_negative_pct)
        .def_readonly("ks_avg_pct",      &StaticFrictionResult::ks_avg_pct)
        .def_readonly("measured",        &StaticFrictionResult::measured)
        .def_readonly("forward_sweep",   &StaticFrictionResult::forward_sweep,
                      "Forward sweep PWM-vs-BEMF curve (list[StaticFrictionSample]).")
        .def_readonly("reverse_sweep",   &StaticFrictionResult::reverse_sweep,
                      "Reverse sweep PWM-vs-BEMF curve (list[StaticFrictionSample]).")
        .def_readonly("motion_threshold", &StaticFrictionResult::motion_threshold,
                      "Median |BEMF| threshold above which the motor counts as moving.")
        .def("__repr__", [](const StaticFrictionResult& r) {
            std::ostringstream oss;
            oss << "StaticFrictionResult(port=" << r.motor_port
                << ", ks_pos=" << r.ks_positive_pct
                << "%, ks_neg=" << r.ks_negative_pct
                << "%, ks_avg=" << r.ks_avg_pct
                << "%, measured=" << (r.measured ? "True" : "False") << ")";
            return oss.str();
        });

    py::class_<StaticFrictionMeasurer>(m, "StaticFrictionMeasurer",
                                       "Per-motor static-friction measurement.")
        .def(py::init([](const std::vector<libstp::hal::motor::IMotor*>& motors) {
                 return std::make_unique<StaticFrictionMeasurer>(motors);
             }),
             py::arg("motors"),
             py::keep_alive<1, 2>(),
             "Construct a StaticFrictionMeasurer from a list of motor references.")
        .def("measure_motor",
             [](const StaticFrictionMeasurer& m_,
                libstp::hal::motor::IMotor* motor,
                const StaticFrictionConfig& cfg) {
                 return m_.measureMotor(motor, cfg);
             },
             py::arg("motor"), py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Measure kS for a single motor.")
        .def("measure",
             &StaticFrictionMeasurer::measure,
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Measure kS for every motor. Returns dict[port -> StaticFrictionResult].");

    // ------------------------------------------------------------------
    // Phase 6 — VelLpfTuner
    // ------------------------------------------------------------------
    py::class_<VelLpfConfig>(m, "VelLpfConfig",
                             "Configuration for the VelLpfTuner.")
        .def(py::init<>())
        .def_readwrite("alpha_min",          &VelLpfConfig::alpha_min)
        .def_readwrite("alpha_max",          &VelLpfConfig::alpha_max)
        .def_readwrite("alpha_step",         &VelLpfConfig::alpha_step)
        .def_readwrite("measure_duration_s", &VelLpfConfig::measure_duration_s)
        .def_readwrite("sample_hz",          &VelLpfConfig::sample_hz)
        .def_readwrite("noise_weight",       &VelLpfConfig::noise_weight)
        .def_readwrite("lag_weight",         &VelLpfConfig::lag_weight)
        .def_readwrite("spin_percent",       &VelLpfConfig::spin_percent)
        .def_readwrite("settle_s",           &VelLpfConfig::settle_s);

    py::class_<VelLpfSweepPoint>(m, "VelLpfSweepPoint",
                                 "One point of the vel_lpf alpha sweep.")
        .def(py::init<>())
        .def_readonly("alpha",           &VelLpfSweepPoint::alpha)
        .def_readonly("variance",        &VelLpfSweepPoint::variance,
                      "Variance of the filtered series at this alpha.")
        .def_readonly("lag_change_rate", &VelLpfSweepPoint::lag_change_rate,
                      "Std-dev of |delta filt| (movement proxy) at this alpha.")
        .def_readonly("score",           &VelLpfSweepPoint::score,
                      "Combined noise+lag score (lower is better).");

    py::class_<VelLpfResult>(m, "VelLpfResult",
                             "Per-motor vel_lpf_alpha tuning result.")
        .def(py::init<>())
        .def_readonly("motor_port",    &VelLpfResult::motor_port)
        .def_readonly("initial_alpha", &VelLpfResult::initial_alpha)
        .def_readonly("tuned_alpha",   &VelLpfResult::tuned_alpha)
        .def_readonly("min_score",     &VelLpfResult::min_score)
        .def_readonly("applied",       &VelLpfResult::applied)
        .def_readonly("raw_bemf",      &VelLpfResult::raw_bemf,
                      "Raw BEMF samples captured while quiescent (list[int]).")
        .def_readonly("sweep",         &VelLpfResult::sweep,
                      "Score-vs-alpha sweep curve (list[VelLpfSweepPoint]).")
        .def("__repr__", [](const VelLpfResult& r) {
            std::ostringstream oss;
            oss << "VelLpfResult(port=" << r.motor_port
                << ", alpha " << r.initial_alpha << "->" << r.tuned_alpha
                << ", score=" << r.min_score
                << ", applied=" << (r.applied ? "True" : "False") << ")";
            return oss.str();
        });

    py::class_<VelLpfTuner>(m, "VelLpfTuner",
                            "Per-motor velocity-LPF alpha tuner.")
        .def(py::init([](const std::vector<libstp::hal::motor::IMotor*>& motors) {
                 return std::make_unique<VelLpfTuner>(motors);
             }),
             py::arg("motors"),
             py::keep_alive<1, 2>(),
             "Construct a VelLpfTuner from a list of motor references.")
        .def("tune_motor",
             [](const VelLpfTuner& t,
                libstp::hal::motor::IMotor* motor,
                const VelLpfConfig& cfg) {
                 return t.tuneMotor(motor, cfg);
             },
             py::arg("motor"), py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Tune vel_lpf_alpha for a single motor.")
        .def("tune",
             &VelLpfTuner::tune,
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Tune vel_lpf_alpha for every motor.");

    // ------------------------------------------------------------------
    // Phase 7 — ToleranceDeriver
    // ------------------------------------------------------------------
    py::class_<ToleranceConfig>(m, "ToleranceConfig",
                                "Configuration for deriveTolerances().")
        .def(py::init<>())
        .def_readwrite("margin_factor",            &ToleranceConfig::margin_factor)
        .def_readwrite("min_distance_tolerance_m", &ToleranceConfig::min_distance_tolerance_m)
        .def_readwrite("max_distance_tolerance_m", &ToleranceConfig::max_distance_tolerance_m)
        .def_readwrite("min_angle_tolerance_rad",  &ToleranceConfig::min_angle_tolerance_rad)
        .def_readwrite("max_angle_tolerance_rad",  &ToleranceConfig::max_angle_tolerance_rad);

    py::class_<ToleranceResult>(m, "ToleranceResult",
                                "Result of deriveTolerances().")
        .def(py::init<>())
        .def_readonly("derived_distance_tolerance_m",
                      &ToleranceResult::derived_distance_tolerance_m)
        .def_readonly("derived_angle_tolerance_rad",
                      &ToleranceResult::derived_angle_tolerance_rad)
        .def_readonly("distance_updated", &ToleranceResult::distance_updated)
        .def_readonly("angle_updated",    &ToleranceResult::angle_updated)
        .def("__repr__", [](const ToleranceResult& r) {
            std::ostringstream oss;
            oss << "ToleranceResult(distance=" << r.derived_distance_tolerance_m
                << "m, angle=" << r.derived_angle_tolerance_rad << "rad, "
                << "updated=[dist=" << (r.distance_updated ? "Y" : "N")
                << ", angle=" << (r.angle_updated ? "Y" : "N") << "])";
            return oss.str();
        });

    m.def("derive_tolerances",
          &deriveTolerances,
          py::arg("tune_results"),
          py::arg("tune_config"),
          py::arg("motion_config"),
          py::arg("cfg") = ToleranceConfig{},
          "Derive distance/angle tolerances from MotionTuner results and "
          "update `motion_config` in place. Returns a ToleranceResult.");

    // ------------------------------------------------------------------
    // Phase 8 — EncoderCalibrator
    // ------------------------------------------------------------------
    py::class_<EncoderCalConfig>(m, "EncoderCalConfig",
                                 "Configuration for the EncoderCalibrator.")
        .def(py::init<>())
        .def_readwrite("angular_velocity_rad_s",
                       &EncoderCalConfig::angular_velocity_rad_s)
        .def_readwrite("total_angle_rad",        &EncoderCalConfig::total_angle_rad)
        .def_readwrite("settle_s",               &EncoderCalConfig::settle_s)
        .def_readwrite("sample_hz",              &EncoderCalConfig::sample_hz)
        .def_readwrite("min_required_angle_rad", &EncoderCalConfig::min_required_angle_rad);

    py::class_<EncoderCalResult>(m, "EncoderCalResult",
                                 "Result of the encoder calibration sweep.")
        .def(py::init<>())
        .def_readonly("ticks_to_rad",        &EncoderCalResult::ticks_to_rad)
        .def_readonly("scale_factors",       &EncoderCalResult::scale_factors)
        .def_readonly("imu_total_angle_rad", &EncoderCalResult::imu_total_angle_rad)
        .def_readonly("odom_total_angle_rad",&EncoderCalResult::odom_total_angle_rad)
        .def_readonly("success",             &EncoderCalResult::success)
        .def_readonly("failure_reason",      &EncoderCalResult::failure_reason)
        .def("__repr__", [](const EncoderCalResult& r) {
            std::ostringstream oss;
            oss << "EncoderCalResult(success=" << (r.success ? "True" : "False")
                << ", imu=" << r.imu_total_angle_rad
                << "rad, odom=" << r.odom_total_angle_rad << "rad";
            if (!r.failure_reason.empty())
                oss << ", reason=\"" << r.failure_reason << "\"";
            oss << ")";
            return oss.str();
        });

    py::class_<EncoderCalibrator>(m, "EncoderCalibrator",
                                  "IMU-as-ground-truth ticks_to_rad calibrator.")
        .def(py::init([](libstp::drive::Drive& drive,
                          libstp::odometry::IOdometry& odometry) {
                 return std::make_unique<EncoderCalibrator>(drive, odometry);
             }),
             py::arg("drive"),
             py::arg("odometry"),
             py::keep_alive<1, 2>(),
             py::keep_alive<1, 3>(),
             "Construct an EncoderCalibrator.")
        .def("calibrate",
             &EncoderCalibrator::calibrate,
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Run the IMU-vs-odometry calibration sweep. Updates each motor's "
             "MotorCalibration in place; the caller must re-publish the "
             "kinematics config to the STM32.");

    // ------------------------------------------------------------------
    // BEMF→velocity calibration (calibration-board ground truth)
    // ------------------------------------------------------------------
    py::class_<BemfVelocityConfig>(m, "BemfVelocityConfig",
        "Configuration for the calibration-board BEMF→rad calibration.")
        .def(py::init<>())
        .def_readwrite("pwm_levels",            &BemfVelocityConfig::pwm_levels)
        .def_readwrite("pwm_min_percent",       &BemfVelocityConfig::pwm_min_percent)
        .def_readwrite("pwm_max_percent",       &BemfVelocityConfig::pwm_max_percent)
        .def_readwrite("pwm_steps",             &BemfVelocityConfig::pwm_steps)
        .def_readwrite("sweeps",                &BemfVelocityConfig::sweeps)
        .def_readwrite("pre_roll_distance_m",   &BemfVelocityConfig::pre_roll_distance_m)
        .def_readwrite("measure_distance_m",    &BemfVelocityConfig::measure_distance_m)
        .def_readwrite("segment_timeout_s",     &BemfVelocityConfig::segment_timeout_s)
        .def_readwrite("return_pwm_percent",    &BemfVelocityConfig::return_pwm_percent)
        .def_readwrite("return_tolerance_m",    &BemfVelocityConfig::return_tolerance_m)
        .def_readwrite("return_timeout_s",      &BemfVelocityConfig::return_timeout_s)
        .def_readwrite("sample_hz",             &BemfVelocityConfig::sample_hz)
        .def_readwrite("settle_s",              &BemfVelocityConfig::settle_s)
        .def_readwrite("min_window_distance_m", &BemfVelocityConfig::min_window_distance_m)
        .def_readwrite("min_window_ticks",      &BemfVelocityConfig::min_window_ticks)
        .def_readwrite("require_calib_board",   &BemfVelocityConfig::require_calib_board)
        .def_readwrite("apply",                 &BemfVelocityConfig::apply);

    py::class_<BemfVelocityPoint>(m, "BemfVelocityPoint",
        "One measurement window at a single PWM level.")
        .def(py::init<>())
        .def_readonly("pwm_percent",             &BemfVelocityPoint::pwm_percent)
        .def_readonly("ground_truth_distance_m", &BemfVelocityPoint::ground_truth_distance_m)
        .def_readonly("window_time_s",           &BemfVelocityPoint::window_time_s)
        .def_readonly("body_speed_mps",          &BemfVelocityPoint::body_speed_mps)
        .def_readonly("wheel_omega_rad_s",       &BemfVelocityPoint::wheel_omega_rad_s)
        .def_readonly("delta_ticks",             &BemfVelocityPoint::delta_ticks)
        .def_readonly("median_bemf",             &BemfVelocityPoint::median_bemf)
        .def_readonly("ticks_to_rad",            &BemfVelocityPoint::ticks_to_rad)
        .def_readonly("valid",                   &BemfVelocityPoint::valid);

    py::class_<BemfMotorFit>(m, "BemfMotorFit",
        "Per-motor fit + linearity diagnostics across the speed sweep.")
        .def(py::init<>())
        .def_readonly("port",                 &BemfMotorFit::port)
        .def_readonly("n_points",             &BemfMotorFit::n_points)
        .def_readonly("ticks_to_rad_median",  &BemfMotorFit::ticks_to_rad_median)
        .def_readonly("ticks_to_rad_mean",    &BemfMotorFit::ticks_to_rad_mean)
        .def_readonly("ticks_to_rad_cv",      &BemfMotorFit::ticks_to_rad_cv)
        .def_readonly("bemf_omega_slope",     &BemfMotorFit::bemf_omega_slope)
        .def_readonly("bemf_omega_intercept", &BemfMotorFit::bemf_omega_intercept)
        .def_readonly("bemf_omega_r2",        &BemfMotorFit::bemf_omega_r2)
        .def_readonly("bemf_offset",          &BemfMotorFit::bemf_offset)
        .def_readonly("linear",               &BemfMotorFit::linear);

    py::class_<BemfVelocityResult>(m, "BemfVelocityResult",
        "Result of a BEMF→velocity calibration run.")
        .def(py::init<>())
        .def_readonly("points",         &BemfVelocityResult::points)
        .def_readonly("motors",         &BemfVelocityResult::motors)
        .def_readonly("ticks_to_rad",   &BemfVelocityResult::ticks_to_rad)
        .def_readonly("bemf_offset",    &BemfVelocityResult::bemf_offset)
        .def_readonly("linear_overall", &BemfVelocityResult::linear_overall)
        .def_readonly("applied",        &BemfVelocityResult::applied)
        .def_readonly("success",        &BemfVelocityResult::success)
        .def_readonly("failure_reason", &BemfVelocityResult::failure_reason)
        .def("__repr__", [](const BemfVelocityResult& r) {
            std::ostringstream oss;
            oss << "BemfVelocityResult(success=" << (r.success ? "True" : "False")
                << ", linear=" << (r.linear_overall ? "True" : "False")
                << ", points=" << r.points.size();
            if (!r.failure_reason.empty())
                oss << ", reason=\"" << r.failure_reason << "\"";
            oss << ")";
            return oss.str();
        });

    py::class_<BemfVelocityTuner>(m, "BemfVelocityTuner",
        "Calibration-board BEMF→rad calibrator (fully automatic).")
        .def(py::init([](libstp::drive::Drive& drive,
                          libstp::odometry::IOdometry& odometry) {
                 return std::make_unique<BemfVelocityTuner>(drive, odometry);
             }),
             py::arg("drive"),
             py::arg("odometry"),
             py::keep_alive<1, 2>(),
             py::keep_alive<1, 3>(),
             "Construct a BemfVelocityTuner.")
        .def("tune",
             &BemfVelocityTuner::tune,
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Sweep PWM levels, measure ground-truth distance vs accumulated "
             "BEMF ticks per motor, derive per-motor ticks_to_rad + a linearity "
             "report. Applies + republishes when cfg.apply is set.");

    // ------------------------------------------------------------------
    // Orchestrator — AutoTuner
    // ------------------------------------------------------------------
    py::class_<AutoTuneConfig>(m, "AutoTuneConfig",
        "Aggregated configuration for the full auto-tune pipeline.")
        .def(py::init<>())
        .def_readwrite("tune_vel_lpf",         &AutoTuneConfig::tune_vel_lpf)
        .def_readwrite("tune_static_friction", &AutoTuneConfig::tune_static_friction)
        .def_readwrite("tune_firmware_pid",    &AutoTuneConfig::tune_firmware_pid)
        .def_readwrite("tune_encoder_cal",     &AutoTuneConfig::tune_encoder_cal)
        .def_readwrite("tune_characterize",    &AutoTuneConfig::tune_characterize)
        .def_readwrite("tune_velocity",        &AutoTuneConfig::tune_velocity)
        .def_readwrite("tune_motion",          &AutoTuneConfig::tune_motion)
        .def_readwrite("tune_tolerances",      &AutoTuneConfig::tune_tolerances)
        .def_readwrite("vel_lpf_cfg",          &AutoTuneConfig::vel_lpf_cfg)
        .def_readwrite("static_friction_cfg",  &AutoTuneConfig::static_friction_cfg)
        .def_readwrite("firmware_pid_cfg",     &AutoTuneConfig::firmware_pid_cfg)
        .def_readwrite("encoder_cal_cfg",      &AutoTuneConfig::encoder_cal_cfg)
        .def_readwrite("characterize_cfg",     &AutoTuneConfig::characterize_cfg)
        .def_readwrite("velocity_cfg",         &AutoTuneConfig::velocity_cfg)
        .def_readwrite("motion_cfg",           &AutoTuneConfig::motion_cfg)
        .def_readwrite("tolerance_cfg",        &AutoTuneConfig::tolerance_cfg)
        .def_readwrite("characterize_axes",    &AutoTuneConfig::characterize_axes)
        .def_readwrite("velocity_axes",        &AutoTuneConfig::velocity_axes)
        .def_readwrite("motion_params",        &AutoTuneConfig::motion_params)
        .def_readwrite("max_velocities",       &AutoTuneConfig::max_velocities)
        .def_readwrite("max_bemf_speeds",      &AutoTuneConfig::max_bemf_speeds);

    py::class_<AutoTuneResult>(m, "AutoTuneResult",
        "Aggregated results from the full auto-tune pipeline.")
        .def(py::init<>())
        .def_readonly("vel_lpf",         &AutoTuneResult::vel_lpf)
        .def_readonly("static_friction", &AutoTuneResult::static_friction)
        .def_readonly("firmware_pid",    &AutoTuneResult::firmware_pid)
        .def_readonly("encoder_cal",     &AutoTuneResult::encoder_cal)
        .def_readonly("characterize",    &AutoTuneResult::characterize)
        .def_readonly("velocity",        &AutoTuneResult::velocity)
        .def_readonly("motion",          &AutoTuneResult::motion)
        .def_readonly("tolerances",      &AutoTuneResult::tolerances)
        .def_readonly("vel_lpf_ran",         &AutoTuneResult::vel_lpf_ran)
        .def_readonly("static_friction_ran", &AutoTuneResult::static_friction_ran)
        .def_readonly("firmware_pid_ran",    &AutoTuneResult::firmware_pid_ran)
        .def_readonly("encoder_cal_ran",     &AutoTuneResult::encoder_cal_ran)
        .def_readonly("characterize_ran",    &AutoTuneResult::characterize_ran)
        .def_readonly("velocity_ran",        &AutoTuneResult::velocity_ran)
        .def_readonly("motion_ran",          &AutoTuneResult::motion_ran)
        .def_readonly("tolerances_ran",      &AutoTuneResult::tolerances_ran);

    py::class_<AutoTuner>(m, "AutoTuner",
        "C++ orchestrator that runs every tune phase in dependency order and "
        "persists results to the live Drive / IMotor / motion-config objects.")
        .def(py::init([](libstp::drive::Drive&                   drive,
                          libstp::odometry::IOdometry&            odometry,
                          libstp::motion::UnifiedMotionPidConfig& motion_pid_config) {
                 return std::make_unique<AutoTuner>(drive, odometry, motion_pid_config);
             }),
             py::arg("drive"),
             py::arg("odometry"),
             py::arg("motion_pid_config"),
             py::keep_alive<1, 2>(),
             py::keep_alive<1, 3>(),
             py::keep_alive<1, 4>(),
             "Construct an AutoTuner bound to the given drive, odometry, and "
             "motion PID config. All three references must outlive the AutoTuner.")
        .def("set_confirm_callback",
             &AutoTuner::setConfirmCallback,
             py::arg("callback"),
             "Install a callable invoked before every phase in tune_all(). "
             "The argument is a stable phase key ('vel_lpf', 'static_friction', "
             "'firmware_pid', 'encoder_cal', 'characterize:<axis>', "
             "'velocity:<axis>', 'motion:<param>', 'tolerances').")
        .def("tune_all",
             &AutoTuner::tuneAll,
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>(),
             "Run every enabled phase in dependency order. The confirm "
             "callback (if installed) fires before each phase.")
        .def("run_vel_lpf",
             &AutoTuner::runVelLpfPhase,
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>())
        .def("run_static_friction",
             &AutoTuner::runStaticFrictionPhase,
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>())
        .def("run_firmware_pid",
             &AutoTuner::runFirmwarePidPhase,
             py::arg("cfg"),
             py::arg("max_bemf_speeds") = std::optional<std::map<int, int>>{},
             py::call_guard<py::gil_scoped_release>())
        .def("run_encoder_calibration",
             &AutoTuner::runEncoderCalibrationPhase,
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>())
        .def("run_characterize",
             &AutoTuner::runCharacterizePhase,
             py::arg("axes"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>())
        .def("run_velocity",
             &AutoTuner::runVelocityPhase,
             py::arg("axes"),
             py::arg("max_velocities"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>())
        .def("run_motion",
             &AutoTuner::runMotionPhase,
             py::arg("params"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>())
        .def("run_tolerances",
             &AutoTuner::runTolerancesPhase,
             py::arg("motion_results"),
             py::arg("motion_cfg"),
             py::arg("cfg"),
             py::call_guard<py::gil_scoped_release>());
}
