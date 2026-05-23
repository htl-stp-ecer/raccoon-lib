// Python bindings for libstp::localization.
//
// Installs as ``raccoon.localization`` — exposes the Localization particle
// filter service plus its Observation / LocalizationConfig POD types. The C++ side
// owns the background tick thread; Python only sees the snapshot/observe API.

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "localization/localization.hpp"
#include "localization/recorder.hpp"
#include "libstp/map/Geometry.hpp"
#include "libstp/map/WorldMap.hpp"
#include "hal/odometry.hpp"
#include "foundation/types.hpp"

#include <Eigen/Core>
#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

namespace py = pybind11;

namespace {

// Accept any 3-element Python iterable (list/tuple/np.array) for sigma so the
// Python side does not have to import Eigen. ``inf`` is supported and means
// "ignore this axis" — matches the doc-comment on Observation.
Eigen::Vector3d coerceSigma(const py::object& obj)
{
    // Direct Eigen cast first (covers numpy arrays via pybind11/eigen.h).
    try {
        return obj.cast<Eigen::Vector3d>();
    } catch (const py::cast_error&) {
        // Fall through to sequence-based extraction.
    }

    auto seq = obj.cast<py::sequence>();
    if (py::len(seq) != 3) {
        throw py::value_error("sigma must have exactly 3 elements (x, y, theta)");
    }
    return Eigen::Vector3d{
        seq[0].cast<double>(),
        seq[1].cast<double>(),
        seq[2].cast<double>(),
    };
}

}  // namespace

PYBIND11_MODULE(localization, m)
{
    m.doc() = "Python bindings for libstp-localization (Phase-2 pass-through "
              "upgraded with a Phase-6 particle-filter core).";

    // Foundation registers Pose; odometry registers IOdometry. Both must be
    // imported before we hand instances of those types across the boundary.
    py::module_::import("raccoon.foundation");
    py::module_::import("raccoon.hal");
    py::module_::import("raccoon.map");

    using libstp::localization::Localization;
    using libstp::localization::LocalizationConfig;
    using libstp::localization::Observation;
    using libstp::map::SensorOffset;
    using libstp::map::WorldMap;

    py::enum_<Observation::SurfaceKind>(m, "SurfaceKind")
        .value("LINE", Observation::SurfaceKind::Line)
        .value("WALL", Observation::SurfaceKind::Wall);

    py::class_<Observation::SurfaceMeasurement>(m, "SurfaceMeasurement",
        "Map-projected boolean sensor observation used as a soft particle likelihood.")
        .def(py::init<>())
        .def(py::init([](Observation::SurfaceKind kind,
                         const py::object& sensor,
                         bool detected,
                         double sigma_cm) {
                 Observation::SurfaceMeasurement measurement;
                 measurement.kind = kind;
                 if (py::hasattr(sensor, "forward_cm") && py::hasattr(sensor, "strafe_cm")) {
                     measurement.sensor = SensorOffset{
                         sensor.attr("forward_cm").cast<float>(),
                         sensor.attr("strafe_cm").cast<float>(),
                     };
                 } else {
                     measurement.sensor = sensor.cast<SensorOffset>();
                 }
                 measurement.detected = detected;
                 measurement.sigma_cm = sigma_cm;
                 return measurement;
             }),
             py::arg("kind"),
             py::arg("sensor"),
             py::arg("detected") = true,
             py::arg("sigma_cm") = 1.0)
        .def_readwrite("kind", &Observation::SurfaceMeasurement::kind)
        .def_property(
            "sensor",
            [](const Observation::SurfaceMeasurement& self) {
                return py::make_tuple(self.sensor.forwardCm, self.sensor.strafeCm);
            },
            [](Observation::SurfaceMeasurement& self, const py::object& sensor) {
                if (!py::hasattr(sensor, "forward_cm") || !py::hasattr(sensor, "strafe_cm")) {
                    self.sensor = sensor.cast<SensorOffset>();
                    return;
                }
                self.sensor.forwardCm = sensor.attr("forward_cm").cast<float>();
                self.sensor.strafeCm = sensor.attr("strafe_cm").cast<float>();
            })
        .def_property_readonly("forward_cm",
                               [](const Observation::SurfaceMeasurement& self) {
                                   return self.sensor.forwardCm;
                               })
        .def_property_readonly("strafe_cm",
                               [](const Observation::SurfaceMeasurement& self) {
                                   return self.sensor.strafeCm;
                               })
        .def_readwrite("detected", &Observation::SurfaceMeasurement::detected)
        .def_readwrite("sigma_cm", &Observation::SurfaceMeasurement::sigma_cm);

    // ──────────────────── Observation ────────────────────
    py::class_<Observation>(m, "Observation",
        "Soft-snap observation pushed by Resync-Steps.\n\n"
        "sigma is the per-axis (x, y, theta) measurement standard deviation. "
        "An axis with infinite sigma is left untouched.")
        .def(py::init<>())
        .def(py::init([](const libstp::foundation::Pose& pose,
                         const py::object& sigma) {
                 Observation obs;
                 obs.pose = pose;
                 obs.sigma = coerceSigma(sigma);
                 return obs;
             }),
             py::arg("pose"),
             py::arg("sigma") = py::make_tuple(1e-3, 1e-3, 1e-3),
             "Construct an Observation. ``sigma`` accepts any 3-element "
             "sequence (list, tuple, numpy array). Use ``math.inf`` on an "
             "axis to leave it untouched.")
        .def_readwrite("pose", &Observation::pose)
        .def_property(
            "sigma",
            [](const Observation& self) {
                // Return as a plain tuple so callers don't need numpy/Eigen.
                return py::make_tuple(self.sigma.x(), self.sigma.y(), self.sigma.z());
            },
            [](Observation& self, const py::object& value) {
                self.sigma = coerceSigma(value);
            })
        .def_readwrite("surface_measurements", &Observation::surface_measurements);

    // ──────────────────── LocalizationConfig ────────────────────
    py::class_<LocalizationConfig>(m, "LocalizationConfig",
        "Configuration for the Localization background tick loop and particle filter.")
        .def(py::init<>())
        .def(py::init([](int tick_period_ms,
                         int particle_count,
                         double process_translation_noise_m,
                         double process_translation_noise_per_m,
                         double process_heading_noise_rad,
                         double process_heading_noise_per_rad,
                         double observation_injection_ratio,
                         double resample_effective_sample_ratio,
                         uint32_t rng_seed) {
                 LocalizationConfig cfg;
                 cfg.tick_period_ms = tick_period_ms;
                 cfg.particle_count = particle_count;
                 cfg.process_translation_noise_m = process_translation_noise_m;
                 cfg.process_translation_noise_per_m = process_translation_noise_per_m;
                 cfg.process_heading_noise_rad = process_heading_noise_rad;
                 cfg.process_heading_noise_per_rad = process_heading_noise_per_rad;
                 cfg.observation_injection_ratio = observation_injection_ratio;
                 cfg.resample_effective_sample_ratio = resample_effective_sample_ratio;
                 cfg.rng_seed = rng_seed;
                 return cfg;
             }),
             py::arg("tick_period_ms") = 10,
             py::arg("particle_count") = 128,
             py::arg("process_translation_noise_m") = 0.002,
             py::arg("process_translation_noise_per_m") = 0.02,
             py::arg("process_heading_noise_rad") = 0.01,
             py::arg("process_heading_noise_per_rad") = 0.05,
             py::arg("observation_injection_ratio") = 0.35,
             py::arg("resample_effective_sample_ratio") = 0.5,
             py::arg("rng_seed") = 0x5EED1234u)
        .def_readwrite("tick_period_ms", &LocalizationConfig::tick_period_ms,
                       "Background tick period in milliseconds (default 10 = 100 Hz).")
        .def_readwrite("particle_count", &LocalizationConfig::particle_count)
        .def_readwrite("process_translation_noise_m",
                       &LocalizationConfig::process_translation_noise_m)
        .def_readwrite("process_translation_noise_per_m",
                       &LocalizationConfig::process_translation_noise_per_m)
        .def_readwrite("process_heading_noise_rad",
                       &LocalizationConfig::process_heading_noise_rad)
        .def_readwrite("process_heading_noise_per_rad",
                       &LocalizationConfig::process_heading_noise_per_rad)
        .def_readwrite("observation_injection_ratio",
                       &LocalizationConfig::observation_injection_ratio)
        .def_readwrite("resample_effective_sample_ratio",
                       &LocalizationConfig::resample_effective_sample_ratio)
        .def_readwrite("rng_seed", &LocalizationConfig::rng_seed);

    // ──────────────────── Localization ────────────────────
    py::class_<Localization, std::shared_ptr<Localization>>(m, "Localization",
        "Thread-safe particle-filter localization service.\n\n"
        "Owns a background thread that polls the supplied IOdometry at a "
        "fixed rate and propagates a particle cloud from the per-tick "
        "odometry delta. ``observe()`` injects sigma-weighted absolute pose "
        "observations for resync sites. The "
        "constructor calls ``start()``; the destructor calls ``stop()`` and "
        "joins the worker thread.")
        .def(py::init([](std::shared_ptr<libstp::odometry::IOdometry> odometry,
                         LocalizationConfig config,
                         const py::object& table_map) {
                 std::optional<WorldMap> copiedMap = std::nullopt;
                 if (!table_map.is_none()) {
                     copiedMap = table_map.cast<WorldMap>();
                 }
                 return std::make_shared<Localization>(
                     std::move(odometry), config, std::move(copiedMap));
             }),
             py::arg("odometry"),
             py::arg("config") = LocalizationConfig{},
             py::arg("table_map") = py::none(),
             // Holding the GIL during start() would be fine (no Python
             // callbacks), but keeping it released here is consistent with
             // stop() and avoids surprises if the worker is ever extended.
             py::call_guard<py::gil_scoped_release>())
        .def("get_pose", &Localization::getPose,
             "Snapshot of the current world pose. Thread-safe.")
        .def("observe", &Localization::observe, py::arg("observation"),
             "Apply a sigma-weighted pose observation. Infinite-sigma axes "
             "are left untouched.")
        .def("start", &Localization::start,
             py::call_guard<py::gil_scoped_release>(),
             "Idempotent. The constructor already calls this; tests may "
             "re-call.")
        .def("stop", &Localization::stop,
             // ``stop()`` joins the worker thread — release the GIL so any
             // future Python-side callbacks from the worker (or simply other
             // Python threads) can make progress while we wait.
             py::call_guard<py::gil_scoped_release>(),
             "Idempotent. Joins the worker thread. The destructor also calls "
             "this.")
        .def("enable_recording",
             [](Localization& self,
                const std::string& path,
                double record_hz,
                const std::string& robot_json,
                const std::string& sensors_json,
                const std::string& table_map_json,
                const std::string& notes,
                int particle_count_hint,
                double tick_hz) {
                 libstp::localization::RecorderConfig cfg;
                 cfg.path = path;
                 cfg.record_hz = record_hz;
                 cfg.tick_hz = tick_hz;
                 cfg.particle_count_hint = particle_count_hint;
                 cfg.robot_json = robot_json;
                 cfg.sensors_json = sensors_json;
                 cfg.table_map_json = table_map_json;
                 cfg.notes = notes;
                 return self.enableRecording(std::move(cfg));
             },
             py::arg("path"),
             py::arg("record_hz") = 20.0,
             py::arg("robot_json") = std::string{"null"},
             py::arg("sensors_json") = std::string{"[]"},
             py::arg("table_map_json") = std::string{"null"},
             py::arg("notes") = std::string{},
             py::arg("particle_count_hint") = 128,
             py::arg("tick_hz") = 100.0,
             py::call_guard<py::gil_scoped_release>(),
             "Enable post-run JSONL recording of localization state. Returns "
             "True if the file was opened successfully. On failure (e.g. "
             "unwritable path) returns False and localization continues "
             "without recording — never raises.");

    // Auto-wire helper invoked from Python (api.py::_build_localization).
    // Reads LIBSTP_RECORD_LOCALIZATION / LIBSTP_RECORDING_PATH /
    // LIBSTP_RECORDING_HZ and calls enable_recording. Keeps the
    // env-var-parsing policy in C++ so the spec lives next to the recorder.
    m.def("_auto_enable_recording",
          [](Localization& loc,
             const std::string& robot_json,
             const std::string& sensors_json,
             const std::string& table_map_json,
             int particle_count_hint,
             double tick_hz) {
              const char* flag = std::getenv("LIBSTP_RECORD_LOCALIZATION");
              if (flag == nullptr || std::strlen(flag) == 0 ||
                  std::strcmp(flag, "0") == 0 || std::strcmp(flag, "false") == 0 ||
                  std::strcmp(flag, "FALSE") == 0) {
                  return false;
              }
              const char* path = std::getenv("LIBSTP_RECORDING_PATH");
              if (path == nullptr || std::strlen(path) == 0) {
                  std::cerr << "[localization::Recorder] LIBSTP_RECORD_LOCALIZATION is set "
                               "but LIBSTP_RECORDING_PATH is empty — disabled.\n";
                  return false;
              }
              double hz = 20.0;
              if (const char* hz_env = std::getenv("LIBSTP_RECORDING_HZ");
                  hz_env != nullptr && std::strlen(hz_env) > 0) {
                  try {
                      hz = std::stod(hz_env);
                  } catch (...) {
                      // keep default
                  }
              }
              libstp::localization::RecorderConfig cfg;
              cfg.path = path;
              cfg.record_hz = hz;
              cfg.tick_hz = tick_hz;
              cfg.particle_count_hint = particle_count_hint;
              cfg.robot_json = robot_json;
              cfg.sensors_json = sensors_json;
              cfg.table_map_json = table_map_json;
              return loc.enableRecording(std::move(cfg));
          },
          py::arg("localization"),
          py::arg("robot_json") = std::string{"null"},
          py::arg("sensors_json") = std::string{"[]"},
          py::arg("table_map_json") = std::string{"null"},
          py::arg("particle_count_hint") = 128,
          py::arg("tick_hz") = 100.0,
          py::call_guard<py::gil_scoped_release>(),
          "Internal: enable LocalizationRecorder iff LIBSTP_RECORD_LOCALIZATION "
          "is truthy. Returns True if recording started.");
}
