// Python bindings for libstp::localization.
//
// Installs as ``raccoon.localization`` — exposes the pass-through Localization
// service plus its Observation / LocalizationConfig POD types. The C++ side
// owns the background tick thread; Python only sees the snapshot/observe API.

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "localization/localization.hpp"
#include "odometry/odometry.hpp"
#include "foundation/types.hpp"

#include <Eigen/Core>
#include <array>
#include <limits>
#include <memory>

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
              "world-pose service).";

    // Foundation registers Pose; odometry registers IOdometry. Both must be
    // imported before we hand instances of those types across the boundary.
    py::module_::import("raccoon.foundation");
    py::module_::import("raccoon.odometry");

    using libstp::localization::Localization;
    using libstp::localization::LocalizationConfig;
    using libstp::localization::Observation;

    // ──────────────────── Observation ────────────────────
    py::class_<Observation>(m, "Observation",
        "Soft-snap observation pushed by Resync-Steps.\n\n"
        "sigma is the per-axis (x, y, theta) measurement standard deviation. "
        "An axis with infinite sigma is left untouched by the pass-through "
        "service; finite-sigma axes are hard-snapped in Phase 2 (proper "
        "likelihood weighting lands with the particle filter in Phase 6).")
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
            });

    // ──────────────────── LocalizationConfig ────────────────────
    py::class_<LocalizationConfig>(m, "LocalizationConfig",
        "Configuration for the Localization background tick loop.")
        .def(py::init<>())
        .def(py::init([](int tick_period_ms) {
                 LocalizationConfig cfg;
                 cfg.tick_period_ms = tick_period_ms;
                 return cfg;
             }),
             py::arg("tick_period_ms") = 10)
        .def_readwrite("tick_period_ms", &LocalizationConfig::tick_period_ms,
                       "Background tick period in milliseconds (default 10 = 100 Hz).");

    // ──────────────────── Localization ────────────────────
    py::class_<Localization, std::shared_ptr<Localization>>(m, "Localization",
        "Thread-safe pass-through localization service.\n\n"
        "Owns a background thread that polls the supplied IOdometry at a "
        "fixed rate and accumulates the per-tick delta into a world-frame "
        "pose. ``observe()`` snaps selected axes for resync sites. The "
        "constructor calls ``start()``; the destructor calls ``stop()`` and "
        "joins the worker thread.")
        .def(py::init<std::shared_ptr<libstp::odometry::IOdometry>,
                      LocalizationConfig>(),
             py::arg("odometry"),
             py::arg("config") = LocalizationConfig{},
             // Holding the GIL during start() would be fine (no Python
             // callbacks), but keeping it released here is consistent with
             // stop() and avoids surprises if the worker is ever extended.
             py::call_guard<py::gil_scoped_release>())
        .def("get_pose", &Localization::getPose,
             "Snapshot of the current world pose. Thread-safe.")
        .def("observe", &Localization::observe, py::arg("observation"),
             "Apply a soft observation. Phase-2 hard-snaps finite-sigma axes; "
             "infinite-sigma axes are left untouched.")
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
             "this.");
}
