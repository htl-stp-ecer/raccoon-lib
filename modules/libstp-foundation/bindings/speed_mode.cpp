#include <pybind11/pybind11.h>

#include "foundation/speed_mode_context.hpp"

namespace py = pybind11;

void init_speed_mode(py::module_& m)
{
    using libstp::foundation::SpeedModeContext;

    // Expose the singleton via free functions on the foundation module so
    // Python code (in particular the `SetSpeedMode` step) can flip the flag
    // after the firmware ACK has arrived. Binding the class itself is
    // intentionally avoided — `SpeedModeContext` is a process-wide
    // singleton and the Python side never needs to construct or pass
    // around instances.
    m.def(
        "is_speed_mode_enabled",
        [] { return SpeedModeContext::instance().isSpeedModeEnabled(); },
        "Return True if SpeedMode is currently active (BEMF disabled).");

    m.def(
        "set_speed_mode_enabled",
        [](bool enabled) { SpeedModeContext::instance().setSpeedModeEnabled(enabled); },
        py::arg("enabled"),
        "Set the process-wide SpeedMode flag. Should only be called by "
        "the `SetSpeedMode` step after the firmware has acknowledged the "
        "BEMF-disable command; calling it directly bypasses the firmware "
        "handshake and leaves the library/firmware state inconsistent.");
}
