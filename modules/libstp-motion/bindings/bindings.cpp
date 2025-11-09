#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_motion_base(py::module_& m);
void init_drive_straight(py::module_& m);
void init_move_to_pose(py::module_& m);

PYBIND11_MODULE(motion, m)
{
    m.doc() = "Python bindings for libstp-motion";

    init_motion_base(m);
    init_drive_straight(m);
    init_move_to_pose(m);
}
