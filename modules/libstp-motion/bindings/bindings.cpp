#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_motion_base(py::module_& m);
void init_linear_motion(py::module_& m);
void init_diagonal_motion(py::module_& m);
void init_turn(py::module_& m);
void init_arc_motion(py::module_& m);

PYBIND11_MODULE(motion, m)
{
    m.doc() = "Python bindings for libstp-motion";

    init_motion_base(m);
    init_linear_motion(m);
    init_diagonal_motion(m);
    init_turn(m);
    init_arc_motion(m);
}
