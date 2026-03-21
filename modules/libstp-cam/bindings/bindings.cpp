//
// Python module entry point for libstp-cam.
//

#include <pybind11/pybind11.h>
namespace py = pybind11;

void init_cam_sensor(py::module& m);

PYBIND11_MODULE(cam, m) {
    m.doc() = "Python bindings for libstp-cam";

    init_cam_sensor(m);
}
