//
// pybind11 bindings for CamSensor.
//

#include "CamSensor.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_cam_sensor(py::module& m) {
    py::class_<libstp::cam::CamSensor>(m, "CamSensor")
        .def(py::init<>())
        .def("is_detected", &libstp::cam::CamSensor::isDetected,
             py::arg("label"))
        .def("get_blob_x", &libstp::cam::CamSensor::getBlobX,
             py::arg("label"))
        .def("get_blob_y", &libstp::cam::CamSensor::getBlobY,
             py::arg("label"))
        .def("get_blob_width", &libstp::cam::CamSensor::getBlobWidth,
             py::arg("label"))
        .def("get_blob_height", &libstp::cam::CamSensor::getBlobHeight,
             py::arg("label"))
        .def("get_blob_area", &libstp::cam::CamSensor::getBlobArea,
             py::arg("label"))
        .def("get_confidence", &libstp::cam::CamSensor::getConfidence,
             py::arg("label"))
        .def("get_detected_labels", &libstp::cam::CamSensor::getDetectedLabels)
        .def_property_readonly("frame_width", &libstp::cam::CamSensor::getFrameWidth)
        .def_property_readonly("frame_height", &libstp::cam::CamSensor::getFrameHeight);
}
