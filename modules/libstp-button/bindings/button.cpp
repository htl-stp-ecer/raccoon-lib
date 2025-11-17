//
// Created by eternalRose on 11/17/25.
//
#include <pybind11/pybind11.h>
#include "../include/button.hpp"
#include "../include/button_wraper.hpp"

namespace py = pybind11;

void init_button(py::module& m) {

    py::class_<libstp::button::ButtonWrapper>(m, "Button")
        .def(py::init<>())
        .def("is_pressed", &libstp::button::ButtonWrapper::isPressed)
        .def("set_digital", &libstp::button::ButtonWrapper::setDigital)
        .def("wait_for_button", &libstp::button::ButtonWrapper::waitForButtonPress);
}
