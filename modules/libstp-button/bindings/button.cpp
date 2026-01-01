//
// Created by eternalRose on 11/17/25.
//
#include <pybind11/pybind11.h>
#include "button/button.hpp"

namespace py = pybind11;

void init_button(py::module& m) {
    py::class_<libstp::button::Button>(m, "Button")
           .def_static("is_pressed", &libstp::button::Button::isPressed)
           .def_static("set_digital", &libstp::button::Button::setDigital)
           .def_static("wait_for_button_press", &libstp::button::Button::waitForButtonPress);

}
