//
// Created by eternalRose on 11/17/25.
//
#include <pybind11/pybind11.h>
#include "button/button.hpp"

namespace py = pybind11;

void init_button(py::module& m) {
    m.def("is_pressed", &libstp::button::Button::isPressed, "Check if button is pressed");
    m.def("set_digital", &libstp::button::Button::setDigital, "Set digital port for button", py::arg("port"));
    m.def("wait_for_button_press", &libstp::button::Button::waitForButtonPress, "Wait for button press");
}
