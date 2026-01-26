//
// Created by eternalRose on 11/17/25.
//
#include <pybind11/pybind11.h>
#include "button/button.hpp"
#include "hal/Digital.hpp"

namespace py = pybind11;

void init_button(py::module& m) {
    // Expose singleton methods as module-level functions
    m.def("is_pressed", []() {
        return libstp::button::Button::instance().isPressed();
    }, "Check if the button is pressed");

    m.def("set_digital", [](int port) {
        libstp::button::Button::instance().setDigital(port);
    }, "Set the digital sensor port", py::arg("port"));

    m.def("set_digital", [](libstp::hal::digital::DigitalSensor& sensor) {
        libstp::button::Button::instance().setDigital(sensor.port);
    }, "Set the digital sensor from existing sensor", py::arg("sensor"));

    m.def("wait_for_button_press", []() {
        libstp::button::Button::instance().waitForButtonPress();
    }, "Wait for the button to be pressed");
}
