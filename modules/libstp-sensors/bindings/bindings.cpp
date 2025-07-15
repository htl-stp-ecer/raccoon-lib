//
// Created by tobias on 12/29/24.
//

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "sensors/ir_light_sensor.hpp"
#include "sensors/sensor.hpp"

namespace py = pybind11;

namespace libstp::sensor
{
    inline void createSensorBindings(py::module_& m)
    {
        py::class_<DistanceSensor, hal::analog::AnalogSensor, std::shared_ptr<DistanceSensor>>(m, "DistanceSensor", R"pbdoc(
            Represents a distance sensor based on analog measurements.
        )pbdoc")
            .def(py::init<int>(), py::arg("port"), R"pbdoc(
                Initializes a DistanceSensor instance.

                Args:
                    port (int): The port number of the sensor.
            )pbdoc")
            .def("get_distance", &DistanceSensor::getDistance, R"pbdoc(
                Gets the measured distance.

                Returns:
                    float: The measured distance in appropriate units.
            )pbdoc");

        py::class_<LightSensor, hal::analog::AnalogSensor, std::shared_ptr<LightSensor>>(m, "LightSensor", R"pbdoc(
            Represents a light sensor with calibration functionality.
        )pbdoc")
            .def(py::init<int, float>(), py::arg("port"), py::arg("calibration_factor") = 0.5, R"pbdoc(
                Initializes a LightSensor instance.

                Args:
                    port (int): The port number of the sensor.
                    calibration_factor (float): The calibration factor.
            )pbdoc")
            .def_property("white_threshold", [](const LightSensor& sensor) { return sensor.whiteThreshold; },
                          [](LightSensor& sensor, const int threshold) { sensor.whiteThreshold = threshold; })
            .def_property("black_threshold", [](const LightSensor& sensor) { return sensor.blackThreshold; },
                          [](LightSensor& sensor, const int threshold) { sensor.blackThreshold = threshold; })
            .def_property("white_mean", [](const LightSensor& sensor) { return sensor.whiteMean; },
                          [](LightSensor& sensor, const float mean) { sensor.whiteMean = mean; })
            .def_property("black_mean", [](const LightSensor& sensor) { return sensor.blackMean; },
                          [](LightSensor& sensor, const float mean) { sensor.blackMean = mean; })
            .def_property("white_std_dev", [](const LightSensor& sensor) { return sensor.whiteStdDev; },
                          [](LightSensor& sensor, const float std_dev) { sensor.whiteStdDev = std_dev; })
            .def_property("black_std_dev", [](const LightSensor& sensor) { return sensor.blackStdDev; },
                          [](LightSensor& sensor, const float std_dev) { sensor.blackStdDev = std_dev; })
            .def("calibrate", &LightSensor::calibrate, py::arg("white_value"), py::arg("black_value"), R"pbdoc(
                Calibrates the light sensor.

                Args:
                    white_value (int): The white calibration value.
                    black_value (int): The black calibration value.

                Returns:
                    bool: True if successful, False otherwise.
            )pbdoc")
            .def("wait_for_light", &LightSensor::wait_for_light, R"pbdoc(
                Waits for the light sensor to detect light.
            )pbdoc")
            .def("is_on_white", &LightSensor::isOnWhite, R"pbdoc(
                Checks if the sensor is on a white surface.

                Returns:
                    bool: True if on white, False otherwise.
            )pbdoc")
            .def("is_on_black", &LightSensor::isOnBlack, R"pbdoc(
                Checks if the sensor is on a black surface.

                Returns:
                    bool: True if on black, False otherwise.
            )pbdoc");

        py::class_<IrLightSensor, LightSensor, std::shared_ptr<IrLightSensor>>(m, "IrLightSensor", R"pbdoc(
            Represents an IR light sensor with enhanced calibration functionality.
        )pbdoc")
            .def(py::init<int, float>(), py::arg("port"), py::arg("calibration_factor") = 0.3f, R"pbdoc(
                Initializes an IrLightSensor instance.

                Args:
                    port (int): The port number of the sensor.
                    calibration_factor (float): The calibration factor.
            )pbdoc");

        m.def("calibrate_light_sensors", &calibrateLightSensors, py::arg("light_sensors"), R"pbdoc(
            Calibrates a list of light sensors.

            Args:
                light_sensors (List[LightSensor]): The light sensors to calibrate.
        )pbdoc");

        m.def("are_on_black", &areOnBlack, py::arg("left_sensor"), py::arg("right_sensor"), R"pbdoc(
            Checks if both sensors are on black.

            Args:
                left_sensor (LightSensor): The left light sensor.
                right_sensor (LightSensor): The right light sensor.

            Returns:
                bool: True if both are on black, False otherwise.
        )pbdoc");

        m.def("are_on_white", &areOnWhite, py::arg("left_sensor"), py::arg("right_sensor"), R"pbdoc(
            Checks if both sensors are on white.

            Args:
                left_sensor (LightSensor): The left light sensor.
                right_sensor (LightSensor): The right light sensor.

            Returns:
                bool: True if both are on white, False otherwise.
        )pbdoc");

        m.def("wait_for_button_click", &waitForButtonClick, R"pbdoc(
            Waits for a button click.
        )pbdoc");


        m.def("is_button_clicked", &isButtonClicked, R"pbdoc(
            Checks if a button is clicked.

            Returns:
                bool: True if clicked, False otherwise.
        )pbdoc");
    }
}


PYBIND11_MODULE(sensors, m) {
    m.doc() = "Python bindings for libstp-sensors";

    libstp::sensor::createSensorBindings(m);
}

