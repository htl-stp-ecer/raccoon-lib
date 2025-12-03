//
// Created by tobias on 12/1/25.
//
#include "kinematics/kinematics.hpp"

std::vector<libstp::calibration::CalibrationResult> libstp::kinematics::IKinematics::calibrateMotors()
{
    return calibrateMotors(calibration::CalibrationConfig{});
}
