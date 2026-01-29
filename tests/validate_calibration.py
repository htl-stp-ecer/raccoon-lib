#!/usr/bin/env python3
"""Simple validation script for per-wheel distance calibration."""
import sys

def test_imports():
    """Test that all new imports work."""
    print("Testing imports...", end=" ")
    try:
        from libstp.step.calibrate_distance import (
            calibrate_distance,
            CalibrateDistance,
            CalibrationRequiredError,
            PerWheelCalibration,
            is_distance_calibrated,
            check_distance_calibration,
            reset_distance_calibration,
        )
        print("OK")
        return True
    except Exception as e:
        print(f"FAILED: {e}")
        return False

def test_calibration_functions():
    """Test calibration flag functions."""
    print("Testing calibration functions...", end=" ")
    try:
        from libstp.step.calibrate_distance import (
            is_distance_calibrated,
            check_distance_calibration,
            reset_distance_calibration,
            CalibrationRequiredError
        )

        reset_distance_calibration()
        assert is_distance_calibrated() == False, "Expected uncalibrated"

        try:
            check_distance_calibration()
            print("FAILED: Should have raised CalibrationRequiredError")
            return False
        except CalibrationRequiredError:
            pass  # Expected

        print("OK")
        return True
    except Exception as e:
        print(f"FAILED: {e}")
        return False

def test_per_wheel_calibration_dataclass():
    """Test PerWheelCalibration dataclass."""
    print("Testing PerWheelCalibration...", end=" ")
    try:
        from libstp.step.calibrate_distance import PerWheelCalibration

        result = PerWheelCalibration(
            motor_port=0,
            old_ticks_to_rad=0.001,
            new_ticks_to_rad=0.0012,
            delta_ticks=1000
        )

        assert result.motor_port == 0
        assert result.old_ticks_to_rad == 0.001
        assert result.new_ticks_to_rad == 0.0012
        assert result.delta_ticks == 1000

        print("OK")
        return True
    except Exception as e:
        print(f"FAILED: {e}")
        return False

def test_motor_set_calibration():
    """Test Motor.set_calibration exists."""
    print("Testing Motor.set_calibration...", end=" ")
    try:
        from libstp.hal import Motor

        assert hasattr(Motor, 'set_calibration'), "Motor missing set_calibration"
        assert hasattr(Motor, 'get_calibration'), "Motor missing get_calibration"

        print("OK")
        return True
    except Exception as e:
        print(f"FAILED: {e}")
        return False

def test_kinematics_wheel_radius():
    """Test kinematics get_wheel_radius."""
    print("Testing kinematics get_wheel_radius...", end=" ")
    try:
        from libstp.kinematics_differential import DifferentialKinematics
        from libstp.kinematics_mecanum import MecanumKinematics

        assert hasattr(DifferentialKinematics, 'get_wheel_radius'), "Differential missing get_wheel_radius"
        assert hasattr(MecanumKinematics, 'get_wheel_radius'), "Mecanum missing get_wheel_radius"

        print("OK")
        return True
    except Exception as e:
        print(f"FAILED: {e}")
        return False

def test_drive_wheel_radius():
    """Test Drive.get_wheel_radius."""
    print("Testing Drive.get_wheel_radius...", end=" ")
    try:
        from libstp.drive import Drive

        assert hasattr(Drive, 'get_wheel_radius'), "Drive missing get_wheel_radius"

        print("OK")
        return True
    except Exception as e:
        print(f"FAILED: {e}")
        return False

def test_drive_requires_calibration():
    """Test that drive_forward/backward require calibration."""
    print("Testing drive requires calibration...", end=" ")
    try:
        from libstp.step.calibrate_distance import reset_distance_calibration, CalibrationRequiredError
        from libstp.step.drive import drive_forward, drive_backward

        reset_distance_calibration()

        try:
            drive_forward(30.0)
            print("FAILED: drive_forward should require calibration")
            return False
        except CalibrationRequiredError:
            pass  # Expected

        try:
            drive_backward(30.0)
            print("FAILED: drive_backward should require calibration")
            return False
        except CalibrationRequiredError:
            pass  # Expected

        print("OK")
        return True
    except Exception as e:
        print(f"FAILED: {e}")
        return False

def test_robot_protocol():
    """Test RobotDefinitionsProtocol has drive_motors."""
    print("Testing RobotDefinitionsProtocol...", end=" ")
    try:
        from libstp.robot.api import RobotDefinitionsProtocol
        import typing

        hints = typing.get_type_hints(RobotDefinitionsProtocol)
        assert 'drive_motors' in hints, "Protocol missing drive_motors"

        print("OK")
        return True
    except Exception as e:
        print(f"FAILED: {e}")
        return False

def test_ticks_to_rad_calculation():
    """Test the mathematical calculation of ticks_to_rad."""
    print("Testing ticks_to_rad calculation...", end=" ")

    wheel_radius = 0.05  # 50mm
    measured_m = 0.30    # 30cm
    delta_ticks = 1000

    theta_rad = measured_m / wheel_radius
    new_ticks_to_rad = theta_rad / abs(delta_ticks)

    if abs(theta_rad - 6.0) > 0.001:
        print(f"FAILED: theta_rad should be 6.0, got {theta_rad}")
        return False

    if abs(new_ticks_to_rad - 0.006) > 0.00001:
        print(f"FAILED: new_ticks_to_rad should be 0.006, got {new_ticks_to_rad}")
        return False

    print("OK")
    return True


def main():
    """Run all validation tests."""
    print("=" * 60)
    print("Per-Wheel Distance Calibration Validation")
    print("=" * 60)

    tests = [
        test_imports,
        test_calibration_functions,
        test_per_wheel_calibration_dataclass,
        test_motor_set_calibration,
        test_kinematics_wheel_radius,
        test_drive_wheel_radius,
        test_drive_requires_calibration,
        test_robot_protocol,
        test_ticks_to_rad_calculation,
    ]

    passed = 0
    failed = 0

    for test in tests:
        if test():
            passed += 1
        else:
            failed += 1

    print("=" * 60)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 60)

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
