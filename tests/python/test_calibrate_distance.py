"""Tests for per-wheel distance calibration."""
import pytest
from unittest.mock import Mock, MagicMock, AsyncMock, patch
import asyncio
import sys


def libstp_available():
    """Check if raccoon module is available."""
    try:
        import raccoon
        return True
    except ImportError:
        return False


# Mark for tests requiring raccoon native module
requires_libstp = pytest.mark.skipif(
    not libstp_available(),
    reason="raccoon native module not installed"
)


class TestCalibrationFunctions:
    """Test the module-level calibration functions."""

    @requires_libstp
    def test_is_distance_calibrated_initial_false(self):
        """Test that calibration flag is initially False."""
        from raccoon.step.calibrate_distance import reset_distance_calibration, is_distance_calibrated
        reset_distance_calibration()
        assert is_distance_calibrated() is False

    @requires_libstp
    def test_check_distance_calibration_raises_when_not_calibrated(self):
        """Test that check_distance_calibration raises when not calibrated."""
        from raccoon.step.calibrate_distance import (
            reset_distance_calibration,
            check_distance_calibration,
            CalibrationRequiredError
        )
        reset_distance_calibration()
        with pytest.raises(CalibrationRequiredError):
            check_distance_calibration()

    @requires_libstp
    def test_reset_distance_calibration(self):
        """Test that reset clears the calibration flag."""
        from raccoon.step.calibrate_distance import reset_distance_calibration, is_distance_calibrated
        reset_distance_calibration()
        assert is_distance_calibrated() is False


class TestPerWheelCalibration:
    """Test the PerWheelCalibration dataclass."""

    @requires_libstp
    def test_per_wheel_calibration_creation(self):
        """Test creating a PerWheelCalibration instance."""
        from raccoon.step.calibrate_distance import PerWheelCalibration

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


class TestCalibrateDistanceStep:
    """Test the CalibrateDistance step class."""

    @requires_libstp
    def test_calibrate_distance_factory(self):
        """Test the calibrate_distance factory function."""
        from raccoon.step.calibrate_distance import calibrate_distance, CalibrateDistance

        step = calibrate_distance(distance_cm=50.0, calibrate_light_sensors=True)

        assert isinstance(step, CalibrateDistance)
        assert step.calibration_distance_cm == 50.0
        assert step.calibrate_light_sensors is True

    @requires_libstp
    def test_calibrate_distance_defaults(self):
        """Test default values for calibrate_distance."""
        from raccoon.step.calibrate_distance import calibrate_distance

        step = calibrate_distance()

        assert step.calibration_distance_cm == 30.0
        assert step.calibrate_light_sensors is False

    @requires_libstp
    def test_generate_signature(self):
        """Test the signature generation."""
        from raccoon.step.calibrate_distance import calibrate_distance

        step = calibrate_distance(distance_cm=25.0, calibrate_light_sensors=True)
        sig = step._generate_signature()

        assert "25.0" in sig
        assert "True" in sig

    @requires_libstp
    def test_parse_measured_distance_with_prefix(self):
        """Test parsing measured distance from response with prefix."""
        from raccoon.step.calibrate_distance import CalibrateDistance

        step = CalibrateDistance()
        result = step._parse_measured_distance("measured_distance=28.5")

        assert result == 28.5

    @requires_libstp
    def test_parse_measured_distance_raw_number(self):
        """Test parsing measured distance from raw number."""
        from raccoon.step.calibrate_distance import CalibrateDistance

        step = CalibrateDistance()
        result = step._parse_measured_distance("32.1")

        assert result == 32.1

    @requires_libstp
    def test_parse_measured_distance_invalid(self):
        """Test parsing invalid measured distance."""
        from raccoon.step.calibrate_distance import CalibrateDistance

        step = CalibrateDistance()
        result = step._parse_measured_distance("invalid input")

        assert result is None


class TestTicksToRadCalculation:
    """Test the ticks_to_rad calculation logic - pure Python, no imports needed."""

    def test_ticks_to_rad_calculation(self):
        """Test the mathematical calculation of new ticks_to_rad.

        Formula: new_ticks_to_rad = (measured_m / wheel_radius) / |delta_ticks|

        Example:
        - wheel_radius = 0.05m (50mm)
        - measured_distance = 0.30m (30cm)
        - delta_ticks = 1000

        theta_rad = 0.30 / 0.05 = 6.0 radians
        new_ticks_to_rad = 6.0 / 1000 = 0.006 rad/tick
        """
        wheel_radius = 0.05  # 50mm
        measured_m = 0.30    # 30cm
        delta_ticks = 1000

        theta_rad = measured_m / wheel_radius
        new_ticks_to_rad = theta_rad / abs(delta_ticks)

        assert abs(theta_rad - 6.0) < 0.001
        assert abs(new_ticks_to_rad - 0.006) < 0.00001

    def test_ticks_to_rad_scale_factor(self):
        """Test that the scale factor calculation works correctly.

        If old_ticks_to_rad is 0.004 and new is 0.006,
        scale_factor = 0.006 / 0.004 = 1.5 (robot was under-reporting distance by 33%)
        """
        old_ticks_to_rad = 0.004
        new_ticks_to_rad = 0.006

        scale_factor = new_ticks_to_rad / old_ticks_to_rad

        assert abs(scale_factor - 1.5) < 0.001

    def test_calibration_corrects_distance_under_reporting(self):
        """Test that calibration correctly adjusts for under-reporting.

        Scenario: Robot thinks it drove 30cm but actually drove 25cm.
        The current ticks_to_rad overestimates distance.

        wheel_radius = 0.05m
        requested_cm = 30cm (what robot thought it drove)
        measured_cm = 25cm (what was actually traveled)

        If delta_ticks = 1000 with old ticks_to_rad that gave 30cm,
        then old ticks_to_rad = (0.30 / 0.05) / 1000 = 0.006

        With measured 25cm:
        new ticks_to_rad = (0.25 / 0.05) / 1000 = 0.005

        Scale factor = 0.005 / 0.006 = 0.833...
        """
        wheel_radius = 0.05
        requested_m = 0.30
        measured_m = 0.25
        delta_ticks = 1000

        # What the old calibration would have been (to produce requested distance)
        old_ticks_to_rad = (requested_m / wheel_radius) / delta_ticks

        # What the new calibration should be (to match measured distance)
        new_ticks_to_rad = (measured_m / wheel_radius) / delta_ticks

        scale_factor = new_ticks_to_rad / old_ticks_to_rad

        # Scale should be less than 1 since robot over-reported distance
        assert scale_factor < 1.0
        assert abs(scale_factor - (25/30)) < 0.001

    def test_calibration_corrects_distance_over_reporting(self):
        """Test that calibration correctly adjusts for over-reporting.

        Scenario: Robot thinks it drove 30cm but actually drove 35cm.
        The current ticks_to_rad underestimates distance.
        """
        wheel_radius = 0.05
        requested_m = 0.30
        measured_m = 0.35
        delta_ticks = 1000

        old_ticks_to_rad = (requested_m / wheel_radius) / delta_ticks
        new_ticks_to_rad = (measured_m / wheel_radius) / delta_ticks

        scale_factor = new_ticks_to_rad / old_ticks_to_rad

        # Scale should be greater than 1 since robot under-reported distance
        assert scale_factor > 1.0
        assert abs(scale_factor - (35/30)) < 0.001

    def test_per_wheel_independent_calibration(self):
        """Test that different wheels can have different calibrations.

        This is the key feature - each wheel gets its own ticks_to_rad
        based on its encoder ticks, accounting for wheel diameter differences.
        """
        wheel_radius = 0.05
        measured_m = 0.30

        # Left wheel: slightly smaller effective diameter, more ticks
        left_delta_ticks = 1050
        # Right wheel: slightly larger effective diameter, fewer ticks
        right_delta_ticks = 950

        left_ticks_to_rad = (measured_m / wheel_radius) / abs(left_delta_ticks)
        right_ticks_to_rad = (measured_m / wheel_radius) / abs(right_delta_ticks)

        # They should be different
        assert left_ticks_to_rad != right_ticks_to_rad

        # Left should be smaller (more ticks = smaller conversion factor)
        assert left_ticks_to_rad < right_ticks_to_rad

        # After calibration, both wheels should report the same distance
        # for their respective tick counts
        left_distance = left_delta_ticks * left_ticks_to_rad * wheel_radius
        right_distance = right_delta_ticks * right_ticks_to_rad * wheel_radius

        assert abs(left_distance - measured_m) < 0.001
        assert abs(right_distance - measured_m) < 0.001


class TestMotorCalibrationBinding:
    """Test the Motor.set_calibration binding."""

    @requires_libstp
    def test_motor_set_calibration_exists(self):
        """Test that set_calibration method exists on Motor."""
        from raccoon.hal import Motor

        # Check that the method exists
        assert hasattr(Motor, 'set_calibration')

    @requires_libstp
    def test_motor_get_calibration_exists(self):
        """Test that get_calibration method exists on Motor."""
        from raccoon.hal import Motor

        assert hasattr(Motor, 'get_calibration')


class TestKinematicsWheelRadius:
    """Test the getWheelRadius method on kinematics classes."""

    @requires_libstp
    def test_differential_kinematics_get_wheel_radius_binding(self):
        """Test that DifferentialKinematics has get_wheel_radius binding."""
        from raccoon.kinematics_differential import DifferentialKinematics

        assert hasattr(DifferentialKinematics, 'get_wheel_radius')

    @requires_libstp
    def test_mecanum_kinematics_get_wheel_radius_binding(self):
        """Test that MecanumKinematics has get_wheel_radius binding."""
        from raccoon.kinematics_mecanum import MecanumKinematics

        assert hasattr(MecanumKinematics, 'get_wheel_radius')


class TestDriveWheelRadius:
    """Test the getWheelRadius method on Drive class."""

    @requires_libstp
    def test_drive_get_wheel_radius_binding(self):
        """Test that Drive has get_wheel_radius binding."""
        from raccoon.drive import Drive

        assert hasattr(Drive, 'get_wheel_radius')


class TestDriveForwardCalibrationCheck:
    """Test that drive_forward requires calibration."""

    @requires_libstp
    def test_drive_forward_requires_calibration(self):
        """Test that drive_forward raises CalibrationRequiredError when not calibrated."""
        from raccoon.step.calibrate_distance import reset_distance_calibration, CalibrationRequiredError
        from raccoon.step.drive import drive_forward

        reset_distance_calibration()

        with pytest.raises(CalibrationRequiredError):
            drive_forward(30.0)

    @requires_libstp
    def test_drive_backward_requires_calibration(self):
        """Test that drive_backward raises CalibrationRequiredError when not calibrated."""
        from raccoon.step.calibrate_distance import reset_distance_calibration, CalibrationRequiredError
        from raccoon.step.drive import drive_backward

        reset_distance_calibration()

        with pytest.raises(CalibrationRequiredError):
            drive_backward(30.0)


class TestDriveGetMotors:
    """Test the Drive.get_motors() method."""

    @requires_libstp
    def test_drive_get_motors_binding(self):
        """Test that Drive has get_motors binding."""
        from raccoon.drive import Drive

        assert hasattr(Drive, 'get_motors')


class TestExportedSymbols:
    """Test that new symbols are properly exported."""

    @requires_libstp
    def test_step_module_exports(self):
        """Test that step module exports the new calibration functions."""
        from raccoon.step import (
            calibrate_distance,
            CalibrateDistance,
            CalibrationRequiredError,
            PerWheelCalibration,
            is_distance_calibrated,
            check_distance_calibration,
            reset_distance_calibration,
        )

        # All should be importable
        assert calibrate_distance is not None
        assert CalibrateDistance is not None
        assert CalibrationRequiredError is not None
        assert PerWheelCalibration is not None
        assert is_distance_calibrated is not None
        assert check_distance_calibration is not None
        assert reset_distance_calibration is not None
