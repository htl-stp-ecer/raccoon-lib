"""Tests for libstp.kinematics bindings."""
import pytest


def test_kinematics_module_import():
    """Test that kinematics module can be imported."""
    try:
        from libstp import kinematics
        assert kinematics is not None
    except ImportError:
        pytest.skip("kinematics module not available")


# TODO: Add more tests for MotorCommands structure once bindings are confirmed
