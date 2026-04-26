"""Tests for raccoon.foundation type bindings."""

from __future__ import annotations


def test_chassis_velocity_default():
    """Test default construction of ChassisVelocity."""
    from raccoon.foundation import ChassisVelocity

    cv = ChassisVelocity()
    assert cv.vx == 0.0
    assert cv.vy == 0.0
    assert cv.wz == 0.0


def test_chassis_velocity_construction():
    """Test parameterized construction of ChassisVelocity."""
    from raccoon.foundation import ChassisVelocity

    cv = ChassisVelocity(1.0, 2.0, 3.0)
    assert cv.vx == 1.0
    assert cv.vy == 2.0
    assert cv.wz == 3.0


def test_chassis_velocity_modification(chassis_velocity):
    """Test modifying ChassisVelocity fields."""
    cv = chassis_velocity(1.0, 2.0, 3.0)
    cv.vx = 5.0
    assert cv.vx == 5.0
