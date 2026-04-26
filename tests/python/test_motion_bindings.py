"""Tests for raccoon.motion bindings.

These exercise the C++/Python boundary on configuration types so binding
drift (renamed fields, removed setters, default-value changes) breaks the
test rather than silently corrupting motion behaviour at runtime.

The previous version of this file was a one-liner that asserted ``motion is
not None`` — any binding rename would have slipped through. The
``conftest.py`` skip-marker handles missing-module situations; we do not
re-implement that here.
"""

from __future__ import annotations

import math

import pytest


def test_axis_constraints_default_construction() -> None:
    from raccoon.motion import AxisConstraints

    cfg = AxisConstraints()
    assert cfg.max_velocity == 0.0
    assert cfg.acceleration == 0.0
    assert cfg.deceleration == 0.0


def test_axis_constraints_keyword_construction_and_mutation() -> None:
    from raccoon.motion import AxisConstraints

    cfg = AxisConstraints(max_velocity=0.5, acceleration=1.5, deceleration=2.0)
    assert cfg.max_velocity == pytest.approx(0.5)
    assert cfg.acceleration == pytest.approx(1.5)
    assert cfg.deceleration == pytest.approx(2.0)

    # Configs are plain data — readwrite is intentional, mutation roundtrips.
    cfg.max_velocity = 0.9
    assert cfg.max_velocity == pytest.approx(0.9)


def test_unified_motion_pid_config_defaults_match_binding() -> None:
    """Lock the C++-side defaults via the binding contract.

    Anyone changing these must update the test deliberately — that is the
    point of pinning them. Numbers come from
    ``modules/libstp-motion/bindings/motion.cpp``.
    """
    from raccoon.motion import UnifiedMotionPidConfig

    cfg = UnifiedMotionPidConfig()
    # Distance PID defaults
    assert cfg.distance.kp == pytest.approx(2.0)
    assert cfg.distance.kd == pytest.approx(0.5)
    # Heading PID defaults
    assert cfg.heading.kp == pytest.approx(2.0)
    assert cfg.heading.kd == pytest.approx(0.3)
    # Saturation handling
    assert cfg.velocity_ff == pytest.approx(1.0)
    assert cfg.saturation_derating_factor == pytest.approx(0.9)
    assert cfg.saturation_min_scale == pytest.approx(0.2)
    assert cfg.saturation_hold_cycles == 5
    # Tolerances
    assert cfg.distance_tolerance_m == pytest.approx(0.01)
    assert cfg.angle_tolerance_rad == pytest.approx(0.035)


def test_unified_motion_pid_config_field_mutation() -> None:
    from raccoon.motion import UnifiedMotionPidConfig

    cfg = UnifiedMotionPidConfig()
    cfg.velocity_ff = 0.7
    cfg.distance_tolerance_m = 0.005
    cfg.angle_tolerance_rad = math.radians(1.0)

    assert cfg.velocity_ff == pytest.approx(0.7)
    assert cfg.distance_tolerance_m == pytest.approx(0.005)
    assert cfg.angle_tolerance_rad == pytest.approx(math.radians(1.0))


def test_unified_motion_pid_config_nested_axis_constraints_share_type() -> None:
    """The nested linear/lateral/angular fields must accept ``AxisConstraints``.

    A binding regression where one of the three got bound to a different
    type would silently default-construct on assignment — catch it here.
    """
    from raccoon.motion import AxisConstraints, UnifiedMotionPidConfig

    cfg = UnifiedMotionPidConfig()
    cfg.linear = AxisConstraints(max_velocity=0.6, acceleration=1.0, deceleration=1.0)
    cfg.lateral = AxisConstraints(max_velocity=0.4, acceleration=0.8, deceleration=0.8)
    cfg.angular = AxisConstraints(max_velocity=2.0, acceleration=4.0, deceleration=4.0)

    assert cfg.linear.max_velocity == pytest.approx(0.6)
    assert cfg.lateral.max_velocity == pytest.approx(0.4)
    assert cfg.angular.max_velocity == pytest.approx(2.0)
