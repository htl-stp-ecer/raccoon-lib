"""Tests for raccoon.kinematics bindings.

Construction tests for differential and mecanum kinematics. We can build
these without a running platform because Motor instances are pure-Python
wrappers; only ``apply_command`` requires a real HAL.
"""

from __future__ import annotations

import pytest


def _make_motor(port: int):
    from raccoon.hal import Motor

    return Motor(port=port)


def test_differential_kinematics_construction() -> None:
    from raccoon.kinematics_differential import DifferentialKinematics

    left = _make_motor(0)
    right = _make_motor(1)
    kin = DifferentialKinematics(
        left,
        right,
        wheelbase=0.20,
        wheel_radius=0.04,
    )
    assert kin.wheel_count() == 2
    # Differential cannot strafe — verify the binding propagates that.
    assert kin.supports_lateral_motion() is False


def test_mecanum_kinematics_construction() -> None:
    from raccoon.kinematics_mecanum import MecanumKinematics

    fl, fr, bl, br = (_make_motor(p) for p in range(4))
    kin = MecanumKinematics(
        front_left_motor=fl,
        front_right_motor=fr,
        back_left_motor=bl,
        back_right_motor=br,
        wheelbase=0.18,
        track_width=0.16,
        wheel_radius=0.04,
    )
    assert kin.wheel_count() == 4
    # Mecanum supports lateral motion by construction.
    assert kin.supports_lateral_motion() is True


def test_differential_kinematics_rejects_zero_wheelbase() -> None:
    """Zero wheelbase divides by zero in the inverse-kinematics matrix.

    The C++ ctor must reject this — a successful Python-side construction
    here would mean the validation has slipped through the binding boundary.
    """
    from raccoon.kinematics_differential import DifferentialKinematics

    with pytest.raises((ValueError, RuntimeError, Exception)):
        DifferentialKinematics(
            _make_motor(0),
            _make_motor(1),
            wheelbase=0.0,
            wheel_radius=0.04,
        )
