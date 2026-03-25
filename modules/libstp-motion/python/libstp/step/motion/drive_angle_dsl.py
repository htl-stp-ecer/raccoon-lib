"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: drive_angle.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .drive_angle import DriveAngle


class DriveAngleBuilder(StepBuilder):
    """Builder for DriveAngle. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._angle_deg = _UNSET
        self._cm = _UNSET
        self._speed = 1.0

    def angle_deg(self, value: float):
        self._angle_deg = value
        return self

    def cm(self, value: float):
        self._cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def _build(self):
        kwargs = {}
        if self._angle_deg is not _UNSET:
            kwargs['angle_deg'] = self._angle_deg
        if self._cm is not _UNSET:
            kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        return DriveAngle(**kwargs)


@dsl(tags=['motion', 'drive'])
def drive_angle(angle_deg: float = _UNSET, cm: float = _UNSET, speed: float = 1.0):
    """
    Drive at an arbitrary angle for a specified distance.

    Decomposes the desired heading into forward and lateral velocity
    components, then runs a profiled PID controller in a rotated
    coordinate frame with heading maintenance and cross-track correction.

    Requires a mecanum or omni-wheel drivetrain.

    Angle convention (robot-centric): ``0`` = forward, ``90`` = right,
    ``-90`` = left, ``180`` = backward.

    Args:
        angle_deg: Travel angle in degrees.
        cm: Distance to travel in centimeters.
        speed: Fraction of max speed, 0.0 to 1.0.

    Returns:
        A DriveAngleBuilder (chainable via ``.angle_deg()``, ``.cm()``, ``.speed()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import drive_angle

        # Drive diagonally forward-right at 45 degrees
        drive_angle(45, cm=30)

        # Drive pure right (same as strafe_right)
        drive_angle(90, cm=20)
    """
    b = DriveAngleBuilder()
    if angle_deg is not _UNSET:
        b._angle_deg = angle_deg
    if cm is not _UNSET:
        b._cm = cm
    b._speed = speed
    return b


__all__ = ['DriveAngleBuilder', 'drive_angle']
