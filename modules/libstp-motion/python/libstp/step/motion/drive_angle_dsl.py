"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: drive_angle.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .drive_angle import DriveAngle, DriveAngleLeft, DriveAngleRight


class DriveAngleBuilder(StepBuilder):
    """Builder for DriveAngle. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._angle_deg = _UNSET
        self._cm = None
        self._speed = 1.0
        self._until = None

    def angle_deg(self, value: float):
        self._angle_deg = value
        return self

    def cm(self, value: float):
        self._cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def until(self, value: StopCondition):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        if self._angle_deg is not _UNSET:
            kwargs['angle_deg'] = self._angle_deg
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        return DriveAngle(**kwargs)


@dsl(tags=['motion', 'drive'])
def drive_angle(angle_deg: float = _UNSET, cm: float = None, speed: float = 1.0, until: StopCondition = None):
    """
    Drive at an arbitrary angle with distance or condition-based termination.

    Decomposes the desired heading into forward and lateral velocity
    components, then runs a profiled PID controller in a rotated
    coordinate frame with heading maintenance and cross-track correction.

    Requires a mecanum or omni-wheel drivetrain.

    Angle convention (robot-centric): ``0`` = forward, ``90`` = right,
    ``-90`` = left, ``180`` = backward.

    Args:
        angle_deg: Travel angle in degrees.
        cm: Distance to travel in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination.

    Returns:
        A DriveAngleBuilder (chainable via ``.angle_deg()``, ``.cm()``, ``.speed()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

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
    b._cm = cm
    b._speed = speed
    b._until = until
    return b


class DriveAngleLeftBuilder(StepBuilder):
    """Builder for DriveAngleLeft. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._angle_deg = _UNSET
        self._cm = None
        self._speed = 1.0
        self._until = None

    def angle_deg(self, value: float):
        self._angle_deg = value
        return self

    def cm(self, value: float):
        self._cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def until(self, value: StopCondition):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        if self._angle_deg is not _UNSET:
            kwargs['angle_deg'] = self._angle_deg
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        return DriveAngleLeft(**kwargs)


@dsl(tags=['motion', 'drive'])
def drive_angle_left(angle_deg: float = _UNSET, cm: float = None, speed: float = 1.0, until: StopCondition = None):
    """
    Drive at an angle to the left with distance or condition-based termination.

    Convenience wrapper around ``DriveAngle`` that negates the angle so
    that the ``angle_deg`` parameter is always positive (pointing left).

    The angle is measured as degrees to the left of forward:
    ``0`` = forward, ``45`` = forward-left diagonal, ``90`` = pure left.

    Args:
        angle_deg: Degrees to the left of forward (0 = forward, 90 = pure left).
        cm: Distance to travel in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination.

    Returns:
        A DriveAngleLeftBuilder (chainable via ``.angle_deg()``, ``.cm()``, ``.speed()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import drive_angle_left

        # Drive diagonally forward-left at 45 degrees
        drive_angle_left(45, cm=30)

        # Drive pure left until sensor
        drive_angle_left(90, speed=0.6).until(on_black(s))
    """
    b = DriveAngleLeftBuilder()
    if angle_deg is not _UNSET:
        b._angle_deg = angle_deg
    b._cm = cm
    b._speed = speed
    b._until = until
    return b


class DriveAngleRightBuilder(StepBuilder):
    """Builder for DriveAngleRight. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._angle_deg = _UNSET
        self._cm = None
        self._speed = 1.0
        self._until = None

    def angle_deg(self, value: float):
        self._angle_deg = value
        return self

    def cm(self, value: float):
        self._cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def until(self, value: StopCondition):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        if self._angle_deg is not _UNSET:
            kwargs['angle_deg'] = self._angle_deg
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        return DriveAngleRight(**kwargs)


@dsl(tags=['motion', 'drive'])
def drive_angle_right(angle_deg: float = _UNSET, cm: float = None, speed: float = 1.0, until: StopCondition = None):
    """
    Drive at an angle to the right with distance or condition-based termination.

    Convenience wrapper around ``DriveAngle`` that passes the angle
    directly (positive = right in the robot frame).

    The angle is measured as degrees to the right of forward:
    ``0`` = forward, ``45`` = forward-right diagonal, ``90`` = pure right.

    Args:
        angle_deg: Degrees to the right of forward (0 = forward, 90 = pure right).
        cm: Distance to travel in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination.

    Returns:
        A DriveAngleRightBuilder (chainable via ``.angle_deg()``, ``.cm()``, ``.speed()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import drive_angle_right

        # Drive diagonally forward-right at 45 degrees
        drive_angle_right(45, cm=30)

        # Drive pure right until sensor
        drive_angle_right(90, speed=0.6).until(on_black(s))
    """
    b = DriveAngleRightBuilder()
    if angle_deg is not _UNSET:
        b._angle_deg = angle_deg
    b._cm = cm
    b._speed = speed
    b._until = until
    return b


__all__ = ['DriveAngleBuilder', 'drive_angle', 'DriveAngleLeftBuilder', 'drive_angle_left', 'DriveAngleRightBuilder', 'drive_angle_right']
