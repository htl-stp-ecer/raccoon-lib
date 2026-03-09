"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: arc.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .arc import DriveArcLeft, DriveArcRight, DriveArc


class DriveArcLeftBuilder(StepBuilder):
    """Builder for DriveArcLeft. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._radius_cm = _UNSET
        self._degrees = _UNSET
        self._speed = 1.0

    def radius_cm(self, value: float):
        self._radius_cm = value
        return self

    def degrees(self, value: float):
        self._degrees = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def _build(self):
        kwargs = {}
        if self._radius_cm is not _UNSET:
            kwargs['radius_cm'] = self._radius_cm
        if self._degrees is not _UNSET:
            kwargs['degrees'] = self._degrees
        kwargs['speed'] = self._speed
        return DriveArcLeft(**kwargs)


@dsl(tags=['motion', 'arc'])
def drive_arc_left(radius_cm: float = _UNSET, degrees: float = _UNSET, speed: float = 1.0):
    """
    Drive along a circular arc curving to the left.

    The robot drives forward while simultaneously turning counter-clockwise,
    tracing a circular arc of the given radius. The motion completes when
    the robot has turned by the specified number of degrees.

    Args:
        radius_cm: Turning radius in centimeters (center of arc to robot center).
        degrees: Arc angle in degrees (how much the robot turns).
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A DriveArcLeftBuilder (chainable via ``.radius_cm()``, ``.degrees()``, ``.speed()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import drive_arc_left

        # Quarter-circle left with 30 cm radius
        drive_arc_left(radius_cm=30, degrees=90)

        # Gentle wide arc at half speed
        drive_arc_left(radius_cm=50, degrees=45, speed=0.5)
    """
    b = DriveArcLeftBuilder()
    if radius_cm is not _UNSET:
        b._radius_cm = radius_cm
    if degrees is not _UNSET:
        b._degrees = degrees
    b._speed = speed
    return b


class DriveArcRightBuilder(StepBuilder):
    """Builder for DriveArcRight. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._radius_cm = _UNSET
        self._degrees = _UNSET
        self._speed = 1.0

    def radius_cm(self, value: float):
        self._radius_cm = value
        return self

    def degrees(self, value: float):
        self._degrees = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def _build(self):
        kwargs = {}
        if self._radius_cm is not _UNSET:
            kwargs['radius_cm'] = self._radius_cm
        if self._degrees is not _UNSET:
            kwargs['degrees'] = self._degrees
        kwargs['speed'] = self._speed
        return DriveArcRight(**kwargs)


@dsl(tags=['motion', 'arc'])
def drive_arc_right(radius_cm: float = _UNSET, degrees: float = _UNSET, speed: float = 1.0):
    """
    Drive along a circular arc curving to the right.

    The robot drives forward while simultaneously turning clockwise,
    tracing a circular arc of the given radius. The motion completes when
    the robot has turned by the specified number of degrees.

    Args:
        radius_cm: Turning radius in centimeters (center of arc to robot center).
        degrees: Arc angle in degrees (how much the robot turns).
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A DriveArcRightBuilder (chainable via ``.radius_cm()``, ``.degrees()``, ``.speed()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import drive_arc_right

        # Quarter-circle right with 30 cm radius
        drive_arc_right(radius_cm=30, degrees=90)
    """
    b = DriveArcRightBuilder()
    if radius_cm is not _UNSET:
        b._radius_cm = radius_cm
    if degrees is not _UNSET:
        b._degrees = degrees
    b._speed = speed
    return b


class DriveArcBuilder(StepBuilder):
    """Builder for DriveArc. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._radius_cm = _UNSET
        self._degrees = _UNSET
        self._speed = 1.0

    def radius_cm(self, value: float):
        self._radius_cm = value
        return self

    def degrees(self, value: float):
        self._degrees = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def _build(self):
        kwargs = {}
        if self._radius_cm is not _UNSET:
            kwargs['radius_cm'] = self._radius_cm
        if self._degrees is not _UNSET:
            kwargs['degrees'] = self._degrees
        kwargs['speed'] = self._speed
        return DriveArc(**kwargs)


@dsl(tags=['motion', 'arc'])
def drive_arc(radius_cm: float = _UNSET, degrees: float = _UNSET, speed: float = 1.0):
    """
    Drive along a circular arc with explicit direction.

    Positive degrees = counter-clockwise (left), negative = clockwise (right).

    Args:
        radius_cm: Turning radius in centimeters (always positive).
        degrees: Arc angle in degrees. Positive = left/CCW, negative = right/CW.
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A DriveArcBuilder (chainable via ``.radius_cm()``, ``.degrees()``, ``.speed()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import drive_arc

        # Left arc
        drive_arc(radius_cm=30, degrees=90)

        # Right arc
        drive_arc(radius_cm=30, degrees=-90)
    """
    b = DriveArcBuilder()
    if radius_cm is not _UNSET:
        b._radius_cm = radius_cm
    if degrees is not _UNSET:
        b._degrees = degrees
    b._speed = speed
    return b


__all__ = ['DriveArcLeftBuilder', 'drive_arc_left', 'DriveArcRightBuilder', 'drive_arc_right', 'DriveArcBuilder', 'drive_arc']
