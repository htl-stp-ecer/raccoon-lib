"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: arc.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .arc import DriveArcLeft, DriveArcRight, StrafeArcLeft, StrafeArcRight


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

        from raccoon.step.motion import drive_arc_left

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

        from raccoon.step.motion import drive_arc_right

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


class StrafeArcLeftBuilder(StepBuilder):
    """Builder for StrafeArcLeft. Auto-generated — do not edit."""

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
        return StrafeArcLeft(**kwargs)


@dsl(tags=['motion', 'strafe', 'arc'])
def strafe_arc_left(radius_cm: float = _UNSET, degrees: float = _UNSET, speed: float = 1.0):
    """
    Strafe along a circular arc curving to the left.

    The robot strafes laterally (to the right) while simultaneously turning
    counter-clockwise, tracing a circular arc of the given radius. The motion
    completes when the robot has turned by the specified number of degrees.

    Internally uses a profiled PID on heading and derives the lateral velocity
    from the angular velocity command: ``vy = |omega| * radius``. This produces
    coordinated acceleration along the arc.

    Prerequisites:
        Requires a mecanum or omni-wheel drivetrain capable of lateral motion.

    Args:
        radius_cm: Turning radius in centimeters (center of arc to robot center).
        degrees: Arc angle in degrees (how much the robot turns).
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A StrafeArcLeftBuilder (chainable via ``.radius_cm()``, ``.degrees()``, ``.speed()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import strafe_arc_left

        # Quarter-circle strafe arc to the left with 30 cm radius
        strafe_arc_left(radius_cm=30, degrees=90)

        # Gentle wide strafe arc at half speed
        strafe_arc_left(radius_cm=50, degrees=45, speed=0.5)
    """
    b = StrafeArcLeftBuilder()
    if radius_cm is not _UNSET:
        b._radius_cm = radius_cm
    if degrees is not _UNSET:
        b._degrees = degrees
    b._speed = speed
    return b


class StrafeArcRightBuilder(StepBuilder):
    """Builder for StrafeArcRight. Auto-generated — do not edit."""

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
        return StrafeArcRight(**kwargs)


@dsl(tags=['motion', 'strafe', 'arc'])
def strafe_arc_right(radius_cm: float = _UNSET, degrees: float = _UNSET, speed: float = 1.0):
    """
    Strafe along a circular arc curving to the right.

    The robot strafes laterally (to the left) while simultaneously turning
    clockwise, tracing a circular arc of the given radius. The motion
    completes when the robot has turned by the specified number of degrees.

    Internally uses a profiled PID on heading and derives the lateral velocity
    from the angular velocity command: ``vy = |omega| * radius``. This produces
    coordinated acceleration along the arc.

    Prerequisites:
        Requires a mecanum or omni-wheel drivetrain capable of lateral motion.

    Args:
        radius_cm: Turning radius in centimeters (center of arc to robot center).
        degrees: Arc angle in degrees (how much the robot turns).
        speed: Fraction of max speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A StrafeArcRightBuilder (chainable via ``.radius_cm()``, ``.degrees()``, ``.speed()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import strafe_arc_right

        # Quarter-circle strafe arc to the right with 30 cm radius
        strafe_arc_right(radius_cm=30, degrees=90)
    """
    b = StrafeArcRightBuilder()
    if radius_cm is not _UNSET:
        b._radius_cm = radius_cm
    if degrees is not _UNSET:
        b._degrees = degrees
    b._speed = speed
    return b


__all__ = ['DriveArcLeftBuilder', 'drive_arc_left', 'DriveArcRightBuilder', 'drive_arc_right', 'StrafeArcLeftBuilder', 'strafe_arc_left', 'StrafeArcRightBuilder', 'strafe_arc_right']
