"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: drive.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .drive import DriveForward, DriveBackward, StrafeLeft, StrafeRight


class DriveForwardBuilder(StepBuilder):
    """Builder for DriveForward. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._cm = None
        self._speed = 1.0
        self._until = None
        self._heading = None

    def cm(self, value: float):
        self._cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def until(self, value: StopCondition):
        self._until = value
        return self

    def heading(self, value: float):
        self._heading = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        kwargs['heading'] = self._heading
        return DriveForward(**kwargs)


@dsl(tags=['motion', 'drive'])
def drive_forward(cm: float = None, speed: float = 1.0, until: StopCondition = None, heading: float = None):
    """
    Drive forward with distance or condition-based termination.

    Uses profiled PID motion control with a trapezoidal velocity profile.
    The robot accelerates, cruises, and decelerates while maintaining
    heading via IMU feedback. When ``heading`` is given, the controller
    holds that absolute heading (degrees from heading reference) instead
    of the heading at start, preventing drift accumulation across
    consecutive drives.

    Requires ``calibrate_distance()`` for distance-based mode.
    Requires ``mark_heading_reference()`` when using ``heading``.

    Args:
        cm: Distance in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination (e.g., ``on_black(sensor)``). Can also be chained via the ``.until()`` builder method.
        heading: Absolute heading in degrees from the heading reference to hold during this drive. ``None`` (default) holds the heading at the start of the drive (relative mode).

    Returns:
        A DriveForwardBuilder (chainable via ``.cm()``, ``.speed()``, ``.until()``, ``.heading()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        drive_forward(25)                            # 25 cm, relative heading
        drive_forward(25, heading=90)                # hold 90° absolute
        drive_forward(speed=0.8).until(on_black(s))  # until sensor
    """
    b = DriveForwardBuilder()
    b._cm = cm
    b._speed = speed
    b._until = until
    b._heading = heading
    return b


class DriveBackwardBuilder(StepBuilder):
    """Builder for DriveBackward. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._cm = None
        self._speed = 1.0
        self._until = None
        self._heading = None

    def cm(self, value: float):
        self._cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def until(self, value: StopCondition):
        self._until = value
        return self

    def heading(self, value: float):
        self._heading = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        kwargs['heading'] = self._heading
        return DriveBackward(**kwargs)


@dsl(tags=['motion', 'drive'])
def drive_backward(cm: float = None, speed: float = 1.0, until: StopCondition = None, heading: float = None):
    """
    Drive backward with distance or condition-based termination.

    Identical to ``drive_forward()`` but in reverse. Uses profiled PID
    motion control while maintaining heading via IMU feedback. Supports
    the same ``heading`` parameter for absolute heading hold.

    Requires ``calibrate_distance()`` for distance-based mode.
    Requires ``mark_heading_reference()`` when using ``heading``.

    Args:
        cm: Distance in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination (e.g., ``on_black(sensor)``). Can also be chained via the ``.until()`` builder method.
        heading: Absolute heading in degrees from the heading reference to hold during this drive. ``None`` (default) holds the heading at the start of the drive (relative mode).

    Returns:
        A DriveBackwardBuilder (chainable via ``.cm()``, ``.speed()``, ``.until()``, ``.heading()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        drive_backward(20)
        drive_backward(20, heading=0)                # hold 0° absolute
        drive_backward(speed=0.5).until(on_white(s))
    """
    b = DriveBackwardBuilder()
    b._cm = cm
    b._speed = speed
    b._until = until
    b._heading = heading
    return b


class StrafeLeftBuilder(StepBuilder):
    """Builder for StrafeLeft. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._cm = None
        self._speed = 1.0
        self._until = None
        self._heading = None

    def cm(self, value: float):
        self._cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def until(self, value: StopCondition):
        self._until = value
        return self

    def heading(self, value: float):
        self._heading = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        kwargs['heading'] = self._heading
        return StrafeLeft(**kwargs)


@dsl(tags=['motion', 'strafe'])
def strafe_left(cm: float = None, speed: float = 1.0, until: StopCondition = None, heading: float = None):
    """
    Strafe left with distance or condition-based termination.

    Requires a mecanum or omni-wheel drivetrain. The robot moves
    laterally to the left while maintaining heading via IMU feedback.
    Supports the same ``heading`` parameter for absolute heading hold.

    Requires ``mark_heading_reference()`` when using ``heading``.

    Args:
        cm: Distance in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination (e.g., ``on_black(sensor)``). Can also be chained via the ``.until()`` builder method.
        heading: Absolute heading in degrees from the heading reference to hold during this strafe. ``None`` (default) holds the heading at the start of the strafe (relative mode).

    Returns:
        A StrafeLeftBuilder (chainable via ``.cm()``, ``.speed()``, ``.until()``, ``.heading()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        strafe_left(15)
        strafe_left(15, heading=90)                  # hold 90° absolute
        strafe_left(speed=0.6).until(on_black(s))
    """
    b = StrafeLeftBuilder()
    b._cm = cm
    b._speed = speed
    b._until = until
    b._heading = heading
    return b


class StrafeRightBuilder(StepBuilder):
    """Builder for StrafeRight. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._cm = None
        self._speed = 1.0
        self._until = None
        self._heading = None

    def cm(self, value: float):
        self._cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def until(self, value: StopCondition):
        self._until = value
        return self

    def heading(self, value: float):
        self._heading = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        kwargs['heading'] = self._heading
        return StrafeRight(**kwargs)


@dsl(tags=['motion', 'strafe'])
def strafe_right(cm: float = None, speed: float = 1.0, until: StopCondition = None, heading: float = None):
    """
    Strafe right with distance or condition-based termination.

    Requires a mecanum or omni-wheel drivetrain. The robot moves
    laterally to the right while maintaining heading via IMU feedback.
    Supports the same ``heading`` parameter for absolute heading hold.

    Requires ``mark_heading_reference()`` when using ``heading``.

    Args:
        cm: Distance in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination (e.g., ``on_black(sensor)``). Can also be chained via the ``.until()`` builder method.
        heading: Absolute heading in degrees from the heading reference to hold during this strafe. ``None`` (default) holds the heading at the start of the strafe (relative mode).

    Returns:
        A StrafeRightBuilder (chainable via ``.cm()``, ``.speed()``, ``.until()``, ``.heading()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        strafe_right(15)
        strafe_right(15, heading=0)                  # hold 0° absolute
        strafe_right(speed=0.6).until(on_black(s))
    """
    b = StrafeRightBuilder()
    b._cm = cm
    b._speed = speed
    b._until = until
    b._heading = heading
    return b


__all__ = ['DriveForwardBuilder', 'drive_forward', 'DriveBackwardBuilder', 'drive_backward', 'StrafeLeftBuilder', 'strafe_left', 'StrafeRightBuilder', 'strafe_right']
