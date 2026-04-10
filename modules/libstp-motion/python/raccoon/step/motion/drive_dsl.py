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
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        return DriveForward(**kwargs)


@dsl(tags=['motion', 'drive'])
def drive_forward(cm: float = None, speed: float = 1.0, until: StopCondition = None):
    """
    Drive forward with distance or condition-based termination.

    Uses profiled PID motion control with a trapezoidal velocity profile.
    The robot accelerates, cruises, and decelerates while maintaining
    heading via IMU feedback. In condition-only mode the robot drives at
    constant speed until the condition triggers.

    Requires ``calibrate_distance()`` for distance-based mode.

    Args:
        cm: Distance in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination (e.g., ``on_black(sensor)``). Can also be chained via the ``.until()`` builder method.

    Returns:
        A DriveForwardBuilder (chainable via ``.cm()``, ``.speed()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        drive_forward(25)                            # 25 cm
        drive_forward(speed=0.8).until(on_black(s))  # until sensor
    """
    b = DriveForwardBuilder()
    b._cm = cm
    b._speed = speed
    b._until = until
    return b


class DriveBackwardBuilder(StepBuilder):
    """Builder for DriveBackward. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._cm = None
        self._speed = 1.0
        self._until = None

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
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        return DriveBackward(**kwargs)


@dsl(tags=['motion', 'drive'])
def drive_backward(cm: float = None, speed: float = 1.0, until: StopCondition = None):
    """
    Drive backward with distance or condition-based termination.

    Identical to ``drive_forward()`` but in reverse. Uses profiled PID
    motion control while maintaining heading via IMU feedback.

    Requires ``calibrate_distance()`` for distance-based mode.

    Args:
        cm: Distance in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination (e.g., ``on_black(sensor)``). Can also be chained via the ``.until()`` builder method.

    Returns:
        A DriveBackwardBuilder (chainable via ``.cm()``, ``.speed()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        drive_backward(20)
        drive_backward(speed=0.5).until(on_white(s))
    """
    b = DriveBackwardBuilder()
    b._cm = cm
    b._speed = speed
    b._until = until
    return b


class StrafeLeftBuilder(StepBuilder):
    """Builder for StrafeLeft. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._cm = None
        self._speed = 1.0
        self._until = None

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
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        return StrafeLeft(**kwargs)


@dsl(tags=['motion', 'strafe'])
def strafe_left(cm: float = None, speed: float = 1.0, until: StopCondition = None):
    """
    Strafe left with distance or condition-based termination.

    Requires a mecanum or omni-wheel drivetrain. The robot moves
    laterally to the left while maintaining heading via IMU feedback.

    Args:
        cm: Distance in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination (e.g., ``on_black(sensor)``). Can also be chained via the ``.until()`` builder method.

    Returns:
        A StrafeLeftBuilder (chainable via ``.cm()``, ``.speed()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        strafe_left(15)
        strafe_left(speed=0.6).until(on_black(s))
    """
    b = StrafeLeftBuilder()
    b._cm = cm
    b._speed = speed
    b._until = until
    return b


class StrafeRightBuilder(StepBuilder):
    """Builder for StrafeRight. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._cm = None
        self._speed = 1.0
        self._until = None

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
        kwargs['cm'] = self._cm
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        return StrafeRight(**kwargs)


@dsl(tags=['motion', 'strafe'])
def strafe_right(cm: float = None, speed: float = 1.0, until: StopCondition = None):
    """
    Strafe right with distance or condition-based termination.

    Requires a mecanum or omni-wheel drivetrain. The robot moves
    laterally to the right while maintaining heading via IMU feedback.

    Args:
        cm: Distance in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination (e.g., ``on_black(sensor)``). Can also be chained via the ``.until()`` builder method.

    Returns:
        A StrafeRightBuilder (chainable via ``.cm()``, ``.speed()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        strafe_right(15)
        strafe_right(speed=0.6).until(on_black(s))
    """
    b = StrafeRightBuilder()
    b._cm = cm
    b._speed = speed
    b._until = until
    return b


__all__ = ['DriveForwardBuilder', 'drive_forward', 'DriveBackwardBuilder', 'drive_backward', 'StrafeLeftBuilder', 'strafe_left', 'StrafeRightBuilder', 'strafe_right']
