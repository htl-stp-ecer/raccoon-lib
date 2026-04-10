"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: turn.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .turn import TurnLeft, TurnRight


class TurnLeftBuilder(StepBuilder):
    """Builder for TurnLeft. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._degrees = None
        self._speed = 1.0
        self._until = None

    def degrees(self, value: float):
        self._degrees = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def until(self, value: StopCondition):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['degrees'] = self._degrees
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        return TurnLeft(**kwargs)


@dsl(tags=['motion', 'turn'])
def turn_left(degrees: float = None, speed: float = 1.0, until: StopCondition = None):
    """
    Turn left (counter-clockwise) with angle or condition-based termination.

    Uses a PID controller on the IMU heading to rotate the robot in place.
    The controller saturates output at the configured max angular rate,
    producing an implicit trapezoidal velocity profile.

    Args:
        degrees: Angle in degrees. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination (e.g., ``on_black(sensor)``). Can also be chained via the ``.until()`` builder method.

    Returns:
        A TurnLeftBuilder (chainable via ``.degrees()``, ``.speed()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        turn_left(90)
        turn_left(speed=0.5).until(on_black(s))
    """
    b = TurnLeftBuilder()
    b._degrees = degrees
    b._speed = speed
    b._until = until
    return b


class TurnRightBuilder(StepBuilder):
    """Builder for TurnRight. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._degrees = None
        self._speed = 1.0
        self._until = None

    def degrees(self, value: float):
        self._degrees = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def until(self, value: StopCondition):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['degrees'] = self._degrees
        kwargs['speed'] = self._speed
        kwargs['until'] = self._until
        return TurnRight(**kwargs)


@dsl(tags=['motion', 'turn'])
def turn_right(degrees: float = None, speed: float = 1.0, until: StopCondition = None):
    """
    Turn right (clockwise) with angle or condition-based termination.

    Uses a PID controller on the IMU heading to rotate the robot in place.
    The controller saturates output at the configured max angular rate,
    producing an implicit trapezoidal velocity profile.

    Args:
        degrees: Angle in degrees. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination (e.g., ``on_black(sensor)``). Can also be chained via the ``.until()`` builder method.

    Returns:
        A TurnRightBuilder (chainable via ``.degrees()``, ``.speed()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        turn_right(90)
        turn_right(speed=0.5).until(on_black(s))
    """
    b = TurnRightBuilder()
    b._degrees = degrees
    b._speed = speed
    b._until = until
    return b


__all__ = ['TurnLeftBuilder', 'turn_left', 'TurnRightBuilder', 'turn_right']
