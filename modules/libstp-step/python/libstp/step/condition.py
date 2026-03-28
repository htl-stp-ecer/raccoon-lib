"""Composable stop conditions for motion steps.

A StopCondition determines when a motion step should terminate. Conditions
can be combined with ``|`` (any/or), ``&`` (all/and), and ``>`` (then/sequence)
operators.

Example::

    from libstp.step.condition import on_black, after_seconds, after_cm

    # Stop when sensor sees black OR after 10 seconds
    drive_forward(speed=1.0).until(on_black(sensor) | after_seconds(10))

    # Sequential: first wait for black, then travel 10 more cm
    drive_forward(speed=1.0).until(on_black(sensor) > after_cm(10))
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Callable, Union

if TYPE_CHECKING:
    from libstp.hal import AnalogSensor, DigitalSensor, Motor
    from libstp.robot.api import GenericRobot
    from libstp.sensor_ir import IRSensor


class StopCondition:
    """Base class for composable stop conditions."""

    def start(self, robot: "GenericRobot") -> None:
        """Called once when the motion step starts. Override to initialize state."""
        pass

    def check(self, robot: "GenericRobot") -> bool:
        """Return True to stop the motion. Called each update cycle."""
        raise NotImplementedError

    def __or__(self, other: "StopCondition") -> "StopCondition":
        """Combine conditions: stop when EITHER triggers."""
        return _AnyOf(self, other)

    def __and__(self, other: "StopCondition") -> "StopCondition":
        """Combine conditions: stop when BOTH are true."""
        return _AllOf(self, other)

    def __gt__(self, other: "StopCondition") -> "StopCondition":
        """Chain conditions: once *self* triggers, start checking *other*."""
        return _Then(self, other)


class _Then(StopCondition):
    """Sequential chain: once *first* triggers, start and check *second*.

    Supports chaining: ``a > b > c`` means wait for *a*, then *b*, then *c*.
    """

    def __init__(self, first: StopCondition, second: StopCondition):
        if not isinstance(first, StopCondition):
            raise TypeError(
                f"Expected a StopCondition, got {type(first).__name__}"
            )
        if not isinstance(second, StopCondition):
            raise TypeError(
                f"Expected a StopCondition, got {type(second).__name__}"
            )
        self._first = first
        self._second = second
        self._first_done = False

    def start(self, robot: "GenericRobot") -> None:
        self._first_done = False
        self._first.start(robot)

    def check(self, robot: "GenericRobot") -> bool:
        if not self._first_done:
            if self._first.check(robot):
                self._first_done = True
                self._second.start(robot)
            return False
        return self._second.check(robot)


class _AnyOf(StopCondition):
    """Stop when any sub-condition triggers (OR)."""

    def __init__(self, *conditions: StopCondition):
        for i, c in enumerate(conditions):
            if not isinstance(c, StopCondition):
                raise TypeError(
                    f"Expected StopCondition at position {i}, "
                    f"got {type(c).__name__}"
                )
        self._conditions = conditions

    def start(self, robot: "GenericRobot") -> None:
        for c in self._conditions:
            c.start(robot)

    def check(self, robot: "GenericRobot") -> bool:
        return any(c.check(robot) for c in self._conditions)


class _AllOf(StopCondition):
    """Stop when all sub-conditions are true (AND)."""

    def __init__(self, *conditions: StopCondition):
        for i, c in enumerate(conditions):
            if not isinstance(c, StopCondition):
                raise TypeError(
                    f"Expected StopCondition at position {i}, "
                    f"got {type(c).__name__}"
                )
        self._conditions = conditions

    def start(self, robot: "GenericRobot") -> None:
        for c in self._conditions:
            c.start(robot)

    def check(self, robot: "GenericRobot") -> bool:
        return all(c.check(robot) for c in self._conditions)


class on_black(StopCondition):
    """Stop when an IR sensor detects a black surface."""

    def __init__(self, sensor: "IRSensor", threshold: float = 0.7):
        if not hasattr(sensor, "probabilityOfBlack"):
            raise TypeError(
                f"Expected an IRSensor with probabilityOfBlack(), "
                f"got {type(sensor).__name__}"
            )
        if not (0.0 <= threshold <= 1.0):
            raise ValueError(f"threshold must be 0.0–1.0, got {threshold}")
        self._sensor = sensor
        self._threshold = threshold

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.probabilityOfBlack() >= self._threshold


class on_white(StopCondition):
    """Stop when an IR sensor detects a white surface."""

    def __init__(self, sensor: "IRSensor", threshold: float = 0.7):
        if not hasattr(sensor, "probabilityOfWhite"):
            raise TypeError(
                f"Expected an IRSensor with probabilityOfWhite(), "
                f"got {type(sensor).__name__}"
            )
        if not (0.0 <= threshold <= 1.0):
            raise ValueError(f"threshold must be 0.0–1.0, got {threshold}")
        self._sensor = sensor
        self._threshold = threshold

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.probabilityOfWhite() >= self._threshold


class after_seconds(StopCondition):
    """Stop after a fixed duration."""

    def __init__(self, seconds: float):
        if not isinstance(seconds, (int, float)):
            raise TypeError(f"seconds must be a number, got {type(seconds).__name__}")
        if seconds < 0:
            raise ValueError(f"seconds must be >= 0, got {seconds}")
        self._duration = seconds
        self._deadline: float = 0

    def start(self, robot: "GenericRobot") -> None:
        self._deadline = time.monotonic() + self._duration

    def check(self, robot: "GenericRobot") -> bool:
        return time.monotonic() >= self._deadline


class after_cm(StopCondition):
    """Stop after traveling a distance (uses odometry path length).

    By default operates in relative mode: the distance is measured from
    the robot's position when the condition starts.  In absolute mode
    (``absolute=True``) the distance is measured from the odometry origin
    (last ``reset()``), so centimeters traveled *before* the condition
    started also count.

    Uses the cumulative path length (odometer) so it works correctly
    regardless of travel direction (forward, strafe, curved paths).
    """

    def __init__(self, cm: float, *, absolute: bool = False):
        if not isinstance(cm, (int, float)):
            raise TypeError(f"cm must be a number, got {type(cm).__name__}")
        if cm <= 0:
            raise ValueError(f"cm must be > 0, got {cm}")
        self._target_m = cm / 100.0
        self._absolute = absolute
        self._baseline_m: float = 0.0
        self._started = False

    def start(self, robot: "GenericRobot") -> None:
        if self._absolute:
            self._baseline_m = 0.0
        else:
            self._baseline_m = robot.odometry.get_path_length()
        self._started = True

    def check(self, robot: "GenericRobot") -> bool:
        if not self._started:
            return False
        return (robot.odometry.get_path_length() - self._baseline_m) >= self._target_m


class after_degrees(StopCondition):
    """Stop after turning a given angle (degrees).

    Uses the absolute IMU heading so it is unaffected by odometry resets.
    Tracks total unsigned rotation — works regardless of turn direction.
    """

    def __init__(self, degrees: float):
        if not isinstance(degrees, (int, float)):
            raise TypeError(f"degrees must be a number, got {type(degrees).__name__}")
        if degrees <= 0:
            raise ValueError(f"degrees must be > 0, got {degrees}")
        self._target_rad = math.radians(abs(degrees))
        self._start_heading: float = 0.0

    def start(self, robot: "GenericRobot") -> None:
        self._start_heading = robot.odometry.get_absolute_heading()

    def check(self, robot: "GenericRobot") -> bool:
        current = robot.odometry.get_absolute_heading()
        delta = abs(current - self._start_heading)
        if delta > math.pi:
            delta = 2 * math.pi - delta
        return delta >= self._target_rad


class on_digital(StopCondition):
    """Stop when a digital sensor reads a given state.

    By default stops when the sensor reads True (pressed). Set *pressed*
    to False to stop when the sensor is released instead.
    """

    def __init__(self, sensor: "DigitalSensor", pressed: bool = True):
        if not hasattr(sensor, "read"):
            raise TypeError(
                f"Expected a DigitalSensor with read(), "
                f"got {type(sensor).__name__}"
            )
        self._sensor = sensor
        self._pressed = pressed

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.read() == self._pressed


class on_analog_above(StopCondition):
    """Stop when an analog sensor reading exceeds a threshold."""

    def __init__(self, sensor: "AnalogSensor", threshold: int):
        if not hasattr(sensor, "read"):
            raise TypeError(
                f"Expected an AnalogSensor with read(), "
                f"got {type(sensor).__name__}"
            )
        if not isinstance(threshold, (int, float)):
            raise TypeError(f"threshold must be a number, got {type(threshold).__name__}")
        self._sensor = sensor
        self._threshold = threshold

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.read() > self._threshold


class on_analog_below(StopCondition):
    """Stop when an analog sensor reading drops below a threshold."""

    def __init__(self, sensor: "AnalogSensor", threshold: int):
        if not hasattr(sensor, "read"):
            raise TypeError(
                f"Expected an AnalogSensor with read(), "
                f"got {type(sensor).__name__}"
            )
        if not isinstance(threshold, (int, float)):
            raise TypeError(f"threshold must be a number, got {type(threshold).__name__}")
        self._sensor = sensor
        self._threshold = threshold

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.read() < self._threshold


class stall_detected(StopCondition):
    """Stop when a motor appears stalled.

    A motor is considered stalled when its position changes by fewer than
    *threshold_tps* ticks per second for at least *duration* seconds
    continuously.
    """

    def __init__(
        self, motor: "Motor", threshold_tps: int = 10, duration: float = 0.25
    ):
        if not hasattr(motor, "get_position"):
            raise TypeError(
                f"Expected a Motor with get_position(), "
                f"got {type(motor).__name__}"
            )
        if not isinstance(threshold_tps, (int, float)) or threshold_tps <= 0:
            raise ValueError(f"threshold_tps must be > 0, got {threshold_tps}")
        if not isinstance(duration, (int, float)) or duration <= 0:
            raise ValueError(f"duration must be > 0, got {duration}")
        self._motor = motor
        self._threshold_tps = threshold_tps
        self._duration = duration
        self._last_position: int = 0
        self._last_time: float = 0.0
        self._stall_start: float | None = None

    def start(self, robot: "GenericRobot") -> None:
        self._last_position = self._motor.get_position()
        self._last_time = time.monotonic()
        self._stall_start = None

    def check(self, robot: "GenericRobot") -> bool:
        now = time.monotonic()
        dt = now - self._last_time
        if dt < 0.02:
            return False
        pos = self._motor.get_position()
        speed = abs(pos - self._last_position) / dt
        self._last_position = pos
        self._last_time = now
        if speed < self._threshold_tps:
            if self._stall_start is None:
                self._stall_start = now
            elif now - self._stall_start >= self._duration:
                return True
        else:
            self._stall_start = None
        return False


class custom(StopCondition):
    """Stop based on a user-provided callable.

    The callable receives the robot and returns True to stop::

        custom(lambda robot: robot.et_sensor.value() < 15)
    """

    def __init__(self, fn: Callable[["GenericRobot"], bool]):
        if not callable(fn):
            raise TypeError(f"fn must be callable, got {type(fn).__name__}")
        self._fn = fn

    def check(self, robot: "GenericRobot") -> bool:
        return self._fn(robot)
