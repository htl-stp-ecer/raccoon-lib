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
        self._conditions = conditions

    def start(self, robot: "GenericRobot") -> None:
        for c in self._conditions:
            c.start(robot)

    def check(self, robot: "GenericRobot") -> bool:
        return any(c.check(robot) for c in self._conditions)


class _AllOf(StopCondition):
    """Stop when all sub-conditions are true (AND)."""

    def __init__(self, *conditions: StopCondition):
        self._conditions = conditions

    def start(self, robot: "GenericRobot") -> None:
        for c in self._conditions:
            c.start(robot)

    def check(self, robot: "GenericRobot") -> bool:
        return all(c.check(robot) for c in self._conditions)


class on_black(StopCondition):
    """Stop when an IR sensor detects a black surface."""

    def __init__(self, sensor: "IRSensor", threshold: float = 0.7):
        self._sensor = sensor
        self._threshold = threshold

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.probabilityOfBlack() >= self._threshold


class on_white(StopCondition):
    """Stop when an IR sensor detects a white surface."""

    def __init__(self, sensor: "IRSensor", threshold: float = 0.7):
        self._sensor = sensor
        self._threshold = threshold

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.probabilityOfWhite() >= self._threshold


class after_seconds(StopCondition):
    """Stop after a fixed duration."""

    def __init__(self, seconds: float):
        self._duration = seconds
        self._deadline: float = 0

    def start(self, robot: "GenericRobot") -> None:
        self._deadline = time.monotonic() + self._duration

    def check(self, robot: "GenericRobot") -> bool:
        return time.monotonic() >= self._deadline


class after_cm(StopCondition):
    """Stop after traveling a distance (uses odometry)."""

    def __init__(self, cm: float):
        self._target_m = cm / 100.0
        self._started = False

    def start(self, robot: "GenericRobot") -> None:
        robot.odometry.reset()
        self._started = True

    def check(self, robot: "GenericRobot") -> bool:
        if not self._started:
            return False
        info = robot.odometry.get_distance_from_origin()
        return info.straight_line >= self._target_m


class after_degrees(StopCondition):
    """Stop after turning a given angle (degrees).

    Uses the absolute IMU heading so it is unaffected by odometry resets.
    Tracks total unsigned rotation — works regardless of turn direction.
    """

    def __init__(self, degrees: float):
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
        self._sensor = sensor
        self._pressed = pressed

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.read() == self._pressed


class on_analog_above(StopCondition):
    """Stop when an analog sensor reading exceeds a threshold."""

    def __init__(self, sensor: "AnalogSensor", threshold: int):
        self._sensor = sensor
        self._threshold = threshold

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.read() > self._threshold


class on_analog_below(StopCondition):
    """Stop when an analog sensor reading drops below a threshold."""

    def __init__(self, sensor: "AnalogSensor", threshold: int):
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
        self._fn = fn

    def check(self, robot: "GenericRobot") -> bool:
        return self._fn(robot)
