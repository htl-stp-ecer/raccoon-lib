"""Composable stop conditions for motion steps.

A StopCondition determines when a motion step should terminate. Conditions
can be combined with ``|`` (any/or) and ``&`` (all/and) operators.

Example::

    from libstp.step.condition import on_black, after_seconds

    # Stop when sensor sees black OR after 10 seconds
    drive_forward(speed=1.0).until(on_black(sensor) | after_seconds(10))
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Callable, Union

if TYPE_CHECKING:
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
        info = robot.odometry.getDistanceFromOrigin()
        return info.euclidean >= self._target_m


class custom(StopCondition):
    """Stop based on a user-provided callable.

    The callable receives the robot and returns True to stop::

        custom(lambda robot: robot.et_sensor.value() < 15)
    """

    def __init__(self, fn: Callable[["GenericRobot"], bool]):
        self._fn = fn

    def check(self, robot: "GenericRobot") -> bool:
        return self._fn(robot)
