"""Composable stop conditions for motion steps.

A StopCondition determines when a motion step should terminate. Conditions
can be combined with ``|`` (any/or), ``&`` (all/and), ``>`` / ``+``
(then/sequence) operators.

Example::

    from raccoon.step.condition import on_black, on_white, after_seconds, after_cm, over_line

    # Stop when sensor sees black OR after 10 seconds
    drive_forward(speed=1.0).until(on_black(sensor) | after_seconds(10))

    # Sequential: first wait for black, then travel 10 more cm
    # Use + instead of > to avoid Python's chained-comparison gotcha
    drive_forward(speed=1.0).until(on_black(sensor) + after_cm(10))

    # Cross a line (black then white) three times
    drive_forward(speed=1.0).until(over_line(sensor) + over_line(sensor) + over_line(sensor))
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from raccoon.hal import AnalogSensor, DigitalSensor, Motor
    from raccoon.robot.api import GenericRobot
    from raccoon.sensor_ir import IRSensor


class StopCondition:
    """Base class for composable stop conditions."""

    def start(self, robot: "GenericRobot") -> None:
        """Called once when the motion step starts. Override to initialize state."""

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

    def __add__(self, other: "StopCondition") -> "StopCondition":
        """Chain conditions (alias for ``>``): ``a + b + c``.

        Unlike ``>``, the ``+`` operator does not suffer from Python's
        chained-comparison rewrite, so parentheses are never required.
        """
        return _Then(self, other)

    def __bool__(self) -> bool:
        msg = (
            "StopCondition cannot be used as a boolean. "
            "You may have written 'a > b > c' — Python treats this as a chained "
            "comparison, not (a > b) > c. Use parentheses: '(a > b) > c', "
            "or use the + operator: 'a + b + c'."
        )
        raise TypeError(msg)


class _Then(StopCondition):
    """Sequential chain: once *first* triggers, start and check *second*.

    Supports chaining: ``a > b > c`` means wait for *a*, then *b*, then *c*.
    """

    def __init__(self, first: StopCondition, second: StopCondition):
        if not isinstance(first, StopCondition):
            msg = f"Expected a StopCondition, got {type(first).__name__}"
            raise TypeError(msg)
        if not isinstance(second, StopCondition):
            msg = f"Expected a StopCondition, got {type(second).__name__}"
            raise TypeError(msg)
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
                msg = f"Expected StopCondition at position {i}, " f"got {type(c).__name__}"
                raise TypeError(msg)
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
                msg = f"Expected StopCondition at position {i}, " f"got {type(c).__name__}"
                raise TypeError(msg)
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
            msg = f"Expected an IRSensor with probabilityOfBlack(), " f"got {type(sensor).__name__}"
            raise TypeError(msg)
        if not (0.0 <= threshold <= 1.0):
            msg = f"threshold must be 0.0–1.0, got {threshold}"
            raise ValueError(msg)
        self._sensor = sensor
        self._threshold = threshold

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.probabilityOfBlack() >= self._threshold


class on_white(StopCondition):
    """Stop when an IR sensor detects a white surface."""

    def __init__(self, sensor: "IRSensor", threshold: float = 0.7):
        if not hasattr(sensor, "probabilityOfWhite"):
            msg = f"Expected an IRSensor with probabilityOfWhite(), " f"got {type(sensor).__name__}"
            raise TypeError(msg)
        if not (0.0 <= threshold <= 1.0):
            msg = f"threshold must be 0.0–1.0, got {threshold}"
            raise ValueError(msg)
        self._sensor = sensor
        self._threshold = threshold

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.probabilityOfWhite() >= self._threshold


class after_seconds(StopCondition):
    """Stop after a fixed duration."""

    def __init__(self, seconds: float):
        if not isinstance(seconds, int | float):
            msg = f"seconds must be a number, got {type(seconds).__name__}"
            raise TypeError(msg)
        if seconds < 0:
            msg = f"seconds must be >= 0, got {seconds}"
            raise ValueError(msg)
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
        if not isinstance(cm, int | float):
            msg = f"cm must be a number, got {type(cm).__name__}"
            raise TypeError(msg)
        if cm <= 0:
            msg = f"cm must be > 0, got {cm}"
            raise ValueError(msg)
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


class _AxisDisplacementCondition(StopCondition):
    """Shared base: signed displacement along a single reference axis.

    The condition measures how far the robot has travelled *along one fixed
    line*, the reference axis. ``after_forward_cm`` projects onto the
    forward direction of that axis, ``after_lateral_cm`` onto the lateral
    direction.

    The reference axis is the heading frozen when the condition starts. It
    does **not** follow the robot as it turns: this is deliberate.
    ``.until(...)`` says "drive 20 cm along *this* line". A line-follow
    corrects *around* that reference, so its slightly diagonal travel
    contributes a little less forward per centimetre of path — which is
    correct. If the robot turns away from the reference and drives off at an
    angle it makes less forward progress along the line, and at 90° it can
    never reach the target at all.

    Which heading defines that reference line:

    * default — the robot's **current** heading at ``start()``. Use this
      when the robot is already aligned with the line it is about to drive.
    * ``heading=<deg>`` — a **targeted absolute** heading (degrees from the
      ``HeadingReferenceService`` reference, same convention as
      ``strafe_left(heading=...)`` / ``turn_to_heading``). Use this when the
      robot may be crooked at ``start()`` — e.g. the line-follow is still
      correcting onto the line — so the reference is the line you *intend*
      to drive, not whatever angle the robot happened to sit at when the
      condition started.

    In **absolute** mode, the condition reads
    ``get_distance_from_origin().forward / .lateral`` directly — axes fixed
    to the odometry origin heading (last ``reset()``).
    """

    # Sign of the projection: +1 for forward, -1 means "use lateral".
    # Concrete subclasses override _project().
    _axis_name = ""  # for error messages

    def __init__(self, cm: float, *, heading: float | None = None, absolute: bool = False):
        if not isinstance(cm, int | float):
            msg = f"cm must be a number, got {type(cm).__name__}"
            raise TypeError(msg)
        if cm == 0:
            msg = f"cm must be nonzero, got {cm}"
            raise ValueError(msg)
        if heading is not None and not isinstance(heading, int | float):
            msg = f"heading must be a number or None, got {type(heading).__name__}"
            raise TypeError(msg)
        if heading is not None and absolute:
            msg = "heading= and absolute=True are mutually exclusive"
            raise ValueError(msg)
        self._target_m = cm / 100.0
        self._heading_deg = heading
        self._absolute = absolute
        self._origin_x_m: float = 0.0
        self._origin_y_m: float = 0.0
        self._cos_h: float = 1.0
        self._sin_h: float = 0.0
        self._started = False

    def _reference_heading_rad(self, robot: "GenericRobot") -> float:
        """Resolve the frozen reference heading (radians) at ``start()``."""
        if self._heading_deg is None:
            return float(robot.odometry.get_pose().heading)
        # Targeted absolute heading: resolve through the same reference the
        # motion controllers hold their heading against, so "heading=0" means
        # the same line for the drive and for this stop condition.
        from raccoon.robot.heading_reference import HeadingReferenceService

        service = robot.get_service(HeadingReferenceService)
        return float(service.target_absolute_rad(self._heading_deg))

    def start(self, robot: "GenericRobot") -> None:
        if not self._absolute:
            pose = robot.odometry.get_pose()
            self._origin_x_m = float(pose.position[0])
            self._origin_y_m = float(pose.position[1])
            heading = self._reference_heading_rad(robot)
            self._cos_h = math.cos(heading)
            self._sin_h = math.sin(heading)
        self._started = True

    def _displacement_m(self, robot: "GenericRobot") -> float:
        if self._absolute:
            return self._absolute_component(robot)
        pose = robot.odometry.get_pose()
        dx = float(pose.position[0]) - self._origin_x_m
        dy = float(pose.position[1]) - self._origin_y_m
        return self._project(dx, dy)

    def _project(self, dx: float, dy: float) -> float:
        raise NotImplementedError

    def _absolute_component(self, robot: "GenericRobot") -> float:
        raise NotImplementedError

    def check(self, robot: "GenericRobot") -> bool:
        if not self._started:
            return False
        delta = self._displacement_m(robot)
        if self._target_m >= 0:
            return delta >= self._target_m
        return delta <= self._target_m


class after_forward_cm(_AxisDisplacementCondition):
    """Stop after a forward/backward displacement along the reference line.

    Unlike :class:`after_cm` (which uses cumulative path length), this
    measures the *signed forward component* of displacement relative to the
    reference axis — the robot's heading at the moment this condition
    started. Any turning that happens *after* ``start()`` does not rotate
    the axis: it stays pinned to the original heading, so "20 cm forward"
    means 20 cm along *that* line. A line-follow correcting around the
    reference therefore needs slightly more path to reach the target, and a
    robot that turns 90° off the reference can never reach it.

    A positive ``cm`` triggers after the robot has moved that far forward of
    its baseline along the reference; a negative ``cm`` triggers after
    moving that far backward.

    By default the reference line is the robot's heading at ``start()``.
    Pass ``heading=<deg>`` to pin it to a targeted absolute heading instead
    — e.g. ``after_forward_cm(20, heading=0)`` means "20 cm along the
    heading-0 line", robust to the robot being crooked when the condition
    starts.

    In absolute mode (``absolute=True``) the displacement is read directly
    from ``get_distance_from_origin().forward`` — i.e. measured from the
    odometry origin, projected onto the origin heading.
    """

    def _project(self, dx: float, dy: float) -> float:
        return dx * self._cos_h + dy * self._sin_h

    def _absolute_component(self, robot: "GenericRobot") -> float:
        return robot.odometry.get_distance_from_origin().forward


class after_lateral_cm(_AxisDisplacementCondition):
    """Stop after a lateral (sideways) displacement along the reference line.

    Measures the *signed lateral component* of displacement relative to the
    reference axis — the robot's heading at the moment this condition
    started. Sign convention matches ``DistanceFromOrigin.lateral``:
    positive values point along the +lateral axis as defined there.

    Only meaningful on drivetrains that support lateral motion (e.g.
    mecanum / omni). On a differential drive this will stay near zero unless
    the robot rotates and then drives forward — in which case the
    world-frame displacement has a component sideways of the original
    heading.

    In absolute mode (``absolute=True``) the displacement is read directly
    from ``get_distance_from_origin().lateral``.
    """

    def _project(self, dx: float, dy: float) -> float:
        return -dx * self._sin_h + dy * self._cos_h

    def _absolute_component(self, robot: "GenericRobot") -> float:
        return robot.odometry.get_distance_from_origin().lateral


class after_degrees(StopCondition):
    """Stop after turning a given angle (degrees).

    Reads the heading from ``robot.odometry.get_pose().heading`` — the same
    source the motion controllers regulate on — so the measured rotation and
    the executed feedback share one frame. Tracks total unsigned rotation,
    so it works regardless of turn direction.
    """

    def __init__(self, degrees: float):
        if not isinstance(degrees, int | float):
            msg = f"degrees must be a number, got {type(degrees).__name__}"
            raise TypeError(msg)
        if degrees <= 0:
            msg = f"degrees must be > 0, got {degrees}"
            raise ValueError(msg)
        self._target_rad = math.radians(abs(degrees))
        self._start_heading: float = 0.0

    @staticmethod
    def _world_heading(robot: "GenericRobot") -> float:
        odom = getattr(robot, "odometry", None)
        if odom is None:
            msg = (
                "after_degrees requires robot.odometry "
                "(heading is read from odometry.get_pose().heading)."
            )
            raise RuntimeError(msg)
        return float(odom.get_pose().heading)

    def start(self, robot: "GenericRobot") -> None:
        self._start_heading = self._world_heading(robot)

    def check(self, robot: "GenericRobot") -> bool:
        current = self._world_heading(robot)
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
            msg = f"Expected a DigitalSensor with read(), " f"got {type(sensor).__name__}"
            raise TypeError(msg)
        self._sensor = sensor
        self._pressed = pressed

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.read() == self._pressed


class on_analog_above(StopCondition):
    """Stop when an analog sensor reading exceeds a threshold."""

    def __init__(self, sensor: "AnalogSensor", threshold: int):
        if not hasattr(sensor, "read"):
            msg = f"Expected an AnalogSensor with read(), " f"got {type(sensor).__name__}"
            raise TypeError(msg)
        if not isinstance(threshold, int | float):
            msg = f"threshold must be a number, got {type(threshold).__name__}"
            raise TypeError(msg)
        self._sensor = sensor
        self._threshold = threshold

    def check(self, robot: "GenericRobot") -> bool:
        return self._sensor.read() > self._threshold


class on_analog_below(StopCondition):
    """Stop when an analog sensor reading drops below a threshold."""

    def __init__(self, sensor: "AnalogSensor", threshold: int):
        if not hasattr(sensor, "read"):
            msg = f"Expected an AnalogSensor with read(), " f"got {type(sensor).__name__}"
            raise TypeError(msg)
        if not isinstance(threshold, int | float):
            msg = f"threshold must be a number, got {type(threshold).__name__}"
            raise TypeError(msg)
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

    def __init__(self, motor: "Motor", threshold_tps: int = 10, duration: float = 0.25):
        if not hasattr(motor, "get_position"):
            msg = f"Expected a Motor with get_position(), " f"got {type(motor).__name__}"
            raise TypeError(msg)
        if not isinstance(threshold_tps, int | float) or threshold_tps <= 0:
            msg = f"threshold_tps must be > 0, got {threshold_tps}"
            raise ValueError(msg)
        if not isinstance(duration, int | float) or duration <= 0:
            msg = f"duration must be > 0, got {duration}"
            raise ValueError(msg)
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
            msg = f"fn must be callable, got {type(fn).__name__}"
            raise TypeError(msg)
        self._fn = fn

    def check(self, robot: "GenericRobot") -> bool:
        return self._fn(robot)


def over_line(
    sensor: "IRSensor",
    black_threshold: float = 0.7,
    white_threshold: float = 0.7,
) -> StopCondition:
    """Stop after crossing a line (black then white).

    Equivalent to ``on_black(sensor) > on_white(sensor)`` — waits for
    the sensor to see black, then waits for it to see white again.

    Args:
        sensor: IR sensor to monitor.
        black_threshold: Probability threshold for the black phase.
        white_threshold: Probability threshold for the white phase.
    """
    return on_black(sensor, black_threshold) + on_white(sensor, white_threshold)
