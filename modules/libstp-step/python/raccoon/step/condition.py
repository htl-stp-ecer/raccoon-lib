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
from typing import TYPE_CHECKING, ClassVar

from raccoon.log import debug

if TYPE_CHECKING:
    from raccoon.hal import AnalogSensor, DigitalSensor, Motor
    from raccoon.robot.api import GenericRobot
    from raccoon.sensor_ir import IRSensor


class StopCondition:
    """Base class for composable stop conditions."""

    # Set the first time this condition fires, so the completion log is emitted
    # exactly once. Not reset on re-`start()` — conditions are normally built
    # fresh per motion step, so at worst a reused instance skips one log line.
    _fire_logged: bool = False

    def start(self, robot: "GenericRobot") -> None:
        """Called once when the motion step starts. Override to initialize state."""

    def check(self, robot: "GenericRobot") -> bool:
        """Return True to stop the motion. Called each update cycle.

        Public entry point. Delegates to the subclass :meth:`_evaluate` and
        emits a one-shot ``debug`` log the moment the condition first fires, so
        mission logs show *which* condition completed and its state at
        completion.

        ``_state()`` (which may poll sensors/odometry) is only evaluated on the
        firing tick, so this adds no per-tick cost to the 100 Hz motion loop.
        """
        fired = self._evaluate(robot)
        if fired and not self._fire_logged:
            self._fire_logged = True
            detail = self._state(robot)
            debug(f"condition met: {self._label()}" + (f" [{detail}]" if detail else ""))
        return fired

    def _evaluate(self, robot: "GenericRobot") -> bool:
        """Return True to stop the motion. Called each update cycle.

        Override this (not :meth:`check`) in subclasses; :meth:`check` wraps it
        with completion/pending logging.
        """
        raise NotImplementedError

    def _label(self) -> str:
        """Short human-readable name for logs. Override to add key parameters."""
        return type(self).__name__

    def _state(self, robot: "GenericRobot") -> str:
        """Optional runtime detail logged alongside the label (e.g. progress)."""
        return ""

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

    def _label(self) -> str:
        return f"({self._first._label()} -> {self._second._label()})"

    def _evaluate(self, robot: "GenericRobot") -> bool:
        if not self._first_done:
            if self._first.check(robot):
                self._first_done = True
                self._second.start(robot)
                debug(f"condition advanced: {self._label()} — first stage done, arming second")
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

    def _label(self) -> str:
        return "any(" + ", ".join(c._label() for c in self._conditions) + ")"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    def _label(self) -> str:
        return "all(" + ", ".join(c._label() for c in self._conditions) + ")"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    def _label(self) -> str:
        return f"on_black(>= {self._threshold:.2f})"

    def _state(self, robot: "GenericRobot") -> str:
        return f"p_black={self._sensor.probabilityOfBlack():.2f}"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    def _label(self) -> str:
        return f"on_white(>= {self._threshold:.2f})"

    def _state(self, robot: "GenericRobot") -> str:
        return f"p_white={self._sensor.probabilityOfWhite():.2f}"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    def _label(self) -> str:
        return f"after_seconds({self._duration:.2f})"

    def _state(self, robot: "GenericRobot") -> str:
        return f"remaining={max(0.0, self._deadline - time.monotonic()):.2f}s"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    def _label(self) -> str:
        mode = "abs" if self._absolute else "rel"
        return f"after_cm({self._target_m * 100:.1f}, {mode})"

    def _state(self, robot: "GenericRobot") -> str:
        if not self._started:
            return "not started"
        traveled = robot.odometry.get_path_length() - self._baseline_m
        return f"traveled={traveled * 100:.1f}/{self._target_m * 100:.1f}cm"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    def _label(self) -> str:
        name = self._axis_name or "axis"
        if self._absolute:
            mode = "abs"
        elif self._heading_deg is not None:
            mode = f"heading={self._heading_deg:.1f}"
        else:
            mode = "rel"
        return f"after_{name}_cm({self._target_m * 100:.1f}, {mode})"

    def _state(self, robot: "GenericRobot") -> str:
        if not self._started:
            return "not started"
        name = self._axis_name or "axis"
        return f"{name}={self._displacement_m(robot) * 100:.1f}/{self._target_m * 100:.1f}cm"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    _axis_name = "forward"

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

    _axis_name = "lateral"

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
        self._last_heading: float = 0.0
        self._accumulated_rad: float = 0.0

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
        self._last_heading = self._world_heading(robot)
        self._accumulated_rad = 0.0

    def _label(self) -> str:
        return f"after_degrees({math.degrees(self._target_rad):.1f})"

    def _state(self, robot: "GenericRobot") -> str:
        return (
            f"turned={math.degrees(self._accumulated_rad):.1f}"
            f"/{math.degrees(self._target_rad):.1f}deg"
        )

    def _evaluate(self, robot: "GenericRobot") -> bool:
        # Accumulate the TOTAL rotation by summing the per-tick heading step.
        # Each step is wrapped to (-pi, pi] so the heading discontinuity at
        # +/-pi never corrupts the running total, and abs() makes it direction-
        # agnostic. This lets targets > 180 deg (e.g. a 270 deg turn) fire — the
        # old shortest-arc difference saturated at 180 deg and never could.
        current = self._world_heading(robot)
        step = (current - self._last_heading + math.pi) % (2 * math.pi) - math.pi
        self._last_heading = current
        self._accumulated_rad += abs(step)
        return self._accumulated_rad >= self._target_rad


# ── Quaternion helpers (w, x, y, z) ─────────────────────────────────────────
# Small, allocation-light utilities for the tilt conditions below. Quaternions
# are plain 4-tuples in (w, x, y, z) order to match IMU().get_quaternion().
def _quat_norm(q: tuple[float, float, float, float]) -> float:
    w, x, y, z = q
    return math.sqrt(w * w + x * x + y * y + z * z)


def _quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    n = _quat_norm(q)
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def _quat_conj(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = q
    return (w, -x, -y, -z)


def _quat_mul(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


class _InclinationCondition(StopCondition):
    """Shared base: monitor how far the robot is tilted off horizontal.

    The tilt comes from the IMU's **DMP fused orientation quaternion**
    (``IMU().get_quaternion()``). Its roll/pitch is gravity-referenced and
    drift/vibration-immune, so — unlike the raw accelerometer — it is not
    corrupted by the robot's own acceleration or chassis vibration while
    driving. (The raw-accel approach used previously let a vibration spike
    read as a 13° incline while the robot was still flat.)

    The measurement is **relative to a flat reference orientation captured by
    :func:`mark_heading_reference`** (which stores the current DMP quaternion
    on the :class:`HeadingReferenceService`). Both ``on_incline`` and
    ``on_level`` read that same shared reference, so ``on_level`` measures
    "back to flat" against the true flat ground even though its own step only
    arms once the robot is already tilted on the ramp. The reference cancels
    the DMP's non-zero mounting/convergence baseline pitch and makes the tilt a
    pure "how far off flat am I now". It is yaw-invariant: only pitch/roll
    relative to the reference count::

        q_rel = q_ref⁻¹ · q_now
        tilt  = acos(1 − 2·(q_rel.x² + q_rel.y²))

    Because the reference is shared, ``mark_heading_reference()`` must be called
    on flat ground **and after the DMP has converged** (it needs some motion
    after boot; a mark taken while the robot is dead-still right after power-on
    can be ~18° off). Placing it shortly before the ramp — as M060 does — is
    correct.

    The scalar tilt is smoothed with an EMA (``smoothing``); the quaternion is
    already clean, so light smoothing suffices.

    Prerequisites:
        (1) :func:`mark_heading_reference` called earlier on flat ground —
        otherwise :meth:`start` raises ``RuntimeError``. (2) A working IMU whose
        firmware publishes the DMP quaternion; an all-zero quaternion (a build
        that does not stream it) also raises. The mock reports a flat identity
        quaternion, so tests inject a fake IMU (``get_quaternion()``) plus a
        reference via ``self._q_ref``.

    Subclasses implement :meth:`_triggered` to decide, given the current
    smoothed tilt in radians, whether to fire.
    """

    _MIN_QUAT_NORM: ClassVar[float] = 0.5

    def __init__(self, degrees: float, *, up_axis: str = "z", smoothing: float = 0.2):
        if not isinstance(degrees, int | float):
            msg = f"degrees must be a number, got {type(degrees).__name__}"
            raise TypeError(msg)
        if not (0.0 <= degrees <= 90.0):
            msg = f"degrees must be within 0–90, got {degrees}"
            raise ValueError(msg)
        # ``up_axis`` is accepted for backward compatibility but no longer has
        # any effect: the quaternion-relative tilt is inherently yaw-invariant
        # and picks up any pitch/roll off the flat reference regardless of
        # mounting axis. Still validated so a typo fails loudly.
        if up_axis not in ("x", "y", "z"):
            msg = f"up_axis must be one of 'x', 'y', 'z', got {up_axis!r}"
            raise ValueError(msg)
        if not (0.0 < smoothing <= 1.0):
            msg = f"smoothing must be within (0, 1], got {smoothing}"
            raise ValueError(msg)
        self._threshold_rad = math.radians(degrees)
        self._alpha = smoothing
        self._imu = None
        self._q_ref: tuple[float, float, float, float] | None = None
        self._tilt_ema_rad: float = 0.0

    def start(self, robot: "GenericRobot") -> None:
        # IMU handle is created lazily so tests can inject a fake IMU by
        # pre-setting ``self._imu`` before start() is called.
        if self._imu is None:
            from raccoon.hal import IMU

            self._imu = IMU()
        # The flat reference is the orientation captured by
        # mark_heading_reference() and shared via HeadingReferenceService, so
        # on_incline and on_level agree on where "flat" is. Tests bypass the
        # service by pre-setting ``self._q_ref``.
        if self._q_ref is None:
            from raccoon.robot.heading_reference import HeadingReferenceService

            ref = robot.get_service(HeadingReferenceService).tilt_reference_quat()
            if ref is None:
                msg = (
                    "on_incline/on_level need a flat tilt reference. Call "
                    "mark_heading_reference() on flat ground (after the DMP has "
                    "converged, i.e. after some motion) before the ramp. If the "
                    "reference is present but all-zero, the firmware is not "
                    "publishing raccoon/imu/quaternion."
                )
                raise RuntimeError(msg)
            self._q_ref = ref
        self._tilt_ema_rad = 0.0

    def _read_quat(self) -> tuple[float, float, float, float]:
        w, x, y, z = self._imu.get_quaternion()
        return (float(w), float(x), float(y), float(z))

    def _tilt_rad(self) -> float:
        """Sample the quaternion, update the EMA, return the smoothed tilt.

        Tilt is the angle between the reference "up" and the current "up",
        i.e. the pitch/roll of ``q_now`` relative to ``q_ref`` (yaw removed).
        """
        q = self._read_quat()
        if self._q_ref is None or _quat_norm(q) < self._MIN_QUAT_NORM:
            # No reference yet, or a momentarily missing sample: hold the last
            # smoothed value instead of injecting a NaN/0 spike.
            return self._tilt_ema_rad
        # q_rel = q_ref⁻¹ · q_now; for a unit quat the inverse is the conjugate.
        rel = _quat_mul(_quat_conj(self._q_ref), _quat_normalize(q))
        _, rx, ry, _ = rel
        cos_tilt = 1.0 - 2.0 * (rx * rx + ry * ry)
        tilt = math.acos(max(-1.0, min(1.0, cos_tilt)))
        a = self._alpha
        self._tilt_ema_rad = a * tilt + (1 - a) * self._tilt_ema_rad
        return self._tilt_ema_rad

    def _triggered(self, tilt_rad: float) -> bool:
        raise NotImplementedError

    def _state(self, robot: "GenericRobot") -> str:
        if self._q_ref is None:
            return "not started"
        tilt = math.degrees(self._tilt_ema_rad)
        return f"tilt={tilt:.1f}deg (threshold={math.degrees(self._threshold_rad):.1f})"

    def _evaluate(self, robot: "GenericRobot") -> bool:
        if self._q_ref is None:
            return False
        return self._triggered(self._tilt_rad())


class on_incline(_InclinationCondition):
    """Stop once the robot is tilted off horizontal by at least an angle.

    Detects when the chassis pitches (or rolls) up onto a slope — e.g. the
    front wheels climbing onto a ramp. The inclination is derived from the
    IMU DMP orientation quaternion relative to the flat reference captured at
    step start (see :class:`_InclinationCondition`); the condition fires as
    soon as the smoothed tilt reaches ``min_deg``.

    Pair it with :func:`on_level` to bracket a ramp: ``on_incline()`` marks
    entering the slope, ``on_level()`` marks cresting back onto flat ground.
    :func:`over_ramp` wires both together.

    Prerequisites:
        A working IMU. See :class:`_InclinationCondition` for tuning notes
        on ``up_axis`` and ``smoothing``.

    Args:
        min_deg: Tilt threshold in degrees (0–90). Fires when the measured
            inclination is at least this large. A 20° ramp reads ~20°, so a
            threshold around 8–12° detects entry robustly while ignoring
            floor roughness and driving jitter. Default 8.0.
        up_axis: Deprecated / no effect (the quaternion tilt is yaw-invariant);
            accepted only for backward compatibility.
        smoothing: EMA factor in (0, 1] applied to the tilt angle to reject any
            residual jitter (default 0.2).

    Returns:
        A :class:`StopCondition` that fires while tilted.

    Example::

        from raccoon.step.motion import drive_forward
        from raccoon.step.condition import on_incline

        # Drive until the robot noses up onto the ramp.
        drive_forward(speed=0.4).until(on_incline(10))
    """

    def __init__(self, min_deg: float = 8.0, *, up_axis: str = "z", smoothing: float = 0.2):
        super().__init__(min_deg, up_axis=up_axis, smoothing=smoothing)

    def _label(self) -> str:
        return f"on_incline(>= {math.degrees(self._threshold_rad):.1f}deg)"

    def _triggered(self, tilt_rad: float) -> bool:
        return tilt_rad >= self._threshold_rad


class on_level(_InclinationCondition):
    """Stop once the robot is back to (near-)horizontal.

    The counterpart to :func:`on_incline`: it fires when the smoothed tilt
    (from the IMU DMP orientation, relative to the flat start reference)
    drops to at most ``max_deg`` — i.e. the robot has crested a ramp and
    settled onto flat ground, or finished descending a slope.

    Because it triggers on a *low* tilt, using it on its own from a flat
    start would fire immediately. It is meant to run *after* the robot is
    already tilted — typically as the second stage of a chain, e.g.
    ``on_incline(10) + on_level(4)`` (which is what :func:`over_ramp`
    builds).

    Prerequisites:
        A working IMU. See :class:`_InclinationCondition` for tuning notes.

    Args:
        max_deg: Tilt threshold in degrees (0–90). Fires when the measured
            inclination is at most this large. Keep it a few degrees below
            the matching ``on_incline`` threshold so transitional wobble
            does not fire it early (hysteresis). Default 4.0.
        up_axis: Deprecated / no effect (the quaternion tilt is yaw-invariant);
            accepted only for backward compatibility.
        smoothing: EMA factor in (0, 1] applied to the tilt angle (default 0.2).

    Returns:
        A :class:`StopCondition` that fires while (near) level.

    Example::

        from raccoon.step.condition import on_incline, on_level

        # Climb onto the ramp, then keep going until back on the flat top.
        drive_forward(speed=0.4).until(on_incline(10) + on_level(4))
    """

    def __init__(self, max_deg: float = 4.0, *, up_axis: str = "z", smoothing: float = 0.2):
        super().__init__(max_deg, up_axis=up_axis, smoothing=smoothing)

    def _label(self) -> str:
        return f"on_level(<= {math.degrees(self._threshold_rad):.1f}deg)"

    def _triggered(self, tilt_rad: float) -> bool:
        return tilt_rad <= self._threshold_rad


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

    def _label(self) -> str:
        return f"on_digital(pressed={self._pressed})"

    def _state(self, robot: "GenericRobot") -> str:
        return f"read={self._sensor.read()}"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    def _label(self) -> str:
        return f"on_analog_above({self._threshold})"

    def _state(self, robot: "GenericRobot") -> str:
        return f"read={self._sensor.read()}"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    def _label(self) -> str:
        return f"on_analog_below({self._threshold})"

    def _state(self, robot: "GenericRobot") -> str:
        return f"read={self._sensor.read()}"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    def _label(self) -> str:
        return f"stall_detected(< {self._threshold_tps}tps for {self._duration:.2f}s)"

    def _state(self, robot: "GenericRobot") -> str:
        if self._stall_start is None:
            return "moving"
        return f"stalled {time.monotonic() - self._stall_start:.2f}/{self._duration:.2f}s"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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

    def _label(self) -> str:
        name = getattr(self._fn, "__name__", None) or repr(self._fn)
        return f"custom({name})"

    def _evaluate(self, robot: "GenericRobot") -> bool:
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


def over_ramp(
    enter_deg: float = 8.0,
    exit_deg: float = 4.0,
    *,
    up_axis: str = "z",
    smoothing: float = 0.2,
) -> StopCondition:
    """Stop after crossing a ramp (tilt up, then back to level).

    Equivalent to ``on_incline(enter_deg) + on_level(exit_deg)`` — waits for
    the robot to tilt onto the slope, then waits for it to settle back onto
    flat ground once it crests. Both stages read the DMP orientation relative
    to the flat reference from :func:`mark_heading_reference` (which must have
    been called on flat ground beforehand), so no odometry or line sensor is
    needed. Use it to drive the whole way up and over a ramp with a single
    stop condition::

        drive_forward(speed=0.4).until(over_ramp())

    To act on just one edge, use :func:`on_incline` (entering) or
    :func:`on_level` (leaving) on their own.

    Args:
        enter_deg: Tilt at which the ramp is considered entered (default 8.0).
        exit_deg: Tilt at or below which the robot is considered back on the
            flat (default 4.0). Keep it below ``enter_deg`` for hysteresis.
        up_axis: Deprecated / no effect (kept for backward compatibility).
        smoothing: EMA factor in (0, 1] for both stages (default 0.2).

    Returns:
        A :class:`StopCondition` that fires once the ramp has been crossed.
    """
    if exit_deg > enter_deg:
        msg = f"exit_deg ({exit_deg}) should be <= enter_deg ({enter_deg}) for hysteresis"
        raise ValueError(msg)
    return on_incline(enter_deg, up_axis=up_axis, smoothing=smoothing) + on_level(
        exit_deg, up_axis=up_axis, smoothing=smoothing
    )
