"""Closed-loop ``goto`` motion step — drive to an ABSOLUTE world pose.

``goto(x, y, theta)`` is a *navigate-to-pose* primitive: each control tick it
reads the live pose estimate from the localization particle filter
(``robot.localization.get_pose()``), computes the world-frame error to the
target, rotates the translational error into the robot body frame using the
current heading, and commands ``robot.drive`` a proportional velocity toward
the goal (simultaneously correcting heading when a target ``theta`` is given).
This is fundamentally different from the dead-reckoning relative drives
(``drive_forward`` etc.): it regulates on absolute feedback, so it converges
on the target pose regardless of accumulated odometry drift.

The proportional control law lives in :func:`_compute_body_velocity`, a small
pure helper that takes plain numbers / a tiny pose-like duck type so the math
is unit-testable without a robot or C++ types. The real step feeds it the
genuine localization ``Pose``.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, NamedTuple

from raccoon.foundation import ChassisVelocity
from raccoon.motion import LinearAxis

from .. import dsl
from .motion_step import MotionStep
from .path.passes.spline import sample_centripetal_catmull_rom

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

    from ...condition import StopCondition


# Proportional gains. Output is a body-frame velocity in m/s and rad/s; the
# step clamps the magnitude to the configured per-axis max velocity and then
# scales by the user ``speed`` fraction, so the absolute gain value only sets
# how aggressively the robot approaches the target before saturating.
_LINEAR_GAIN = 2.0  # (m/s) per metre of position error
_ANGULAR_GAIN = 3.0  # (rad/s) per radian of heading error


class BodyVelocityCommand(NamedTuple):
    """Result of the pure goto control law (pre-saturation, pre-speed-scale)."""

    vx: float  # body-forward velocity, m/s (positive = forward)
    vy: float  # body-lateral velocity, m/s (positive = right, per ChassisVelocity)
    wz: float  # angular velocity, rad/s (positive = counter-clockwise)
    reached: bool  # within position tol AND (heading tol or theta not requested)


def _world_to_body(dx_m: float, dy_m: float, heading_rad: float) -> tuple[float, float]:
    """Rotate a world-frame translation into the body frame.

    Returns ``(forward, strafe_right)`` matching the convention used by the
    path executor's ``_world_to_body``: ``forward`` is +X-body (heading
    direction), ``strafe_right`` is +Y-body to the robot's right.
    """
    return (
        dx_m * math.cos(heading_rad) + dy_m * math.sin(heading_rad),
        dx_m * math.sin(heading_rad) - dy_m * math.cos(heading_rad),
    )


def _compute_body_velocity(
    current_pose,
    target,
    speed: float = 1.0,
    *,
    pos_tol_m: float = 0.02,
    heading_tol_rad: float = math.radians(3.0),
    linear_gain: float = _LINEAR_GAIN,
    angular_gain: float = _ANGULAR_GAIN,
) -> BodyVelocityCommand:
    """Pure proportional control law for ``goto`` — no robot required.

    Args:
        current_pose: Pose-like with ``.position`` (indexable x, y in metres)
            and ``.heading`` (radians).
        target: Target-like with ``.x_m``, ``.y_m`` (metres) and
            ``.theta_rad`` (radians or ``None`` to ignore heading).
        speed: Velocity scale in ``(0, 1]`` applied to the proportional output.
        pos_tol_m: Position tolerance (metres) for ``reached``.
        heading_tol_rad: Heading tolerance (radians) for ``reached`` (ignored
            when ``target.theta_rad`` is ``None``).
        linear_gain: Proportional gain mapping metres of error → m/s.
        angular_gain: Proportional gain mapping radians of error → rad/s.

    Returns:
        :class:`BodyVelocityCommand` — body-frame ``(vx, vy, wz)`` scaled by
        ``speed`` plus a ``reached`` flag. Velocities are intentionally NOT
        clamped to per-axis maxima here (the step does that against the
        robot's configured limits); this keeps the helper pure.
    """
    heading_rad = float(current_pose.heading)
    dx_m = float(target.x_m) - float(current_pose.position[0])
    dy_m = float(target.y_m) - float(current_pose.position[1])
    dist_m = math.hypot(dx_m, dy_m)

    forward_m, strafe_right_m = _world_to_body(dx_m, dy_m, heading_rad)

    pos_reached = dist_m <= pos_tol_m

    theta_rad = getattr(target, "theta_rad", None)
    if theta_rad is None:
        dtheta_rad = 0.0
        heading_reached = True
    else:
        dtheta_rad = math.remainder(float(theta_rad) - heading_rad, 2.0 * math.pi)
        heading_reached = abs(dtheta_rad) <= heading_tol_rad

    reached = pos_reached and heading_reached

    # Zero the translational command once inside the position tolerance so the
    # robot doesn't jitter around the goal while it finishes correcting heading.
    if pos_reached:
        vx = 0.0
        vy = 0.0
    else:
        vx = linear_gain * forward_m * speed
        vy = linear_gain * strafe_right_m * speed

    wz = angular_gain * dtheta_rad * speed if not heading_reached else 0.0

    return BodyVelocityCommand(vx=vx, vy=vy, wz=wz, reached=reached)


def _clamp(value: float, limit: float) -> float:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


class _Target(NamedTuple):
    x_m: float
    y_m: float
    theta_rad: float | None


@dsl(hidden=True)
class Goto(MotionStep):
    """Internal closed-loop navigate-to-pose step — users go through ``goto()``.

    Reads the particle-filter pose each tick and commands a proportional
    body-frame velocity toward the absolute target ``(x_m, y_m, theta_rad)``.
    Requires ``robot.localization`` (the particle filter); raises at start
    otherwise.
    """

    def __init__(
        self,
        x_m: float,
        y_m: float,
        theta_rad: float | None = None,
        speed: float = 1.0,
        pos_tol_m: float = 0.02,
        heading_tol_rad: float = math.radians(3.0),
    ) -> None:
        super().__init__()
        if not isinstance(speed, int | float):
            msg = f"speed must be a number, got {type(speed).__name__}"
            raise TypeError(msg)
        if not (0.0 < speed <= 1.0):
            msg = f"speed must be in (0.0, 1.0], got {speed}"
            raise ValueError(msg)
        if pos_tol_m <= 0.0:
            msg = f"pos_tol_m must be > 0, got {pos_tol_m}"
            raise ValueError(msg)
        if heading_tol_rad <= 0.0:
            msg = f"heading_tol_rad must be > 0, got {heading_tol_rad}"
            raise ValueError(msg)
        self._target = _Target(x_m=x_m, y_m=y_m, theta_rad=theta_rad)
        self._speed = speed
        self._pos_tol_m = pos_tol_m
        self._heading_tol_rad = heading_tol_rad

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive", "localization"})

    def _generate_signature(self) -> str:
        if self._target.theta_rad is None:
            theta = "None"
        else:
            theta = f"{math.degrees(self._target.theta_rad):.1f}"
        return f"Goto(x={self._target.x_m:.2f}, y={self._target.y_m:.2f}, theta={theta})"

    def on_start(self, robot: "GenericRobot") -> None:
        if getattr(robot, "localization", None) is None:
            msg = "goto() requires robot.localization (the particle filter)"
            raise RuntimeError(msg)

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        pose = robot.localization.get_pose()
        cmd = _compute_body_velocity(
            pose,
            self._target,
            self._speed,
            pos_tol_m=self._pos_tol_m,
            heading_tol_rad=self._heading_tol_rad,
        )
        if cmd.reached:
            return True

        cfg = robot.motion_pid_config
        robot.drive.set_velocity(
            ChassisVelocity(
                _clamp(cmd.vx, cfg.linear.max_velocity),
                _clamp(cmd.vy, cfg.lateral.max_velocity),
                _clamp(cmd.wz, cfg.angular.max_velocity),
            )
        )
        robot.drive.update(dt)
        return False


def _anchor_to_world_target(
    anchor_pose,
    forward_m: float,
    left_m: float,
    dtheta_rad: float | None,
) -> tuple[float, float, float | None]:
    """Compose a body-frame delta onto an anchor pose → absolute world target.

    Pure helper (no robot required). Given an ``anchor_pose`` (pose-like with
    ``.position`` indexable x, y in metres and ``.heading`` in radians) and a
    body-frame displacement ``(forward_m, left_m)`` plus an optional heading
    change ``dtheta_rad``, returns the absolute world target
    ``(x_m, y_m, theta_rad)``.

    The body→world rotation uses the SAME convention as the spline integration
    (``segments_to_spline_waypoints``) and the inverse of goto's
    ``_world_to_body``: ``+forward`` is the anchor heading direction and
    ``+left`` is 90° CCW of it::

        dx = forward·cos(h) - left·sin(h)
        dy = forward·sin(h) + left·cos(h)

    The target heading is ``anchor.heading + dtheta_rad`` (or ``None`` when no
    heading correction is requested).
    """
    h = float(anchor_pose.heading)
    cos_h = math.cos(h)
    sin_h = math.sin(h)
    dx = forward_m * cos_h - left_m * sin_h
    dy = forward_m * sin_h + left_m * cos_h
    x_m = float(anchor_pose.position[0]) + dx
    y_m = float(anchor_pose.position[1]) + dy
    theta_rad = None if dtheta_rad is None else h + dtheta_rad
    return (x_m, y_m, theta_rad)


@dsl(hidden=True)
class GotoRelative(MotionStep):
    """Internal closed-loop navigate-to-RELATIVE-pose step.

    Captures the localization pose at ``on_start`` as an anchor, composes the
    body-frame delta ``(forward_m, left_m, dtheta_rad)`` onto it to obtain an
    absolute world target, then runs the IDENTICAL proportional closed-loop as
    :class:`Goto` (reusing :func:`_compute_body_velocity`).  Used by the
    ``to_absolute`` optimizer pass to turn dead-reckoning relative legs into
    feedback-regulated navigate-to-pose moves whose absolute target is unknown
    at compile time but fixed at run time relative to the run-start pose.

    Requires ``robot.localization`` (the particle filter); raises at start
    otherwise.
    """

    def __init__(
        self,
        forward_m: float,
        left_m: float,
        dtheta_rad: float | None = None,
        speed: float = 1.0,
        pos_tol_m: float = 0.02,
        heading_tol_rad: float = math.radians(3.0),
    ) -> None:
        super().__init__()
        if not isinstance(speed, int | float):
            msg = f"speed must be a number, got {type(speed).__name__}"
            raise TypeError(msg)
        if not (0.0 < speed <= 1.0):
            msg = f"speed must be in (0.0, 1.0], got {speed}"
            raise ValueError(msg)
        if pos_tol_m <= 0.0:
            msg = f"pos_tol_m must be > 0, got {pos_tol_m}"
            raise ValueError(msg)
        if heading_tol_rad <= 0.0:
            msg = f"heading_tol_rad must be > 0, got {heading_tol_rad}"
            raise ValueError(msg)
        self._forward_m = forward_m
        self._left_m = left_m
        self._dtheta_rad = dtheta_rad
        self._speed = speed
        self._pos_tol_m = pos_tol_m
        self._heading_tol_rad = heading_tol_rad
        # Resolved at on_start once the anchor pose is known.
        self._target: _Target | None = None

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive", "localization"})

    def _generate_signature(self) -> str:
        dtheta = "None" if self._dtheta_rad is None else f"{math.degrees(self._dtheta_rad):.1f}"
        return (
            f"GotoRelative(fwd={self._forward_m:.2f}, " f"left={self._left_m:.2f}, dtheta={dtheta})"
        )

    def on_start(self, robot: "GenericRobot") -> None:
        if getattr(robot, "localization", None) is None:
            msg = "goto_relative() requires robot.localization (the particle filter)"
            raise RuntimeError(msg)
        anchor = robot.localization.get_pose()
        x_m, y_m, theta_rad = _anchor_to_world_target(
            anchor, self._forward_m, self._left_m, self._dtheta_rad
        )
        self._target = _Target(x_m=x_m, y_m=y_m, theta_rad=theta_rad)

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        pose = robot.localization.get_pose()
        cmd = _compute_body_velocity(
            pose,
            self._target,
            self._speed,
            pos_tol_m=self._pos_tol_m,
            heading_tol_rad=self._heading_tol_rad,
        )
        if cmd.reached:
            return True

        cfg = robot.motion_pid_config
        robot.drive.set_velocity(
            ChassisVelocity(
                _clamp(cmd.vx, cfg.linear.max_velocity),
                _clamp(cmd.vy, cfg.lateral.max_velocity),
                _clamp(cmd.wz, cfg.angular.max_velocity),
            )
        )
        robot.drive.update(dt)
        return False


def _waypoints_to_world_targets(
    anchor_pose,
    waypoints: list[tuple[float, float, float | None]],
) -> list[tuple[float, float, float | None]]:
    """Compose a list of body-frame waypoints onto an anchor → absolute targets.

    Pure helper (no robot required). Each waypoint is a body-frame displacement
    ``(forward_m, left_m, dtheta_rad)`` expressed in the SAME run-start frame
    (i.e. all relative to the single ``anchor_pose``, NOT chained leg-to-leg).
    Returns the list of absolute world targets ``(x_m, y_m, theta_rad)``, one
    per waypoint, each computed via :func:`_anchor_to_world_target`.

    Because every waypoint shares the one anchor, drift in the live pose does
    not accumulate across legs — leg N always aims at a fixed world point.
    """
    return [
        _anchor_to_world_target(anchor_pose, forward_m, left_m, dtheta_rad)
        for (forward_m, left_m, dtheta_rad) in waypoints
    ]


@dsl(hidden=True)
class GotoWaypoints(MotionStep):
    """Internal closed-loop navigate through a sequence of ABSOLUTE waypoints.

    Holds a list of body-frame waypoints ``[(forward_m, left_m, dtheta_rad|None),
    ...]`` expressed in the run-start frame (all relative to ONE anchor — the
    pose captured at ``on_start`` — not chained leg-to-leg).  At ``on_start`` it
    captures the localization pose ONCE and precomputes the ABSOLUTE world
    target for every waypoint via :func:`_waypoints_to_world_targets`.  It then
    runs the IDENTICAL proportional closed-loop as :class:`Goto` /
    :class:`GotoRelative` (reusing :func:`_compute_body_velocity`) toward the
    current target; when that target is reached it advances to the next, and the
    step finishes once the LAST target is reached.

    Because all targets are fixed at run start relative to a single anchor,
    leg N drives to a fixed world point — accumulated drift does NOT chain from
    one leg to the next (unlike emitting one re-anchoring :class:`GotoRelative`
    per leg).  Used by the ``to_absolute`` optimizer pass: it collects a whole
    contiguous run's relative waypoints and emits ONE ``GotoWaypoints``.

    Requires ``robot.localization`` (the particle filter); raises at start
    otherwise.
    """

    def __init__(
        self,
        waypoints: list[tuple[float, float, float | None]],
        speed: float = 1.0,
        pos_tol_m: float = 0.02,
        heading_tol_rad: float = math.radians(3.0),
    ) -> None:
        super().__init__()
        if not isinstance(speed, int | float):
            msg = f"speed must be a number, got {type(speed).__name__}"
            raise TypeError(msg)
        if not (0.0 < speed <= 1.0):
            msg = f"speed must be in (0.0, 1.0], got {speed}"
            raise ValueError(msg)
        if pos_tol_m <= 0.0:
            msg = f"pos_tol_m must be > 0, got {pos_tol_m}"
            raise ValueError(msg)
        if heading_tol_rad <= 0.0:
            msg = f"heading_tol_rad must be > 0, got {heading_tol_rad}"
            raise ValueError(msg)
        if not waypoints:
            msg = "GotoWaypoints requires at least one waypoint"
            raise ValueError(msg)
        self._waypoints: list[tuple[float, float, float | None]] = list(waypoints)
        self._speed = speed
        self._pos_tol_m = pos_tol_m
        self._heading_tol_rad = heading_tol_rad
        # Resolved at on_start once the anchor pose is known.
        self._targets: list[_Target] = []
        self._index = 0

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive", "localization"})

    def _generate_signature(self) -> str:
        parts = []
        for forward_m, left_m, dtheta_rad in self._waypoints:
            dtheta = "None" if dtheta_rad is None else f"{math.degrees(dtheta_rad):.1f}"
            parts.append(f"(fwd={forward_m:.2f}, left={left_m:.2f}, dtheta={dtheta})")
        return f"GotoWaypoints(n={len(self._waypoints)}: {', '.join(parts)})"

    def on_start(self, robot: "GenericRobot") -> None:
        if getattr(robot, "localization", None) is None:
            msg = "goto waypoints require robot.localization (the particle filter)"
            raise RuntimeError(msg)
        anchor = robot.localization.get_pose()
        self._targets = [
            _Target(x_m=x_m, y_m=y_m, theta_rad=theta_rad)
            for (x_m, y_m, theta_rad) in _waypoints_to_world_targets(anchor, self._waypoints)
        ]
        self._index = 0

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        pose = robot.localization.get_pose()
        target = self._targets[self._index]
        cmd = _compute_body_velocity(
            pose,
            target,
            self._speed,
            pos_tol_m=self._pos_tol_m,
            heading_tol_rad=self._heading_tol_rad,
        )
        if cmd.reached:
            if self._index >= len(self._targets) - 1:
                return True
            self._index += 1
            return False

        cfg = robot.motion_pid_config
        robot.drive.set_velocity(
            ChassisVelocity(
                _clamp(cmd.vx, cfg.linear.max_velocity),
                _clamp(cmd.vy, cfg.lateral.max_velocity),
                _clamp(cmd.wz, cfg.angular.max_velocity),
            )
        )
        robot.drive.update(dt)
        return False


def _free_axis_world_dir(axis: "LinearAxis", heading_rad: float) -> tuple[float, float]:
    """World-frame unit vector of a single LINEAR travel axis at ``heading_rad``.

    Matches the integration convention used by ``_run_to_waypoints`` /
    ``segments_to_spline_waypoints`` (and the inverse of goto's
    ``_world_to_body``):

    - ``Forward`` → the heading direction ``(cos h, sin h)``.
    - ``Lateral`` → the +left direction ``(-sin h, cos h)`` (90° CCW of forward).

    The segment's ``sign`` (direction along the axis) is applied by the caller;
    this returns the unsigned positive-axis unit vector.
    """
    if axis == LinearAxis.Forward:
        return (math.cos(heading_rad), math.sin(heading_rad))
    # Lateral → +left, consistent with d·(-sin h, cos h) in _run_to_waypoints.
    return (-math.sin(heading_rad), math.cos(heading_rad))


def _hold_velocity(
    anchor,
    current,
    free_axis: "LinearAxis",
    sign: float,
    speed: float,
    *,
    pos_gain: float = _LINEAR_GAIN,
    heading_gain: float = _ANGULAR_GAIN,
) -> tuple[float, float, float]:
    """Pure control law for :class:`AbsoluteHoldMove` — no robot required.

    Holds 2 of 3 world DOF absolutely (the CROSS-axis position + heading,
    read off localization) while driving the FREE axis open-loop. Given the
    ``anchor`` pose (captured at start) and the live ``current`` pose (both
    pose-like with ``.position`` indexable x, y in metres and ``.heading`` in
    radians):

    - The free-axis world unit vector is taken at the ANCHOR heading (the line
      the move travels along is fixed at start, like ``GotoWaypoints`` targets).
    - The cross world unit vector is the free vector rotated 90° CCW.
    - ``cross_err = (current_pos - anchor_pos) · cross_dir`` — drift off the
      intended travel line (should be 0).
    - Desired WORLD velocity = ``sign·speed·1.0`` along the free vector minus
      ``pos_gain·cross_err`` along the cross vector. (The free magnitude is the
      raw ``sign·speed`` fraction; the step clamps it to the configured per-axis
      max velocity, so ``speed`` is a fraction of max like the relative drives.)
    - That world velocity is rotated into the BODY frame using the CURRENT
      heading (so drift in heading still produces a correct body command), via
      the same ``_world_to_body`` convention goto uses.
    - ``wz = heading_gain · wrap(anchor_heading - current_heading)`` corrects
      heading back onto the anchor heading.

    Returns body-frame ``(vx, vy, wz)`` — NOT clamped here (the step clamps
    against the robot's configured limits, keeping this helper pure).
    """
    anchor_h = float(anchor.heading)
    cur_h = float(current.heading)

    free_x, free_y = _free_axis_world_dir(free_axis, anchor_h)
    # Cross direction = free rotated 90° CCW: (x, y) -> (-y, x).
    cross_x, cross_y = -free_y, free_x

    dpos_x = float(current.position[0]) - float(anchor.position[0])
    dpos_y = float(current.position[1]) - float(anchor.position[1])
    cross_err = dpos_x * cross_x + dpos_y * cross_y

    free_speed = sign * speed
    # World-frame desired velocity: drive the free axis, correct the cross axis.
    vx_world = free_speed * free_x - pos_gain * cross_err * cross_x
    vy_world = free_speed * free_y - pos_gain * cross_err * cross_y

    # Rotate world velocity into the body frame using the CURRENT heading
    # (same convention as goto's _world_to_body: forward, strafe_right).
    vx, vy = _world_to_body(vx_world, vy_world, cur_h)

    heading_err = math.remainder(anchor_h - cur_h, 2.0 * math.pi)
    wz = heading_gain * heading_err

    return (vx, vy, wz)


@dsl(hidden=True)
class AbsoluteHoldMove(MotionStep):
    """Closed-loop-on-localization single-axis drive that holds 2 world DOF.

    A SENSOR-bounded single-axis drive (e.g. ``strafe_left().until(on_black)``)
    pins 2 of 3 world DOF — the cross-axis position and the heading — and leaves
    only the travel (free) axis free until the sensor fires. This step drives
    the free body-axis open-loop at ``speed`` while CORRECTING drift on the two
    pinned DOF against their absolute (localization) targets DURING the move,
    until the original ``.until()`` condition fires.

    At ``on_start`` it captures the localization pose as an anchor. The pinned
    target is: stay on the world line through ``anchor_pos`` along the free-axis
    direction (cross-axis displacement = 0) and hold ``anchor_heading``. Each
    tick it reads the live pose and feeds :func:`_hold_velocity` (read-only on
    localization — it never writes the filter). The result is the whole
    ``to_absolute`` path regulating on the particle filter instead of odometry
    dead reckoning.

    Requires ``robot.localization`` (the particle filter); raises at start
    otherwise.
    """

    def __init__(
        self,
        free_axis: "LinearAxis",
        sign: float,
        speed: float,
        condition: "StopCondition",
    ) -> None:
        super().__init__()
        if not isinstance(speed, int | float):
            msg = f"speed must be a number, got {type(speed).__name__}"
            raise TypeError(msg)
        self._free_axis = free_axis
        self._sign = sign
        self._speed = speed
        self._condition = condition
        self._anchor = None

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive", "localization"})

    def _generate_signature(self) -> str:
        axis = "Forward" if self._free_axis == LinearAxis.Forward else "Lateral"
        return f"AbsoluteHoldMove(free={axis}, sign={self._sign:+.0f}, until=...)"

    def on_start(self, robot: "GenericRobot") -> None:
        if getattr(robot, "localization", None) is None:
            msg = "AbsoluteHoldMove requires robot.localization (the particle filter)"
            raise RuntimeError(msg)
        self._anchor = robot.localization.get_pose()
        if self._condition is not None:
            self._condition.start(robot)

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        current = robot.localization.get_pose()
        vx, vy, wz = _hold_velocity(
            self._anchor,
            current,
            self._free_axis,
            self._sign,
            self._speed,
        )

        cfg = robot.motion_pid_config
        robot.drive.set_velocity(
            ChassisVelocity(
                _clamp(vx, cfg.linear.max_velocity),
                _clamp(vy, cfg.lateral.max_velocity),
                _clamp(wz, cfg.angular.max_velocity),
            )
        )
        robot.drive.update(dt)

        return bool(self._condition is not None and self._condition.check(robot))


# Default pure-pursuit lookahead distance (metres). Picks a point this far
# ahead along the polyline as the carrot the robot chases — large enough to
# smooth out sample-to-sample jitter, small enough to track curvature.
_DEFAULT_LOOKAHEAD_M = 0.15

# Heading-channel sentinels for SplineFollow.
_HEADING_HOLD = "hold"
_HEADING_TANGENT = "tangent"


def _project_index(
    path: list[tuple[float, float]],
    point: tuple[float, float],
    start_idx: int,
    window: int,
) -> int:
    """Project ``point`` onto ``path``, scanning forward from ``start_idx``.

    Pure helper (no robot). Searches the polyline indices in
    ``[start_idx, start_idx + window]`` (clamped to the path) for the one whose
    point is nearest ``point`` and returns it. The search is MONOTONIC: it never
    returns an index below ``start_idx``, so progress along the curve cannot run
    backward even if the robot wobbles laterally near an earlier sample.

    Args:
        path: Dense world polyline ``[(x, y), ...]`` (at least 1 point).
        point: The query point ``(x, y)`` (the robot position).
        start_idx: Lower bound — never project before this index.
        window: How many indices ahead of ``start_idx`` to consider.

    Returns:
        The index in ``path`` of the nearest forward point.
    """
    n = len(path)
    if n == 0:
        msg = "_project_index requires a non-empty path"
        raise ValueError(msg)
    lo = max(0, min(start_idx, n - 1))
    hi = min(n - 1, lo + max(0, window))
    best_idx = lo
    best_d2 = float("inf")
    px, py = point
    for i in range(lo, hi + 1):
        dx = path[i][0] - px
        dy = path[i][1] - py
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best_idx = i
    return best_idx


def _lookahead_point(
    path: list[tuple[float, float]],
    idx: int,
    lookahead_m: float,
) -> tuple[float, float]:
    """Walk ``lookahead_m`` of arc length forward from ``idx`` along ``path``.

    Pure helper (no robot). Accumulates straight-line segment lengths starting
    at ``path[idx]`` until the running arc length reaches ``lookahead_m``, then
    returns that point (the last segment is linearly interpolated for a smooth
    carrot). If the remaining path is shorter than ``lookahead_m``, returns the
    final point — so near the end the carrot clamps to the goal.

    Args:
        path: Dense world polyline ``[(x, y), ...]`` (at least 1 point).
        idx: Index to start walking from (the projected progress index).
        lookahead_m: Arc-length distance to walk forward (metres).

    Returns:
        The lookahead target point ``(x, y)``.
    """
    n = len(path)
    if n == 0:
        msg = "_lookahead_point requires a non-empty path"
        raise ValueError(msg)
    i = max(0, min(idx, n - 1))
    if lookahead_m <= 0.0 or i >= n - 1:
        return path[-1] if i >= n - 1 else path[i]

    remaining = lookahead_m
    for j in range(i, n - 1):
        ax, ay = path[j]
        bx, by = path[j + 1]
        seg = math.hypot(bx - ax, by - ay)
        if seg >= remaining:
            if seg < 1e-12:
                return (bx, by)
            t = remaining / seg
            return (ax + t * (bx - ax), ay + t * (by - ay))
        remaining -= seg
    return path[-1]


def _pursuit_velocity(
    current_pose,
    lookahead_pt: tuple[float, float],
    heading_target: float,
    speed: float,
    *,
    linear_gain: float = _LINEAR_GAIN,
    angular_gain: float = _ANGULAR_GAIN,
) -> tuple[float, float, float]:
    """Pure pure-pursuit control law — no robot required.

    Drives the robot toward the lookahead carrot ``lookahead_pt`` (a world point
    already chosen ``lookahead_m`` ahead along the curve) and independently
    rotates toward ``heading_target``. Unlike :func:`_compute_body_velocity`,
    there is NO at-target zeroing of the translational command — the carrot is
    always ahead, so continuous pursuit keeps the robot flowing along the curve.

    - World error ``(tx - x, ty - y)`` is rotated into the body frame via
      :func:`_world_to_body` using the CURRENT heading → ``(forward, strafe)``.
    - ``vx = linear_gain · forward · speed``, ``vy = linear_gain · strafe · speed``
      (``vy`` positive = right, matching ``ChassisVelocity``).
    - ``wz = angular_gain · wrap(heading_target − current_heading) · speed``
      (positive = counter-clockwise).

    Args:
        current_pose: Pose-like with ``.position`` (indexable x, y in metres)
            and ``.heading`` (radians).
        lookahead_pt: World carrot point ``(x, y)`` in metres.
        heading_target: Absolute target heading (radians) for the independent
            heading channel.
        speed: Velocity scale applied to the proportional output.
        linear_gain: Gain mapping metres of carrot error → m/s.
        angular_gain: Gain mapping radians of heading error → rad/s.

    Returns:
        Body-frame ``(vx, vy, wz)`` — NOT clamped (the step clamps against the
        robot's configured per-axis limits, keeping this helper pure).
    """
    heading_rad = float(current_pose.heading)
    dx_m = float(lookahead_pt[0]) - float(current_pose.position[0])
    dy_m = float(lookahead_pt[1]) - float(current_pose.position[1])

    forward_m, strafe_right_m = _world_to_body(dx_m, dy_m, heading_rad)
    vx = linear_gain * forward_m * speed
    vy = linear_gain * strafe_right_m * speed

    dtheta_rad = math.remainder(float(heading_target) - heading_rad, 2.0 * math.pi)
    wz = angular_gain * dtheta_rad * speed

    return (vx, vy, wz)


def _path_tangent_heading(path: list[tuple[float, float]], idx: int) -> float:
    """World heading (radians) of the local polyline tangent at ``idx``.

    Uses the segment leaving ``path[idx]`` (or entering it, at the last point):
    ``atan2(dy, dx)``. Falls back to the previous segment if the local segment
    is degenerate (zero length).
    """
    n = len(path)
    if n < 2:
        return 0.0
    i = max(0, min(idx, n - 1))
    j = i if i < n - 1 else i - 1
    ax, ay = path[j]
    bx, by = path[j + 1]
    dx, dy = bx - ax, by - ay
    if dx * dx + dy * dy < 1e-18 and j > 0:
        ax, ay = path[j - 1]
        bx, by = path[j]
        dx, dy = bx - ax, by - ay
    return math.atan2(dy, dx)


@dsl(hidden=True)
class SplineFollow(MotionStep):
    """Continuous pure-pursuit follower of a Catmull-Rom curve, closed-loop.

    Rides a centripetal Catmull-Rom spline SMOOTHLY on the localization particle
    filter instead of stop-and-go stepping through dense waypoints. At
    ``on_start`` it captures the live pose as an anchor, transforms the
    body-frame control points ``(forward_m, left_m)`` (run-start frame, same
    format as :class:`GotoWaypoints`) into absolute world points via
    :func:`_anchor_to_world_target`, and densely samples the curve through them
    (:func:`sample_centripetal_catmull_rom`) into a world polyline.

    Each tick it PROJECTS the live pose onto the polyline (monotonic forward
    progress), picks a carrot ``lookahead_m`` ahead along the arc, and commands a
    proportional body velocity toward it (:func:`_pursuit_velocity`). HEADING is
    an INDEPENDENT channel — hold the anchor heading, interpolate to a goal angle
    with progress, or face the path tangent — so a mecanum base can strafe along
    the curve without rotating to follow it. It finishes when the projection
    reaches the end of the polyline and the robot is within ``pos_tol_m`` of the
    final point.

    Requires ``robot.localization`` (the particle filter); raises at start
    otherwise.

    Args:
        waypoints: Body-frame control points ``[(forward_m, left_m), ...]`` in
            the run-start frame (relative to the single ``on_start`` anchor).
        speed: Velocity scale in ``(0.0, 1.0]`` applied to the pursuit command.
        heading_mode: ``"hold"`` (default — keep the anchor heading),
            ``"tangent"`` (face the local curve tangent), or a float goal angle
            in radians (interpolate from the anchor heading to it with progress).
        pos_tol_m: Final-point position tolerance (metres) for completion.
        lookahead_m: Pure-pursuit lookahead arc length (metres, default
            ``0.15``).
        spacing_m: Dense-sample spacing along the spline (metres, default
            ``0.03``).
    """

    def __init__(
        self,
        waypoints: list[tuple[float, float]],
        speed: float = 1.0,
        heading_mode: "str | float" = _HEADING_HOLD,
        pos_tol_m: float = 0.02,
        lookahead_m: float = _DEFAULT_LOOKAHEAD_M,
        spacing_m: float = 0.03,
    ) -> None:
        super().__init__()
        if not isinstance(speed, int | float):
            msg = f"speed must be a number, got {type(speed).__name__}"
            raise TypeError(msg)
        if not (0.0 < speed <= 1.0):
            msg = f"speed must be in (0.0, 1.0], got {speed}"
            raise ValueError(msg)
        if pos_tol_m <= 0.0:
            msg = f"pos_tol_m must be > 0, got {pos_tol_m}"
            raise ValueError(msg)
        if lookahead_m <= 0.0:
            msg = f"lookahead_m must be > 0, got {lookahead_m}"
            raise ValueError(msg)
        if spacing_m <= 0.0:
            msg = f"spacing_m must be > 0, got {spacing_m}"
            raise ValueError(msg)
        if len(waypoints) < 2:
            msg = "SplineFollow requires at least 2 control points"
            raise ValueError(msg)
        if isinstance(heading_mode, str) and heading_mode not in (
            _HEADING_HOLD,
            _HEADING_TANGENT,
        ):
            msg = (
                f"heading_mode str must be '{_HEADING_HOLD}' or "
                f"'{_HEADING_TANGENT}', got {heading_mode!r}"
            )
            raise ValueError(msg)
        if not isinstance(heading_mode, str) and not isinstance(heading_mode, int | float):
            msg = "heading_mode must be 'hold', 'tangent', or a goal angle (radians)"
            raise TypeError(msg)
        self._waypoints: list[tuple[float, float]] = [
            (float(fwd), float(left)) for (fwd, left) in waypoints
        ]
        self._speed = speed
        self._heading_mode = heading_mode
        self._pos_tol_m = pos_tol_m
        self._lookahead_m = lookahead_m
        self._spacing_m = spacing_m
        # Resolved at on_start once the anchor pose is known.
        self._path: list[tuple[float, float]] = []
        self._idx = 0
        self._anchor_heading = 0.0
        # Search window (in polyline indices) for the monotonic projection: a
        # couple of lookahead-distances' worth of samples ahead is plenty.
        self._window = max(4, int(math.ceil((2.0 * lookahead_m) / spacing_m)))

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive", "localization"})

    def _generate_signature(self) -> str:
        if isinstance(self._heading_mode, str):
            heading = self._heading_mode
        else:
            heading = f"{math.degrees(self._heading_mode):.1f}"
        return f"SplineFollow(n_ctrl={len(self._waypoints)}, heading={heading})"

    def on_start(self, robot: "GenericRobot") -> None:
        if getattr(robot, "localization", None) is None:
            msg = "spline_follow requires robot.localization (the particle filter)"
            raise RuntimeError(msg)
        anchor = robot.localization.get_pose()
        self._anchor_heading = float(anchor.heading)
        abs_points = [
            (x_m, y_m)
            for (x_m, y_m, _theta) in (
                _anchor_to_world_target(anchor, forward_m, left_m, 0.0)
                for (forward_m, left_m) in self._waypoints
            )
        ]
        self._path = sample_centripetal_catmull_rom(abs_points, self._spacing_m)
        self._idx = 0

    def _heading_target(self, current_heading: float) -> float:
        mode = self._heading_mode
        if mode == _HEADING_HOLD:
            return self._anchor_heading
        if mode == _HEADING_TANGENT:
            return _path_tangent_heading(self._path, self._idx)
        # Goal-angle interpolation: lerp anchor → goal by fractional progress.
        n = len(self._path)
        progress = (self._idx / (n - 1)) if n > 1 else 1.0
        progress = max(0.0, min(1.0, progress))
        delta = math.remainder(float(mode) - self._anchor_heading, 2.0 * math.pi)
        return self._anchor_heading + progress * delta

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        pose = robot.localization.get_pose()
        point = (float(pose.position[0]), float(pose.position[1]))

        # Monotonic projection onto the polyline (never runs backward).
        self._idx = _project_index(self._path, point, self._idx, self._window)

        # Completion: projection at the end AND within tolerance of the final pt.
        last = self._path[-1]
        dist_to_end = math.hypot(last[0] - point[0], last[1] - point[1])
        if self._idx >= len(self._path) - 2 and dist_to_end <= self._pos_tol_m:
            return True

        lookahead_pt = _lookahead_point(self._path, self._idx, self._lookahead_m)
        heading_target = self._heading_target(float(pose.heading))
        vx, vy, wz = _pursuit_velocity(pose, lookahead_pt, heading_target, self._speed)

        cfg = robot.motion_pid_config
        robot.drive.set_velocity(
            ChassisVelocity(
                _clamp(vx, cfg.linear.max_velocity),
                _clamp(vy, cfg.lateral.max_velocity),
                _clamp(wz, cfg.angular.max_velocity),
            )
        )
        robot.drive.update(dt)
        return False


@dsl(tags=["motion", "drive"])
def goto_relative(
    forward_cm: float,
    left_cm: float = 0.0,
    dtheta_deg: float | None = None,
    speed: float = 1.0,
    pos_tol_cm: float = 2.0,
    heading_tol_deg: float = 3.0,
) -> GotoRelative:
    """Drive to a pose given as a body-frame delta from the run-start pose.

    Closed-loop *navigate-to-pose by delta*: at start the step captures the
    live localization pose as an anchor, composes the body-frame displacement
    ``(forward_cm, left_cm)`` and optional heading change ``dtheta_deg`` onto
    it to obtain an absolute world target, then regulates onto that target with
    the IDENTICAL proportional controller as :func:`goto` (reading
    ``robot.localization.get_pose()`` each tick). Unlike the dead-reckoning
    ``drive_forward`` / ``strafe_*`` steps, it converges on the computed target
    pose regardless of accumulated odometry drift.

    The absolute target is not known at construction time — it equals
    ``anchor ⊕ delta`` and is fixed only once the anchor pose is sampled at
    ``on_start``. This is what lets the ``to_absolute`` optimizer pass convert a
    run of relative legs into closed-loop moves with no executor changes.

    Sign convention (matches :func:`goto` / the HAL ``ChassisVelocity``):
    ``+forward_cm`` is the anchor heading direction, ``+left_cm`` is 90° CCW
    (to the robot's left), ``+dtheta_deg`` is counter-clockwise.

    Prerequisites:
        - ``robot.localization`` (the particle filter) must be available;
          the step raises ``RuntimeError`` at start if it is missing.
        - A mecanum / omni-wheel drivetrain for full 2-D translation.

    Args:
        forward_cm: Body-forward displacement from the anchor, centimetres.
        left_cm: Body-left displacement from the anchor, centimetres
            (default ``0.0``).
        dtheta_deg: Heading change relative to the anchor heading, degrees,
            or ``None`` (default) to leave heading uncorrected.
        speed: Velocity scale in ``(0.0, 1.0]`` (default ``1.0``).
        pos_tol_cm: Position tolerance in centimetres (default ``2.0``).
        heading_tol_deg: Heading tolerance in degrees (default ``3.0``); only
            used when ``dtheta_deg`` is given.

    Returns:
        :class:`GotoRelative` — a ``MotionStep`` running the closed-loop
        controller against the anchor-relative target.

    Raises:
        RuntimeError: at start, if ``robot.localization`` is unavailable.

    Example::

        from raccoon.step.motion import goto_relative

        goto_relative(50)  # drive 0.5 m forward (closed-loop)
        goto_relative(50, dtheta_deg=90)  # 0.5 m forward, end 90° CCW
        goto_relative(30, left_cm=20)  # 0.3 m forward + 0.2 m left
    """
    return GotoRelative(
        forward_m=forward_cm / 100.0,
        left_m=left_cm / 100.0,
        dtheta_rad=None if dtheta_deg is None else math.radians(dtheta_deg),
        speed=speed,
        pos_tol_m=pos_tol_cm / 100.0,
        heading_tol_rad=math.radians(heading_tol_deg),
    )


@dsl(tags=["motion", "drive"])
def goto(
    x_cm: float,
    y_cm: float,
    theta_deg: float | None = None,
    speed: float = 1.0,
    pos_tol_cm: float = 2.0,
    heading_tol_deg: float = 3.0,
) -> Goto:
    """Drive to an ABSOLUTE world pose using localization as feedback.

    Closed-loop *navigate-to-pose* primitive. Each control tick the step reads
    the live pose estimate from the localization particle filter
    (``robot.localization.get_pose()``), computes the world-frame error to the
    target ``(x_cm, y_cm)``, rotates that translational error into the robot
    body frame using the current heading, and commands ``robot.drive`` a
    proportional velocity toward the goal. When ``theta_deg`` is given the step
    simultaneously rotates toward that absolute heading; otherwise heading is
    left uncorrected. The step finishes once the robot is within ``pos_tol_cm``
    of the target AND (within ``heading_tol_deg`` of ``theta_deg``, or no
    heading was requested).

    Because it regulates on the absolute particle-filter estimate rather than
    dead reckoning, ``goto`` converges on the target pose regardless of
    accumulated odometry drift — unlike the relative ``drive_forward`` /
    ``strafe_*`` steps. It commands lateral velocity, so it needs an
    omni-directional (mecanum / omni-wheel) drivetrain to translate freely;
    on a differential base only the forward and heading axes are effective.

    Prerequisites:
        - ``robot.localization`` (the particle filter) must be available;
          anchor it first (e.g. ``resync_at_start_pose()``). The step raises
          ``RuntimeError`` at start if localization is missing.
        - A mecanum / omni-wheel drivetrain for full 2-D translation.

    Args:
        x_cm: Target world X position, centimetres.
        y_cm: Target world Y position, centimetres.
        theta_deg: Target absolute heading in degrees, or ``None`` (default)
            to leave heading uncorrected.
        speed: Velocity scale in ``(0.0, 1.0]`` (default ``1.0``) applied to
            the proportional command before per-axis saturation.
        pos_tol_cm: Position tolerance in centimetres (default ``2.0``).
        heading_tol_deg: Heading tolerance in degrees (default ``3.0``); only
            used when ``theta_deg`` is given.

    Returns:
        :class:`Goto` — a ``MotionStep`` running the closed-loop controller.

    Raises:
        RuntimeError: at start, if ``robot.localization`` is unavailable.

    Example::

        from raccoon.step.motion import goto, resync_at_start_pose

        resync_at_start_pose(expected_x_cm=0, expected_y_cm=0, expected_theta_deg=0)
        goto(100, 50, theta_deg=90)  # drive to (1.0 m, 0.5 m), face 90°
        goto(30, 0)  # drive to (0.3 m, 0 m), hold heading
        goto(50, 50, speed=0.5, pos_tol_cm=1.0)
    """
    return Goto(
        x_m=x_cm / 100.0,
        y_m=y_cm / 100.0,
        theta_rad=None if theta_deg is None else math.radians(theta_deg),
        speed=speed,
        pos_tol_m=pos_tol_cm / 100.0,
        heading_tol_rad=math.radians(heading_tol_deg),
    )


__all__ = [
    "AbsoluteHoldMove",
    "Goto",
    "GotoRelative",
    "GotoWaypoints",
    "SplineFollow",
    "goto",
    "goto_relative",
]
