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

from .. import dsl
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


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


__all__ = ["Goto", "GotoRelative", "goto", "goto_relative"]
