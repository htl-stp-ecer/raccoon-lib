"""Motion factory — constructs a controller for one ``Segment``.

The executor calls ``create_motion(robot, seg, is_last)`` and gets back an
object with the uniform motion-controller interface:

    motion.start()
    motion.start_warm(offset, velocity)
    motion.update(dt)
    motion.is_finished()
    motion.has_reached_distance() / .has_reached_angle()
    motion.get_filtered_velocity()
    motion.set_suppress_hard_stop(bool)

For ``linear`` / ``turn`` / ``arc`` kinds, this is the corresponding C++
``LinearMotion`` / ``TurnMotion`` / ``ArcMotion`` class directly.

For ``follow_line`` / ``spline`` / ``diagonal`` kinds, the Python step is
wrapped in an adapter that translates the ``MotionStep`` lifecycle
(``on_start``, ``on_update(robot, dt)``) to the uniform interface.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from raccoon.motion import (
    ArcMotion,
    ArcMotionConfig,
    LinearMotion,
    LinearMotionConfig,
    TurnConfig,
    TurnMotion,
)

from .._motion_trim import MotionTrimService
from .ir import SENTINEL_DISTANCE_M, Segment

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# ---------------------------------------------------------------------------
# Profile inflation for non-terminal segments
# ---------------------------------------------------------------------------
# The trapezoidal profile naturally decelerates to zero at its target.  For
# intermediate segments we want the robot still cruising at full speed when
# the actual endpoint is reached, so we inflate the target.  The executor
# checks the actual position manually and triggers the transition while the
# profile is still in cruise phase.

OVERSHOOT_M = 1.0  # 1m overshoot (decel zone ~0.2m at typical params)
OVERSHOOT_RAD = 3.0  # ~172° overshoot (decel zone ~1.5rad at typical params)


# ---------------------------------------------------------------------------
# Opaque motion adapters (follow_line, spline)
# ---------------------------------------------------------------------------


class LineFollowAdapter:
    """Adapts a LineFollow/SingleSensorLineFollow step to the C++ motion API.

    Delegates to the step's ``on_start``/``on_update`` lifecycle and forwards
    completion queries to the internal ``LinearMotion``.  Supports warm-start
    for seamless velocity hand-off from a preceding forward linear segment.
    """

    def __init__(self, step, robot: "GenericRobot") -> None:
        self._step = step
        self._robot = robot
        self._done = False

    def start(self) -> None:
        self._step.on_start(self._robot)
        self._done = False

    def start_warm(self, offset: float, velocity: float) -> None:
        self._step.on_start(self._robot)
        motion = self._step._motion
        # The unified C++ line-follow motion (DirectionalLineFollowMotion) does
        # not expose start_warm — on_start above already cold-initialized it, so
        # degrade gracefully to a cold start (correct, matches plain seq()).
        if motion is not None and hasattr(motion, "start_warm"):
            motion.start_warm(offset, velocity)
        self._done = False

    def update(self, dt: float) -> None:
        result = self._step.on_update(self._robot, dt)
        if result:
            self._done = True

    def is_finished(self) -> bool:
        return self._done

    def has_reached_distance(self) -> bool:
        if self._step._motion is not None:
            return self._step._motion.has_reached_distance()
        return self._done

    def get_filtered_velocity(self) -> float:
        motion = self._step._motion
        if motion is not None and hasattr(motion, "get_filtered_velocity"):
            return motion.get_filtered_velocity()
        return 0.0

    def set_suppress_hard_stop(self, val: bool) -> None:
        if self._step._motion is not None:
            self._step._motion.set_suppress_hard_stop(val)


class TurnHeadingAdapter:
    """Adapts a :class:`TurnToHeading` step to the C++ motion API.

    Delegates to the step's ``on_start``/``on_update`` lifecycle. The step
    resolves its absolute target heading (via the HeadingReferenceService) and
    builds its own ``TurnMotion`` at ``on_start``, so this adapter only tracks
    completion. Turns never warm-start, so ``start_warm`` is a cold start.
    """

    def __init__(self, step, robot: "GenericRobot") -> None:
        self._step = step
        self._robot = robot
        self._done = False

    def start(self) -> None:
        self._step.on_start(self._robot)
        self._done = False

    def start_warm(self, offset: float, velocity: float) -> None:
        # Turns don't warm-start; treat like a cold start.
        self._step.on_start(self._robot)
        self._done = False

    def update(self, dt: float) -> None:
        result = self._step.on_update(self._robot, dt)
        if result:
            self._done = True

    def is_finished(self) -> bool:
        return self._done

    def has_reached_angle(self) -> bool:
        return self._done

    def get_filtered_velocity(self) -> float:
        return 0.0

    def set_suppress_hard_stop(self, val: bool) -> None:
        motion = getattr(self._step, "_motion", None)
        if motion is not None:
            motion.set_suppress_hard_stop(val)


class DriveAngleAdapter:
    """Adapts a DriveAngle (true diagonal) step to the C++ motion API.

    Delegates to the step's ``on_start``/``on_update`` lifecycle. The
    underlying ``DiagonalMotion`` has no guaranteed warm-start, so every
    transition into a diagonal uses a cold start.
    """

    def __init__(self, step, robot: "GenericRobot") -> None:
        self._step = step
        self._robot = robot
        self._done = False

    def start(self) -> None:
        self._step.on_start(self._robot)
        self._done = False

    def start_warm(self, offset: float, velocity: float) -> None:
        # DiagonalMotion warm-start isn't guaranteed; treat like cold start.
        self._step.on_start(self._robot)
        self._done = False

    def update(self, dt: float) -> None:
        result = self._step.on_update(self._robot, dt)
        if result:
            self._done = True

    def is_finished(self) -> bool:
        return self._done

    def has_reached_distance(self) -> bool:
        return self._done

    def get_filtered_velocity(self) -> float:
        motion = self._step._motion
        # DiagonalMotion does not expose get_filtered_velocity; the next leg
        # cold-starts anyway (diagonal is never same-type), so 0.0 is correct.
        if motion is not None and hasattr(motion, "get_filtered_velocity"):
            return motion.get_filtered_velocity()
        return 0.0

    def set_suppress_hard_stop(self, val: bool) -> None:
        if self._step._motion is not None:
            self._step._motion.set_suppress_hard_stop(val)


class CrabArcAdapter:
    """Drives a constant-heading 90° corner blend (``crab_arc``).

    A holonomic base can round a ``forward↔strafe`` corner without rotating: the
    body-frame velocity vector is swept from the entry leg's travel direction
    (``crab_from``) to the exit leg's (``crab_to``) along a quarter circle of
    radius ``R = seg.radius_m``, while a proportional controller holds the
    heading captured at start. Progress is the path length travelled (read from
    odometry); the arc angle is ``α = travelled / R`` and the blend completes at
    ``α = arc_angle_rad`` (π/2).

    No C++ motion backs this — like ``DriveAngle`` it commands chassis velocity
    directly. It never relies on a trapezoidal profile, so a cold ``start`` and a
    profiled ``start_warm`` both simply begin commanding the blend at cruise (or
    the carried) speed; ``inflate`` is irrelevant.
    """

    # Proportional heading-hold gain (rad/s per rad of error), clamped to the
    # drivetrain's angular cap. Gentle: the base barely yaws during a crab blend.
    _HEADING_KP = 4.0

    def __init__(self, seg: Segment, robot: "GenericRobot") -> None:
        self._seg = seg
        self._robot = robot
        self._e1 = seg.crab_from or (1.0, 0.0)
        self._e2 = seg.crab_to or (0.0, 1.0)
        self._radius = seg.radius_m or 0.0
        self._arc = abs(seg.arc_angle_rad if seg.arc_angle_rad is not None else math.pi / 2.0)
        self._done = False
        self._traveled = 0.0
        self._prev_xy: tuple[float, float] | None = None
        self._target_heading = 0.0
        self._speed = 0.0

    def _cruise_speed(self) -> float:
        cfg = self._robot.motion_pid_config
        # The blend reaches full vx at α=0 and full vy at α=π/2, so cap the
        # magnitude by the SLOWER of the two axes to respect both limits.
        base = min(cfg.linear.max_velocity, cfg.lateral.max_velocity)
        return base * (self._seg.speed_scale or 1.0)

    def _begin(self, speed: float) -> None:
        from .._heading_utils import get_world_heading_rad

        self._target_heading = get_world_heading_rad(self._robot)
        self._speed = max(0.0, min(speed, self._cruise_speed()))
        self._done = self._radius <= 1e-9
        self._traveled = 0.0
        pos = self._robot.odometry.get_pose().position
        self._prev_xy = (float(pos[0]), float(pos[1]))

    def start(self) -> None:
        self._begin(self._cruise_speed())

    def start_warm(self, offset: float, velocity: float) -> None:  # offset unused
        # ``velocity`` is the linear m/s carried from the previous leg (the
        # executor does NOT divide by radius for non-"arc" kinds). Use it so the
        # robot flows into the corner instead of restarting from rest.
        self._begin(velocity if velocity > 1e-3 else self._cruise_speed())

    def update(self, dt: float) -> None:
        from raccoon.foundation import ChassisVelocity

        if self._done:
            self._robot.drive.set_velocity(ChassisVelocity(0.0, 0.0, 0.0))
            self._robot.drive.update(dt)
            return

        self._robot.odometry.update(dt)
        pos = self._robot.odometry.get_pose().position
        x, y = float(pos[0]), float(pos[1])
        if self._prev_xy is not None:
            self._traveled += math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
        self._prev_xy = (x, y)

        alpha = self._traveled / self._radius if self._radius > 1e-9 else self._arc
        if alpha >= self._arc:
            self._done = True
            self._robot.drive.set_velocity(ChassisVelocity(0.0, 0.0, 0.0))
            self._robot.drive.update(dt)
            return

        # Body-frame velocity direction: rotate from crab_from to crab_to.
        c, s = math.cos(alpha), math.sin(alpha)
        dx = c * self._e1[0] + s * self._e2[0]
        dy = c * self._e1[1] + s * self._e2[1]

        # Hold the heading captured at start (P control, clamped to angular cap).
        heading = float(self._robot.odometry.get_heading())
        err = math.remainder(self._target_heading - heading, 2.0 * math.pi)
        omega_cap = self._robot.motion_pid_config.angular.max_velocity
        wz = max(-omega_cap, min(omega_cap, self._HEADING_KP * err))

        self._robot.drive.set_velocity(ChassisVelocity(self._speed * dx, self._speed * dy, wz))
        self._robot.drive.update(dt)

    def is_finished(self) -> bool:
        return self._done

    def has_reached_distance(self) -> bool:
        return self._done

    def get_filtered_velocity(self) -> float:
        return self._speed

    def set_suppress_hard_stop(self, val: bool) -> None:
        pass  # the adapter commands its own velocity; nothing to suppress


class SplineAdapter:
    """Adapts a SplinePath step to the C++ motion API.

    SplineMotion does not support warm-start; cross-type transitions to/from
    spline segments always use a cold start.
    """

    def __init__(self, step, robot: "GenericRobot") -> None:
        self._step = step
        self._robot = robot

    def start(self) -> None:
        self._step.on_start(self._robot)

    def start_warm(self, offset: float, velocity: float) -> None:
        # SplineMotion has no warm-start; treat like cold start.
        self._step.on_start(self._robot)

    def update(self, dt: float) -> None:
        self._step.on_update(self._robot, dt)

    def is_finished(self) -> bool:
        return self._step._motion is not None and self._step._motion.is_finished()

    def has_reached_distance(self) -> bool:
        return self.is_finished()

    def get_filtered_velocity(self) -> float:
        return 0.0  # exit velocity from a spline is treated as zero

    def set_suppress_hard_stop(self, val: bool) -> None:
        pass  # SplineMotion handles its own stop behaviour


# ---------------------------------------------------------------------------
# Per-kind motion constructors
# ---------------------------------------------------------------------------


def _create_linear_motion(
    robot: "GenericRobot",
    seg: Segment,
    is_last: bool,
    current_world_heading_rad: float | None = None,
    inflate: bool | None = None,
) -> LinearMotion:
    # Inflate (cruise through the endpoint) ONLY when the next segment will
    # warm-start from this one. When the next leg cold-starts (cross-type, a
    # blocking side action between, or this is the last segment) the profile
    # must DECELERATE to the real target instead — otherwise a short move runs
    # through its endpoint at cruise speed and overshoots (e.g. a 3 cm grab
    # drive overshooting ~8 cm). Defaults to ``not is_last`` for back-compat.
    do_inflate = (not is_last) if inflate is None else inflate
    config = LinearMotionConfig()
    config.axis = seg.axis
    actual = seg.distance_m if seg.distance_m is not None else seg.sign * SENTINEL_DISTANCE_M

    if do_inflate and seg.distance_m is not None:
        # Inflate target so profile cruises through the actual endpoint.
        config.distance_m = actual + math.copysign(OVERSHOOT_M, actual)
    else:
        config.distance_m = actual
    if seg.distance_m is not None:
        trim_svc = robot.get_service(MotionTrimService)
        axis_name = "forward" if seg.axis.name.lower() == "forward" else "lateral"
        config.distance_m = trim_svc.scale_distance_m(axis_name, config.distance_m)
    # Path segments with a real `distance_m` are positional goals; ones
    # backed by the sentinel are condition-driven. Flag accordingly so
    # the motion can still run while SpeedMode is on.
    config.has_distance_target = seg.distance_m is not None
    config.speed_scale = seg.speed_scale

    # Heading: phase 4 made target_heading_rad mandatory. Priority is
    #   1. user-specified seg.heading_deg (relative to HeadingReference),
    #   2. absolute target heading from the Phase-5 absolute-plan bridge,
    #   3. current world heading from localization (passed in by executor).
    if seg.heading_deg is not None:
        from raccoon.robot.heading_reference import HeadingReferenceService

        ref_svc = robot.get_service(HeadingReferenceService)
        sign = 1.0 if ref_svc._positive_direction == "left" else -1.0
        config.target_heading_rad = ref_svc._reference_rad + sign * math.radians(seg.heading_deg)
    elif seg.target_heading_rad is not None:
        config.target_heading_rad = seg.target_heading_rad
    elif current_world_heading_rad is not None:
        config.target_heading_rad = current_world_heading_rad
    else:
        # Defensive: callers in this repo always pass the heading in. If a
        # downstream caller forgets, fail loudly instead of silently
        # holding 0 rad.
        msg = (
            "_create_linear_motion: no target_heading_rad source. "
            "Pass current_world_heading_rad or set seg.heading_deg."
        )
        raise RuntimeError(msg)

    motion = LinearMotion(
        robot.drive,
        robot.odometry,
        robot.motion_pid_config,
        config,
    )
    if do_inflate:
        motion.set_suppress_hard_stop(True)
    return motion


def _create_turn_motion(
    robot: "GenericRobot",
    seg: Segment,
    is_last: bool,
    current_world_heading_rad: float | None = None,
    inflate: bool | None = None,
) -> TurnMotion:
    do_inflate = (not is_last) if inflate is None else inflate
    config = TurnConfig()
    if seg.target_heading_rad is not None:
        if current_world_heading_rad is None:
            msg = (
                "_create_turn_motion: absolute turn requires "
                "current_world_heading_rad from localization."
            )
            raise RuntimeError(msg)
        actual = math.remainder(seg.target_heading_rad - current_world_heading_rad, 2.0 * math.pi)
    else:
        actual = (
            seg.angle_rad
            if seg.angle_rad is not None
            else seg.sign * math.radians(180)  # sentinel for condition-based
        )

    if do_inflate:
        config.target_angle_rad = actual + math.copysign(OVERSHOOT_RAD, actual)
    else:
        config.target_angle_rad = actual
    # If the segment encodes an explicit angle (or an absolute target
    # heading) it is a real angular goal; the sentinel branch is the
    # condition-driven case which has no measurable end-state and must
    # therefore be tolerant of SpeedMode.
    config.has_angle_target = (seg.angle_rad is not None) or (seg.target_heading_rad is not None)
    config.speed_scale = seg.speed_scale

    motion = TurnMotion(
        robot.drive,
        robot.odometry,
        robot.motion_pid_config,
        config,
    )
    if do_inflate:
        motion.set_suppress_hard_stop(True)
    return motion


def _create_arc_motion(
    robot: "GenericRobot",
    seg: Segment,
    is_last: bool,
    current_world_heading_rad: float | None = None,  # noqa: ARG001 — ArcMotion is delta-based
    inflate: bool | None = None,
) -> ArcMotion:
    do_inflate = (not is_last) if inflate is None else inflate
    config = ArcMotionConfig()
    config.radius_m = seg.radius_m
    # IR convention: ``seg.arc_angle_rad`` is the TRUE heading delta and a
    # negative ``speed_scale`` means reverse travel. ArcMotion's reverse mode
    # is a forward arc mirrored in time — it NEGATES the executed heading
    # change relative to ``config.arc_angle_rad`` — so flip the angle here to
    # keep the commanded heading delta equal to the IR's.
    actual = seg.arc_angle_rad
    if actual is not None and seg.speed_scale < 0:
        actual = -actual

    if do_inflate and actual is not None:
        config.arc_angle_rad = actual + math.copysign(OVERSHOOT_RAD, actual)
    else:
        config.arc_angle_rad = actual
    config.speed_scale = seg.speed_scale
    config.lateral = seg.lateral

    motion = ArcMotion(
        robot.drive,
        robot.odometry,
        robot.motion_pid_config,
        config,
    )
    if do_inflate:
        motion.set_suppress_hard_stop(True)
    return motion


# ---------------------------------------------------------------------------
# Public dispatch
# ---------------------------------------------------------------------------


def create_motion(
    robot: "GenericRobot",
    seg: Segment,
    is_last: bool,
    current_world_heading_rad: float | None = None,
    inflate: bool | None = None,
):
    """Construct a controller for the given segment kind.

    ``current_world_heading_rad`` is the absolute world heading at the
    moment the executor is about to start this segment, read from
    ``robot.localization.get_pose().heading``. Used by ``linear`` segments
    that have no user-specified ``heading_deg`` and by absolute
    ``turn`` segments bridged from the Phase-5 plan IR. Ignored by
    ``arc`` and by the opaque ``follow_line`` / ``spline`` adapters
    (those steps fetch the heading themselves).

    ``inflate`` controls profile-overshoot inflation: ``True`` cruises through
    the endpoint (only correct when the next segment warm-starts from this
    one), ``False`` decelerates to the real target (cross-type / blocked /
    last). Defaults (``None``) to ``not is_last`` for back-compat. The executor
    passes the look-ahead result so short moves before a direction change stop
    on target instead of overshooting.
    """
    if seg.kind == "linear":
        return _create_linear_motion(robot, seg, is_last, current_world_heading_rad, inflate)
    if seg.kind == "turn":
        if seg.opaque_step is not None:
            # Opaque heading turn (TurnToHeading): the step resolves its own
            # absolute target heading and TurnMotion at on_start.
            return TurnHeadingAdapter(seg.opaque_step, robot)
        return _create_turn_motion(robot, seg, is_last, current_world_heading_rad, inflate)
    if seg.kind == "arc":
        return _create_arc_motion(robot, seg, is_last, current_world_heading_rad, inflate)
    if seg.kind == "crab_arc":
        # Constant-heading corner blend (holonomic forward↔strafe). The adapter
        # reads the held heading itself, so current_world_heading_rad is unused.
        return CrabArcAdapter(seg, robot)
    if seg.kind == "follow_line":
        return LineFollowAdapter(seg.opaque_step, robot)
    if seg.kind == "spline":
        return SplineAdapter(seg.opaque_step, robot)
    if seg.kind == "diagonal":
        return DriveAngleAdapter(seg.opaque_step, robot)
    msg = f"Unknown segment kind: {seg.kind}"
    raise ValueError(msg)
