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
        if self._step._motion is not None:
            self._step._motion.start_warm(offset, velocity)
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
        if self._step._motion is not None:
            return self._step._motion.get_filtered_velocity()
        return 0.0

    def set_suppress_hard_stop(self, val: bool) -> None:
        if self._step._motion is not None:
            self._step._motion.set_suppress_hard_stop(val)


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
        if self._step._motion is not None:
            return self._step._motion.get_filtered_velocity()
        return 0.0

    def set_suppress_hard_stop(self, val: bool) -> None:
        if self._step._motion is not None:
            self._step._motion.set_suppress_hard_stop(val)


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
) -> LinearMotion:
    config = LinearMotionConfig()
    config.axis = seg.axis
    actual = seg.distance_m if seg.distance_m is not None else seg.sign * SENTINEL_DISTANCE_M

    if not is_last and seg.distance_m is not None:
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
    if not is_last:
        motion.set_suppress_hard_stop(True)
    return motion


def _create_turn_motion(
    robot: "GenericRobot",
    seg: Segment,
    is_last: bool,
    current_world_heading_rad: float | None = None,
) -> TurnMotion:
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

    if not is_last:
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
    if not is_last:
        motion.set_suppress_hard_stop(True)
    return motion


def _create_arc_motion(
    robot: "GenericRobot",
    seg: Segment,
    is_last: bool,
    current_world_heading_rad: float | None = None,  # noqa: ARG001 — ArcMotion is delta-based
) -> ArcMotion:
    config = ArcMotionConfig()
    config.radius_m = seg.radius_m
    actual = seg.arc_angle_rad

    if not is_last and actual is not None:
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
    if not is_last:
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
):
    """Construct a controller for the given segment kind.

    ``current_world_heading_rad`` is the absolute world heading at the
    moment the executor is about to start this segment, read from
    ``robot.localization.get_pose().heading``. Used by ``linear`` segments
    that have no user-specified ``heading_deg`` and by absolute
    ``turn`` segments bridged from the Phase-5 plan IR. Ignored by
    ``arc`` and by the opaque ``follow_line`` / ``spline`` adapters
    (those steps fetch the heading themselves).
    """
    if seg.kind == "linear":
        return _create_linear_motion(robot, seg, is_last, current_world_heading_rad)
    if seg.kind == "turn":
        return _create_turn_motion(robot, seg, is_last, current_world_heading_rad)
    if seg.kind == "arc":
        return _create_arc_motion(robot, seg, is_last, current_world_heading_rad)
    if seg.kind == "follow_line":
        return LineFollowAdapter(seg.opaque_step, robot)
    if seg.kind == "spline":
        return SplineAdapter(seg.opaque_step, robot)
    if seg.kind == "diagonal":
        return DriveAngleAdapter(seg.opaque_step, robot)
    msg = f"Unknown segment kind: {seg.kind}"
    raise ValueError(msg)
