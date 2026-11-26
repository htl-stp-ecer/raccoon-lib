"""Spline path motion step — smooth curved path through waypoints."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from raccoon.motion import SplineMotion, SplineMotionConfig

from .. import dsl
from ._heading_utils import get_world_heading_rad
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl(hidden=True)
class SplinePath(MotionStep):
    """Drive along a centripetal Catmull-Rom spline through waypoints.

    Supports two waypoint formats:
    - 2-tuples (forward_cm, left_cm): heading follows spline tangent.
    - 3-tuples (forward_cm, left_cm, heading_deg): explicit heading at each
      waypoint, linearly interpolated vs arc-length. Omni drivetrains only.
    """

    def __init__(
        self,
        waypoints: list[tuple[float, ...]],
        speed: float,
        absolute_heading: bool = False,
        final_heading_deg: float | None = None,
    ) -> None:
        super().__init__()
        if len(waypoints) < 2:
            msg = "spline() requires at least 2 waypoints"
            raise ValueError(msg)

        first_len = len(waypoints[0])
        if first_len not in (2, 3):
            msg = (
                f"Waypoints must be 2-tuples (fwd, left) or "
                f"3-tuples (fwd, left, heading_deg), got {first_len}-tuple"
            )
            raise ValueError(msg)
        for i, wp in enumerate(waypoints):
            if len(wp) != first_len:
                msg = (
                    f"All waypoints must have the same length; "
                    f"waypoint 0 has {first_len} elements but waypoint {i} has {len(wp)}"
                )
                raise ValueError(msg)

        self._waypoints = waypoints
        self._speed = speed
        self._absolute_heading = absolute_heading
        self._final_heading_deg = final_heading_deg
        self._has_headings = first_len == 3
        self._motion: SplineMotion | None = None

    def _generate_signature(self) -> str:
        pts = ", ".join(
            f"({wp[0]:.1f}, {wp[1]:.1f})"
            if len(wp) == 2
            else f"({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}°)"
            for wp in self._waypoints
        )
        return f"SplinePath([{pts}], speed={self._speed:.2f})"

    def on_start(self, robot: "GenericRobot") -> None:
        if self._has_headings and not robot.drive.supports_lateral_motion():
            msg = (
                "Explicit headings in spline() require an omni drivetrain "
                "(robot.drive.supports_lateral_motion() is False)"
            )
            raise ValueError(msg)

        uses_final_heading = self._final_heading_deg is not None and not self._has_headings
        if self._has_headings and self._final_heading_deg is not None:
            self.warn(
                "spline(): final_heading has no effect with 3-tuple waypoints; "
                "the heading of the last waypoint applies"
            )
        if self._absolute_heading and not (self._has_headings or uses_final_heading):
            self.warn(
                "spline(): absolute_heading=True has no effect without "
                "3-tuple waypoints or final_heading"
            )

        config = SplineMotionConfig()

        # Convert cm → meters; negate lateral (left_cm → right_m for odometry convention)
        config.waypoints_m = [(wp[0] / 100.0, -wp[1] / 100.0) for wp in self._waypoints]

        if self._has_headings:
            config.headings_rad = [
                self._to_start_relative_rad(robot, wp[2]) for wp in self._waypoints
            ]

        config.speed_scale = self._speed

        if uses_final_heading:
            config.final_heading_rad = self._to_start_relative_rad(robot, self._final_heading_deg)

        self._motion = SplineMotion(
            robot.drive,
            robot.odometry,
            robot.motion_pid_config,
            config,
        )
        self._motion.start()

    def _to_start_relative_rad(self, robot: "GenericRobot", heading_deg: float) -> float:
        """Convert a heading argument to what SplineMotion expects.

        SplineMotion interprets headings relative to the heading at start. With
        ``absolute_heading`` the argument is measured from the heading reference.

        Raises:
            RuntimeError: If ``absolute_heading`` is set and no heading
                reference has been marked.
        """
        if not self._absolute_heading:
            return math.radians(heading_deg)

        from raccoon.robot.heading_reference import HeadingReferenceService

        target_rad = robot.get_service(HeadingReferenceService).target_absolute_rad(heading_deg)
        return target_rad - get_world_heading_rad(robot)

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        self._motion.update(dt)
        return self._motion.is_finished()

    def lower_to_segments(self) -> "list":
        from .path.ir import Segment

        return [Segment(kind="spline", has_known_endpoint=True, opaque_step=self)]


@dsl(tags=["motion", "drive"])
def spline(
    *waypoints: tuple[float, ...],
    speed: float = 1.0,
    absolute_heading: bool = False,
    final_heading: float | None = None,
) -> SplinePath:
    """Drive along a smooth curved path through a series of waypoints.

    The robot follows a centripetal Catmull-Rom spline that passes through
    every waypoint with continuous velocity. A trapezoidal velocity profile
    on arc-length provides smooth acceleration and deceleration along the
    path, while a heading PID keeps the robot oriented along the curve.

    Waypoints are specified relative to the robot's pose when the step
    starts. Two formats are supported:

    - **2-tuples** ``(forward_cm, left_cm)``: heading automatically follows
      the spline tangent. Works on both differential and omni drivetrains.
    - **3-tuples** ``(forward_cm, left_cm, heading_deg)``: explicit heading
      at each waypoint, linearly interpolated along the path. Omni
      drivetrains only — raises ``ValueError`` on differential bots.

    **Final heading** (``final_heading``): the heading the robot should be
    facing when it arrives at the last waypoint, in degrees.  The robot
    smoothly rotates from its start heading to this target heading as it
    follows the path (linear interpolation by arc-length).  It has no effect
    with 3-tuple waypoints, which carry their own headings (logs a warning).

    **Absolute headings** (``absolute_heading=True``): every heading argument,
    meaning the headings of 3-tuple waypoints and ``final_heading``, is
    measured from the heading reference set by ``mark_heading_reference()``
    (same convention as ``drive_forward(heading=…)``, including its
    ``positive_direction``).  With ``absolute_heading=False`` they are relative
    to the robot's start orientation (0 = keep facing forward, 90 = face 90°
    CCW).  Waypoint positions are relative to the start pose in both cases.
    Without any heading argument (2-tuples and no ``final_heading``) the robot
    follows the path tangent, ``absolute_heading`` has no effect and a warning
    is logged.

    Prerequisites:
        Drive characterization (``auto_tune`` or ``characterize_drive``)
        must have been run so that axis constraints are populated.
        ``mark_heading_reference()`` required when ``absolute_heading=True``
        is combined with 3-tuple waypoints or ``final_heading``.

    Args:
        *waypoints: Two or more waypoints as tuples. ``forward_cm`` is
            centimeters ahead of the start position (positive = forward).
            ``left_cm`` is centimeters to the left (positive = left).
            Optional ``heading_deg`` is the desired heading in degrees,
            relative to the start orientation (positive = CCW) or, with
            ``absolute_heading=True``, measured from the heading reference.
        speed: Fraction of maximum speed (default 1.0). The sign selects the
            travel direction: positive drives the path forwards. Negative drives
            it in reverse — on a mecanum/omni drivetrain the robot HOLDS its
            start orientation and strafes the path backwards (nose stays put,
            no turn); on a differential drivetrain, which cannot hold heading
            through a curve, it backs along the path rear-first (nose points
            opposite the travel). Magnitude is clamped to [0.01, 1.0].
        absolute_heading: If True, waypoint headings and ``final_heading``
            are measured from the heading reference instead of the start
            orientation (default False).
        final_heading: Optional desired heading in degrees at the end of
            the path, in the frame selected by ``absolute_heading``. Default
            None (no heading target).

    Returns:
        A SplinePath step configured for the described curved path.

    Raises:
        ValueError: If fewer than 2 waypoints or inconsistent tuple sizes.
        ValueError: If explicit headings (3-tuples) are used on a
            non-omni drivetrain.
        RuntimeError: When the step starts, if ``absolute_heading=True``
            applies to a heading argument and no heading reference is marked.

    Example::

        from raccoon.step.motion import spline

        # S-curve around an obstacle (heading follows tangent)
        spline((30, 0), (50, 15), (50, 30), (30, 30))

        # Gentle curve at half speed
        spline((40, 0), (60, 20), speed=0.5)

        # Arrive facing 90° left of start orientation
        spline((30, 0), (50, 20), final_heading=90)

        # Omni: curve while rotating to face 90° left at the end
        spline((30, 0, 0), (50, 15, 45), (50, 30, 90))

        # Absolute heading + arrive facing north (heading reference = north)
        spline((30, 0), (50, 20), absolute_heading=True, final_heading=0)

        # Omni: turn to the heading reference by the first waypoint, then hold it
        spline((30, 0, 0), (50, 15, 0), (50, 30, 0), absolute_heading=True)
    """
    if len(waypoints) < 2:
        msg = "spline() requires at least 2 waypoints"
        raise ValueError(msg)
    return SplinePath(list(waypoints), speed, absolute_heading, final_heading)
