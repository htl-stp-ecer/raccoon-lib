"""Spline path motion step — smooth curved path through waypoints."""

import math
from typing import TYPE_CHECKING

from raccoon.motion import SplineMotion, SplineMotionConfig

from .. import Step, dsl
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
            raise ValueError("spline() requires at least 2 waypoints")

        first_len = len(waypoints[0])
        if first_len not in (2, 3):
            raise ValueError(
                f"Waypoints must be 2-tuples (fwd, left) or "
                f"3-tuples (fwd, left, heading_deg), got {first_len}-tuple"
            )
        for i, wp in enumerate(waypoints):
            if len(wp) != first_len:
                raise ValueError(
                    f"All waypoints must have the same length; "
                    f"waypoint 0 has {first_len} elements but waypoint {i} has {len(wp)}"
                )

        self._waypoints = waypoints
        self._speed = speed
        self._absolute_heading = absolute_heading
        self._final_heading_deg = final_heading_deg
        self._has_headings = first_len == 3
        self._motion: SplineMotion | None = None

    def _generate_signature(self) -> str:
        pts = ", ".join(
            f"({wp[0]:.1f}, {wp[1]:.1f})" if len(wp) == 2
            else f"({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}°)"
            for wp in self._waypoints
        )
        return f"SplinePath([{pts}], speed={self._speed:.2f})"

    def on_start(self, robot: "GenericRobot") -> None:
        if self._has_headings and not robot.drive.supports_lateral_motion():
            raise ValueError(
                "Explicit headings in spline() require an omni drivetrain "
                "(robot.drive.supports_lateral_motion() is False)"
            )

        config = SplineMotionConfig()

        # Convert cm → meters; negate lateral (left_cm → right_m for odometry convention)
        config.waypoints_m = [
            (wp[0] / 100.0, -wp[1] / 100.0)
            for wp in self._waypoints
        ]

        if self._has_headings:
            config.headings_rad = [
                math.radians(wp[2]) for wp in self._waypoints
            ]

        config.speed_scale = self._speed
        config.use_absolute_heading = self._absolute_heading

        if self._final_heading_deg is not None and not self._has_headings:
            if self._absolute_heading:
                # User gave an absolute heading (degrees, same convention as
                # drive_forward(heading=…)).  Convert to a relative offset so
                # the C++ addition of initial_absolute_heading_rad_ produces
                # the correct absolute target at the end of the path.
                from raccoon.robot.heading_reference import HeadingReferenceService
                ref_svc = robot.get_service(HeadingReferenceService)
                sign = 1.0 if ref_svc._positive_direction == "left" else -1.0
                abs_target_rad = ref_svc._reference_rad + sign * math.radians(self._final_heading_deg)
                current_abs_rad = robot.odometry.get_absolute_heading()
                config.final_heading_rad = abs_target_rad - current_abs_rad
            else:
                config.final_heading_rad = math.radians(self._final_heading_deg)

        self._motion = SplineMotion(
            robot.drive, robot.odometry, robot.motion_pid_config, config,
        )
        self._motion.start()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        self._motion.update(dt)
        return self._motion.is_finished()


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

    **Absolute heading mode** (``absolute_heading=True``): the heading PID
    computes its error against the IMU's absolute heading
    (``get_absolute_heading()``) which is never reset by odometry resets.
    The absolute heading at the start of the step is captured and used as
    an offset so that waypoint headings are still expressed relative to the
    robot's facing direction at the start — the convention is unchanged.
    Use this when the spline is chained after other motions that reset
    odometry and you need heading tracking to remain stable.

    **Final heading** (``final_heading``): the heading the robot should be
    facing when it arrives at the last waypoint, in degrees.  The robot
    smoothly rotates from its start heading to this target heading as it
    follows the path (linear interpolation by arc-length).  Ignored when
    3-tuple waypoints are used (per-waypoint headings take precedence).
    When ``absolute_heading=True``, ``final_heading`` is interpreted as an
    absolute heading relative to the heading reference (same convention as
    ``drive_forward(heading=…)``).  When ``absolute_heading=False``,
    ``final_heading`` is relative to the robot's start orientation
    (0 = keep facing forward, 90 = face 90° CCW at the end).

    Prerequisites:
        Drive characterization (``auto_tune`` or ``characterize_drive``)
        must have been run so that axis constraints are populated.
        ``mark_heading_reference()`` required when using
        ``absolute_heading=True`` with ``final_heading``.

    Args:
        *waypoints: Two or more waypoints as tuples. ``forward_cm`` is
            centimeters ahead of the start position (positive = forward).
            ``left_cm`` is centimeters to the left (positive = left).
            Optional ``heading_deg`` is the desired heading in degrees
            (0 = initial heading, positive = CCW).
        speed: Fraction of maximum speed, 0.0 to 1.0 (default 1.0).
        absolute_heading: If True, compute heading error against the
            absolute IMU heading instead of the reset-relative heading
            (default False).
        final_heading: Optional desired heading in degrees at the end of
            the path.  Relative to start orientation when
            ``absolute_heading=False``; absolute (heading-reference-based)
            when ``absolute_heading=True``.  Default None (no heading target).

    Returns:
        A SplinePath step configured for the described curved path.

    Raises:
        ValueError: If fewer than 2 waypoints or inconsistent tuple sizes.
        ValueError: If explicit headings (3-tuples) are used on a
            non-omni drivetrain.

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

        # After an odometry reset, keep heading stable against the IMU
        spline((30, 0), (50, 20), absolute_heading=True)

        # Absolute heading + arrive facing north (heading reference = north)
        spline((30, 0), (50, 20), absolute_heading=True, final_heading=0)
    """
    if len(waypoints) < 2:
        raise ValueError("spline() requires at least 2 waypoints")
    return SplinePath(list(waypoints), speed, absolute_heading, final_heading)
