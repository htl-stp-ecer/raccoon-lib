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

        self._motion = SplineMotion(
            robot.drive, robot.odometry, robot.motion_pid_config, config,
        )
        self._motion.start()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        self._motion.update(dt)
        return self._motion.is_finished()


@dsl(tags=["motion", "drive"])
def spline(*waypoints: tuple[float, ...], speed: float = 1.0) -> SplinePath:
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

    Prerequisites:
        Drive characterization (``auto_tune`` or ``characterize_drive``)
        must have been run so that axis constraints are populated.

    Args:
        *waypoints: Two or more waypoints as tuples. ``forward_cm`` is
            centimeters ahead of the start position (positive = forward).
            ``left_cm`` is centimeters to the left (positive = left).
            Optional ``heading_deg`` is the desired heading in degrees
            (0 = initial heading, positive = CCW).
        speed: Fraction of maximum speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A SplinePath step configured for the described curved path.

    Raises:
        ValueError: If fewer than 2 waypoints, inconsistent tuple sizes,
            or explicit headings on a non-omni drivetrain.

    Example::

        from raccoon.step.motion import spline

        # S-curve around an obstacle (heading follows tangent)
        spline((30, 0), (50, 15), (50, 30), (30, 30))

        # Gentle curve at half speed
        spline((40, 0), (60, 20), speed=0.5)

        # Omni: curve while rotating to face 90° left at the end
        spline((30, 0, 0), (50, 15, 45), (50, 30, 90))
    """
    if len(waypoints) < 2:
        raise ValueError("spline() requires at least 2 waypoints")
    return SplinePath(list(waypoints), speed)
