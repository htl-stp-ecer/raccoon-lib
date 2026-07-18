from __future__ import annotations

import math
from typing import TYPE_CHECKING

from raccoon.motion import DiagonalMotion, DiagonalMotionConfig

from ..annotation import dsl_step
from ..condition import StopCondition
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl_step(tags=["motion", "drive"])
class DriveAngle(MotionStep):
    """Drive at an arbitrary angle with distance or condition-based termination.

    Decomposes the desired heading into forward and lateral velocity
    components, then runs a profiled PID controller in a rotated
    coordinate frame with heading maintenance and cross-track correction.

    Requires a mecanum or omni-wheel drivetrain.

    Angle convention (robot-centric): ``0`` = forward, ``90`` = right,
    ``-90`` = left, ``180`` = backward.

    The ``angle_deg`` travel direction is always robot-centric. The held
    heading, however, is decoupled: when ``heading`` is given the controller
    holds that absolute heading (degrees from the heading reference) via
    ``HeadingReferenceService`` \u2014 the same frame ``turn_to_heading`` and the
    ``heading=`` argument of ``drive_forward``/``strafe_*`` use \u2014 instead of
    the heading captured at start. This prevents drift accumulation across
    consecutive diagonal drives.

    Requires ``mark_heading_reference()`` when using ``heading``.

    Args:
        angle_deg: Travel angle in degrees.
        cm: Distance to travel in centimeters. Omit to use condition-only mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination.
        heading: Absolute heading in degrees from the heading reference to
            hold during the drive. ``None`` (default) holds the heading at the
            start of the drive (relative mode).

    Example::

        from raccoon.step.motion import drive_angle

        # Drive diagonally forward-right at 45 degrees
        drive_angle(45, cm=30)

        # Drive pure right (same as strafe_right)
        drive_angle(90, cm=20)

        # Push diagonally while holding an absolute 0\u00b0 heading
        drive_angle(-120, heading=0).until(on_black(s))
    """

    _SENTINEL_DISTANCE_M = 100.0

    def __init__(
        self,
        angle_deg: float,
        cm: float | None = None,
        speed: float = 1.0,
        until: StopCondition = None,
        heading: float | None = None,
    ) -> None:
        super().__init__()
        if cm is None and until is None:
            msg = "DriveAngle requires either 'cm' or 'until'"
            raise ValueError(msg)
        if heading is not None and not isinstance(heading, int | float):
            msg = f"heading must be a number, got {type(heading).__name__}"
            raise TypeError(msg)
        self._angle_deg = angle_deg
        self._cm = cm
        self._speed = speed
        self._until = until
        self._heading_deg = heading
        self._motion: DiagonalMotion | None = None

    def _generate_signature(self) -> str:
        mode = f"{self._cm:.1f}cm" if self._cm else "until"
        heading = (
            f", heading={self._heading_deg:.1f}\u00b0" if self._heading_deg is not None else ""
        )
        return (
            f"DriveAngle(angle={self._angle_deg:.1f}\u00b0, mode={mode}, "
            f"speed={self._speed:.2f}{heading})"
        )

    def on_start(self, robot: "GenericRobot") -> None:
        config = DiagonalMotionConfig()
        config.angle_rad = math.radians(self._angle_deg)
        config.distance_m = self._cm / 100.0 if self._cm is not None else self._SENTINEL_DISTANCE_M
        # When `cm` is None the sentinel distance is only an upper bound;
        # the until-clause terminates the motion. Flag this so the C++
        # controller skips its SpeedMode distance-target check.
        config.has_distance_target = self._cm is not None
        config.speed_scale = self._speed
        # DiagonalMotion closes its heading loop on odometry().getHeading(); the
        # target must live in that same frame. When the user pinned an absolute
        # heading we resolve it through HeadingReferenceService (identical to
        # drive_forward/strafe_*); otherwise we hold the current world heading.
        if self._heading_deg is not None:
            from raccoon.robot.heading_reference import HeadingReferenceService

            ref_svc = robot.get_service(HeadingReferenceService)
            config.target_heading_rad = ref_svc.target_absolute_rad(self._heading_deg)
        else:
            from ._heading_utils import get_world_heading_rad

            config.target_heading_rad = get_world_heading_rad(robot)
        self._motion = DiagonalMotion(robot.drive, robot.odometry, robot.motion_pid_config, config)
        self._motion.start()

        if self._until is not None:
            self._until.start(robot)

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        if self._until is not None and self._until.check(robot):
            return True
        self._motion.update(dt)
        return self._motion.is_finished()

    def lower_to_segments(self) -> "list":
        """Lower a drive-angle into a single segment.

        Axis-aligned angles (0 / ±180 / ±90) lower to a ``linear`` segment;
        true diagonals lower to a ``diagonal`` segment (see below).

        ``DriveAngle`` holds the robot's heading fixed while translating the
        chassis along a vector at ``angle_deg`` relative to that heading. The
        path IR's ``linear`` segment can only express travel along the
        ``Forward`` or ``Lateral`` axis (travel direction == held heading, or
        90° off it) — it has no field to decouple the travel direction from the
        held heading. So only the axis-aligned angles map cleanly:

        - ``0``        → forward  (``Forward`` axis, +sign)
        - ``180``/``-180`` → backward (``Forward`` axis, -sign)
        - ``90``       → right    (``Lateral`` axis, +sign)
        - ``-90``      → left     (``Lateral`` axis, -sign)

        Any other angle is a true diagonal (simultaneous vx+vy at a held
        heading). A single Forward/Lateral linear axis can't express it, so it
        lowers to a ``"diagonal"`` segment carrying the known body-frame
        displacement (``forward_m`` / ``left_m``). That segment runs via an
        opaque adapter (this step's own lifecycle) and ``to_absolute`` integrates
        the body-frame displacement into a waypoint.

        ``angle_deg`` (the robot-centric travel direction) is never stamped
        onto ``heading_deg`` — it is not an absolute heading-reference angle, so
        emitting it there would mis-bind it through ``HeadingReferenceService``.
        The optional ``heading`` argument, on the other hand, *is* an absolute
        heading-reference angle, so it is threaded through as ``heading_deg``
        (linear case) / honoured via the opaque step's ``on_start`` (diagonal
        case). With no ``heading`` the segment holds the current world heading
        at execution — matching ``on_start``'s relative-mode fallback.
        """
        from raccoon.motion import LinearAxis

        from .path.ir import Segment

        # Normalise to (-180, 180] so 180/-180 both map to backward.
        angle = ((self._angle_deg + 180.0) % 360.0) - 180.0
        tol = 1e-6
        if abs(angle) < tol:
            axis, sign = LinearAxis.Forward, 1.0
        elif abs(abs(angle) - 180.0) < tol:
            axis, sign = LinearAxis.Forward, -1.0
        elif abs(angle - 90.0) < tol:
            axis, sign = LinearAxis.Lateral, 1.0
        elif abs(angle + 90.0) < tol:
            axis, sign = LinearAxis.Lateral, -1.0
        else:
            # True diagonal — not representable by a single Forward/Lateral
            # linear axis. Lower to a "diagonal" segment carrying the known
            # body-frame displacement; it runs via an opaque adapter and
            # to_absolute integrates it.
            theta = math.radians(self._angle_deg)
            d = self._cm / 100.0 if self._cm is not None else None
            fwd = d * math.cos(theta) if d is not None else None
            left = -d * math.sin(theta) if d is not None else None  # right=+sin → left=-sin
            return [
                Segment(
                    kind="diagonal",
                    forward_m=fwd,
                    left_m=left,
                    distance_m=d,
                    speed_scale=self._speed,
                    heading_deg=self._heading_deg,
                    condition=self._until,
                    has_known_endpoint=self._cm is not None,
                    opaque_step=self,
                )
            ]

        cm = self._cm
        return [
            Segment(
                kind="linear",
                axis=axis,
                sign=sign,
                distance_m=sign * cm / 100.0 if cm is not None else None,
                speed_scale=self._speed,
                heading_deg=self._heading_deg,
                condition=self._until,
                has_known_endpoint=cm is not None,
            )
        ]


@dsl_step(tags=["motion", "drive"])
class DriveAngleLeft(DriveAngle):
    """Drive at an angle to the left with distance or condition-based termination.

    Convenience wrapper around ``DriveAngle`` that negates the angle so
    that the ``angle_deg`` parameter is always positive (pointing left).

    The angle is measured as degrees to the left of forward:
    ``0`` = forward, ``45`` = forward-left diagonal, ``90`` = pure left.

    Args:
        angle_deg: Degrees to the left of forward (0 = forward, 90 = pure left).
        cm: Distance to travel in centimeters. Omit to use condition-only
            mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination.
        heading: Absolute heading in degrees from the heading reference to
            hold during the drive. ``None`` (default) holds the heading at the
            start of the drive (relative mode).

    Example::

        from raccoon.step.motion import drive_angle_left

        # Drive diagonally forward-left at 45 degrees
        drive_angle_left(45, cm=30)

        # Drive pure left until sensor
        drive_angle_left(90, speed=0.6).until(on_black(s))
    """

    def __init__(
        self,
        angle_deg: float,
        cm: float | None = None,
        speed: float = 1.0,
        until: StopCondition = None,
        heading: float | None = None,
    ) -> None:
        super().__init__(angle_deg=-angle_deg, cm=cm, speed=speed, until=until, heading=heading)


@dsl_step(tags=["motion", "drive"])
class DriveAngleRight(DriveAngle):
    """Drive at an angle to the right with distance or condition-based termination.

    Convenience wrapper around ``DriveAngle`` that passes the angle
    directly (positive = right in the robot frame).

    The angle is measured as degrees to the right of forward:
    ``0`` = forward, ``45`` = forward-right diagonal, ``90`` = pure right.

    Args:
        angle_deg: Degrees to the right of forward (0 = forward, 90 = pure right).
        cm: Distance to travel in centimeters. Omit to use condition-only
            mode.
        speed: Fraction of max speed, 0.0 to 1.0.
        until: Stop condition for early termination.
        heading: Absolute heading in degrees from the heading reference to
            hold during the drive. ``None`` (default) holds the heading at the
            start of the drive (relative mode).

    Example::

        from raccoon.step.motion import drive_angle_right

        # Drive diagonally forward-right at 45 degrees
        drive_angle_right(45, cm=30)

        # Drive pure right until sensor
        drive_angle_right(90, speed=0.6).until(on_black(s))
    """

    def __init__(
        self,
        angle_deg: float,
        cm: float | None = None,
        speed: float = 1.0,
        until: StopCondition = None,
        heading: float | None = None,
    ) -> None:
        super().__init__(angle_deg=angle_deg, cm=cm, speed=speed, until=until, heading=heading)
