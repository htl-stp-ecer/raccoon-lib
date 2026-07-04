from __future__ import annotations

import math
from typing import TYPE_CHECKING, Literal

from raccoon.motion import TurnConfig, TurnMotion
from raccoon.robot.heading_reference import HeadingReferenceService, TurnDirection

from .. import Step, dsl
from ..annotation import dsl_step
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

__all__ = [
    "MarkHeadingReference",
    "TurnDirection",
    "TurnToHeading",
    "turn_to_heading_left",
    "turn_to_heading_right",
]


@dsl_step(tags=["motion", "turn"])
class MarkHeadingReference(Step):
    """Mark the current IMU heading as a reference point for absolute turns.

    Captures the robot's current absolute IMU heading and stores it as
    a reference. Subsequent calls to :func:`turn_to_heading_right` and
    :func:`turn_to_heading_left` will compute turn angles relative to
    this stored reference, enabling absolute heading control even after
    the robot has moved and turned through other motion steps.

    The reference uses the raw IMU heading which is unaffected by
    odometry resets that occur during normal motion steps.

    Multiple calls overwrite the previous reference.

    Place this step right after ``wait_for_light()`` so the heading
    origin is captured before the robot moves.

    Args:
        origin_offset_deg: Offset in degrees added to the captured
            heading. Use this to define a consistent board-relative
            origin regardless of the robot's physical starting rotation.
            For example, if the robot always starts angled 30° clockwise
            from "forward on the board", pass ``origin_offset_deg=-30``
            so that 0° means "forward on the board".
        positive_direction: Which physical direction is treated as
            positive for subsequent ``turn_to_heading_left`` and
            ``turn_to_heading_right`` calls. ``"left"`` (default) means
            counter-clockwise angles are positive, matching the standard
            mathematical convention. ``"right"`` flips the sign so
            clockwise angles are positive.

    Example::

        from raccoon.step.motion import mark_heading_reference, turn_to_heading_right

        # Capture heading origin right after wait-for-light
        mark_heading_reference()

        # ... robot drives around ...

        # Turn to face 90 degrees clockwise from origin
        turn_to_heading_right(90)

        # With offset: robot starts 30° CW from board forward
        mark_heading_reference(origin_offset_deg=-30)

        # Positive direction is clockwise (right)
        mark_heading_reference(positive_direction="right")
    """

    def __init__(
        self,
        origin_offset_deg: float = 0.0,
        positive_direction: Literal["left", "right"] = "left",
    ) -> None:
        super().__init__()
        self._origin_offset_deg = origin_offset_deg
        self._positive_direction = positive_direction

    def _generate_signature(self) -> str:
        parts = []
        if self._origin_offset_deg != 0.0:
            parts.append(f"offset={self._origin_offset_deg:.1f}°")
        if self._positive_direction != "left":
            parts.append(f"positive={self._positive_direction}")
        inner = ", ".join(parts)
        return f"MarkHeadingReference({inner})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        robot.get_service(HeadingReferenceService).mark(
            self._origin_offset_deg, self._positive_direction
        )


class TurnToHeading(MotionStep):
    """Turn to an absolute heading defined relative to the heading reference.

    This is a KNOWN-endpoint turn: it targets an absolute world heading
    derived from the :class:`HeadingReferenceService`, NOT an opaque
    runtime-deferred angle.  At ``on_start`` it asks the reference service
    for the signed shortest-path delta (``compute_turn``) from the current
    heading to ``target_deg`` (reference-relative, CCW-positive), then turns
    to the resulting ABSOLUTE world heading.

    Because the target is absolute, the underlying :class:`TurnMotion`
    regulates onto a fixed world heading (drift-corrected) rather than
    integrating a pre-computed relative angle.  Behaviour for plain ``seq()``
    use matches the historical step: it resolves the shortest-path turn to the
    reference heading; the only change is that the turn now holds the absolute
    target instead of a one-shot relative angle.

    Users normally go through :func:`turn_to_heading_right` /
    :func:`turn_to_heading_left`.

    Args:
        target_deg: Target heading in degrees relative to the reference,
            CCW-positive (``turn_to_heading_right(d)`` passes ``-d``;
            ``turn_to_heading_left(d)`` passes ``+d``).
        speed: Fraction of max angular speed, 0.0 to 1.0.
        force_direction: :class:`TurnDirection` (or the strings ``"left"`` /
            ``"right"``) to force the physical turn direction, or ``None`` for
            shortest path. An invalid value raises ``ValueError``.
    """

    def __init__(
        self,
        target_deg: float,
        speed: float = 1.0,
        force_direction: TurnDirection | str | None = None,
    ) -> None:
        super().__init__()
        self._target_deg = target_deg
        self._speed = speed
        # Validate eagerly so a typo fails at mission-build time, not mid-run.
        self._force_direction = TurnDirection.coerce(force_direction)
        self._motion: TurnMotion | None = None
        self._done = False

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive"})

    def _generate_signature(self) -> str:
        return f"TurnToHeading(target={self._target_deg:.1f}°)"

    def on_start(self, robot: "GenericRobot") -> None:
        self._motion = None
        self._done = False

        service = robot.get_service(HeadingReferenceService)
        current_deg = service.current_relative_deg()
        relative_deg = service.compute_turn(self._target_deg, force_direction=self._force_direction)

        direction = "left" if relative_deg > 0 else "right"
        service.debug(
            f"turn_to_heading: compensating {abs(relative_deg):.1f}° {direction} "
            f"(from {current_deg:.1f}° → to {self._target_deg:.1f}°, "
            f"delta={relative_deg:+.1f}°)"
        )

        if abs(relative_deg) < 0.1:
            service.debug(
                f"Already at target heading (error={abs(relative_deg):.3f}°) — skipping turn"
            )
            self._done = True
            return

        # TurnMotion regulates on an unwrapped, accumulated heading, so
        # ``target_angle_rad`` is the SIGNED RELATIVE turn delta (positive = CCW,
        # negative = CW). Use the value from ``compute_turn`` verbatim: it is the
        # shortest path when no direction is forced, or an extended ±180..±360
        # delta when ``force_direction`` is set. Normalizing it back to [-π, π]
        # here (e.g. via ``math.remainder``) would silently undo a forced
        # direction — turning a forced 300° left back into a 60° right turn.
        config = TurnConfig()
        config.target_angle_rad = math.radians(relative_deg)
        config.has_angle_target = True
        config.speed_scale = self._speed
        self._motion = TurnMotion(
            robot.drive,
            robot.odometry,
            robot.motion_pid_config,
            config,
        )
        self._motion.start()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        if self._done:
            return True
        if self._motion is None:
            return True
        self._motion.update(dt)
        return self._motion.is_finished()

    def lower_to_segments(self) -> "list":
        from .path.ir import Segment

        return [
            Segment(
                kind="turn",
                opaque_step=self,
                has_known_endpoint=True,
                angle_rad=None,
                speed_scale=self._speed,
            )
        ]


@dsl(tags=["motion", "turn"])
def turn_to_heading_right(
    degrees: float,
    speed: float = 1.0,
    force_direction: TurnDirection | str | None = None,
) -> TurnToHeading:
    """Turn to face a heading measured clockwise from the origin.

    Computes the absolute target heading as ``origin - degrees`` (since
    clockwise is the negative direction), then turns via the shortest
    path. The actual turn direction (left or right) is chosen
    automatically to minimize rotation — only the target angle convention
    is clockwise.

    Use ``force_direction`` to override the automatic shortest-path
    choice when obstacles prevent turning in one direction.

    Requires :func:`mark_heading_reference` to have been called earlier
    in the mission.

    Args:
        degrees: Angle in degrees clockwise from the heading origin.
            Must be positive. For example, 90 means "face 90° to the
            right of origin".
        speed: Fraction of max angular speed, 0.0 to 1.0 (default 1.0).
        force_direction: :class:`TurnDirection` (``TurnDirection.LEFT`` /
            ``TurnDirection.RIGHT``, or the equivalent strings ``"left"`` /
            ``"right"``) to force the physical turn direction regardless of
            shortest path, or ``None`` (default) for automatic shortest-path
            selection. An invalid value raises ``ValueError``.

    Returns:
        A :class:`TurnToHeading` step that resolves the shortest-path turn
        to the absolute reference heading at start time.

    Raises:
        RuntimeError: If no heading reference has been set.
        ValueError: If ``force_direction`` is not a valid direction.

    Example::

        from raccoon.step.motion import mark_heading_reference, turn_to_heading_right
        from raccoon.robot.heading_reference import TurnDirection

        # Capture origin after wait-for-light
        mark_heading_reference()

        drive_forward(30)

        # Face 90° clockwise from where we started (shortest path)
        turn_to_heading_right(90)

        # Force turning right even if left would be shorter
        turn_to_heading_right(30, force_direction=TurnDirection.RIGHT)

        # Return to origin heading
        turn_to_heading_right(0)
    """
    return TurnToHeading(-degrees, speed=speed, force_direction=force_direction)


@dsl(tags=["motion", "turn"])
def turn_to_heading_left(
    degrees: float,
    speed: float = 1.0,
    force_direction: TurnDirection | str | None = None,
) -> TurnToHeading:
    """Turn to face a heading measured counter-clockwise from the origin.

    Computes the absolute target heading as ``origin + degrees`` (since
    counter-clockwise is the positive direction), then turns via the
    shortest path. The actual turn direction (left or right) is chosen
    automatically to minimize rotation — only the target angle convention
    is counter-clockwise.

    Use ``force_direction`` to override the automatic shortest-path
    choice when obstacles prevent turning in one direction.

    Requires :func:`mark_heading_reference` to have been called earlier
    in the mission.

    Args:
        degrees: Angle in degrees counter-clockwise from the heading
            origin. Must be positive. For example, 90 means "face 90°
            to the left of origin".
        speed: Fraction of max angular speed, 0.0 to 1.0 (default 1.0).
        force_direction: :class:`TurnDirection` (``TurnDirection.LEFT`` /
            ``TurnDirection.RIGHT``, or the equivalent strings ``"left"`` /
            ``"right"``) to force the physical turn direction regardless of
            shortest path, or ``None`` (default) for automatic shortest-path
            selection. An invalid value raises ``ValueError``.

    Returns:
        A :class:`TurnToHeading` step that resolves the shortest-path turn
        to the absolute reference heading at start time.

    Raises:
        RuntimeError: If no heading reference has been set.
        ValueError: If ``force_direction`` is not a valid direction.

    Example::

        from raccoon.step.motion import mark_heading_reference, turn_to_heading_left
        from raccoon.robot.heading_reference import TurnDirection

        # Capture origin after wait-for-light
        mark_heading_reference()

        drive_forward(30)

        # Face 90° counter-clockwise from where we started
        turn_to_heading_left(90)

        # Force turning right to avoid obstacle on the left
        turn_to_heading_left(45, force_direction=TurnDirection.RIGHT)

        # Return to origin heading
        turn_to_heading_left(0)
    """
    return TurnToHeading(degrees, speed=speed, force_direction=force_direction)
