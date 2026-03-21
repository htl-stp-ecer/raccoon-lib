from typing import TYPE_CHECKING, Literal

from libstp.robot.heading_reference import HeadingReferenceService

from .. import Step, dsl
from ..annotation import dsl_step
from ..logic.defer import Defer
from .turn_dsl import turn_left, turn_right

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


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

    Example::

        from libstp.step.motion import mark_heading_reference, turn_to_heading_right

        # Capture heading origin right after wait-for-light
        mark_heading_reference()

        # ... robot drives around ...

        # Turn to face 90 degrees clockwise from origin
        turn_to_heading_right(90)

        # With offset: robot starts 30° CW from board forward
        mark_heading_reference(origin_offset_deg=-30)
    """

    def __init__(self, origin_offset_deg: float = 0.0) -> None:
        super().__init__()
        self._origin_offset_deg = origin_offset_deg

    def _generate_signature(self) -> str:
        if self._origin_offset_deg != 0.0:
            return f"MarkHeadingReference(offset={self._origin_offset_deg:.1f}°)"
        return "MarkHeadingReference()"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        robot.get_service(HeadingReferenceService).mark(self._origin_offset_deg)


def _build_heading_turn(
    robot: "GenericRobot",
    target_deg: float,
    speed: float,
    force_direction: str | None,
) -> Step:
    """Shared logic for heading turn steps."""
    service = robot.get_service(HeadingReferenceService)
    relative_deg = service.compute_turn(target_deg, force_direction=force_direction)

    if relative_deg >= 0:
        return turn_left(relative_deg, speed=speed)
    else:
        return turn_right(-relative_deg, speed=speed)


@dsl(tags=["motion", "turn"])
def turn_to_heading_right(
    degrees: float,
    speed: float = 1.0,
    force_direction: Literal["left", "right"] | None = None,
) -> Defer:
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
        force_direction: ``"left"`` or ``"right"`` to force the physical
            turn direction regardless of shortest path, or ``None``
            (default) for automatic shortest-path selection.

    Returns:
        A deferred step that computes and executes the turn at runtime.

    Raises:
        RuntimeError: If no heading reference has been set.

    Example::

        from libstp.step.motion import mark_heading_reference, turn_to_heading_right

        # Capture origin after wait-for-light
        mark_heading_reference()

        drive_forward(30)

        # Face 90° clockwise from where we started (shortest path)
        turn_to_heading_right(90)

        # Force turning right even if left would be shorter
        turn_to_heading_right(30, force_direction="right")

        # Return to origin heading
        turn_to_heading_right(0)
    """
    def _build(robot: "GenericRobot") -> Step:
        return _build_heading_turn(robot, -degrees, speed, force_direction)

    return Defer(_build)


@dsl(tags=["motion", "turn"])
def turn_to_heading_left(
    degrees: float,
    speed: float = 1.0,
    force_direction: Literal["left", "right"] | None = None,
) -> Defer:
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
        force_direction: ``"left"`` or ``"right"`` to force the physical
            turn direction regardless of shortest path, or ``None``
            (default) for automatic shortest-path selection.

    Returns:
        A deferred step that computes and executes the turn at runtime.

    Raises:
        RuntimeError: If no heading reference has been set.

    Example::

        from libstp.step.motion import mark_heading_reference, turn_to_heading_left

        # Capture origin after wait-for-light
        mark_heading_reference()

        drive_forward(30)

        # Face 90° counter-clockwise from where we started
        turn_to_heading_left(90)

        # Force turning right to avoid obstacle on the left
        turn_to_heading_left(45, force_direction="right")

        # Return to origin heading
        turn_to_heading_left(0)
    """
    def _build(robot: "GenericRobot") -> Step:
        return _build_heading_turn(robot, degrees, speed, force_direction)

    return Defer(_build)
