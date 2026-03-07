from typing import TYPE_CHECKING

from libstp.robot.heading_reference import HeadingReferenceService

from .. import Step, dsl
from ..logic.defer import Defer
from .turn import turn_left, turn_right

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class MarkHeadingReference(Step):
    """Captures the absolute IMU heading as a reference for future absolute turns."""

    def _generate_signature(self) -> str:
        return "MarkHeadingReference()"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        robot.get_service(HeadingReferenceService).mark()


@dsl(tags=["motion", "turn"])
def mark_heading_reference() -> MarkHeadingReference:
    """Mark the current IMU heading as a reference point for absolute turns.

    Captures the robot's current absolute IMU heading and stores it as
    a reference. Subsequent calls to :func:`turn_to_heading` will compute
    turn angles relative to this stored reference, enabling absolute
    heading control even after the robot has moved and turned through
    other motion steps.

    The reference uses the raw IMU heading which is unaffected by
    odometry resets that occur during normal motion steps.

    Multiple calls overwrite the previous reference.

    Returns:
        A MarkHeadingReference step that records the heading when executed.

    Example::

        from libstp.step.motion import mark_heading_reference, turn_to_heading

        # Mark current heading as 0-degree reference
        mark_heading_reference()

        # ... robot drives around ...

        # Turn to face 180 degrees from where we marked
        turn_to_heading(180)
    """
    return MarkHeadingReference()


@dsl(tags=["motion", "turn"])
def turn_to_heading(degrees: float, speed: float = 1.0) -> Defer:
    """Turn to an absolute heading relative to the marked reference.

    Computes the shortest rotation from the robot's current heading to
    the target heading (reference + degrees) at execution time, then
    delegates to :func:`turn_left` or :func:`turn_right` accordingly.
    The turn direction is chosen automatically to minimize rotation.

    Requires :func:`mark_heading_reference` to have been called earlier
    in the mission.

    Args:
        degrees: Target heading in degrees relative to the reference.
            0 returns to the reference heading, 90 faces 90 degrees
            counter-clockwise from it, -90 faces 90 degrees clockwise, etc.
        speed: Fraction of max angular speed, 0.0 to 1.0 (default 1.0).

    Returns:
        A deferred step that computes and executes the turn at runtime.

    Raises:
        RuntimeError: If no heading reference has been set.

    Example::

        from libstp.step.motion import mark_heading_reference, turn_to_heading

        # Mark heading at start of mission
        mark_heading_reference()

        # Drive around, turn, etc.
        drive_forward(30)
        turn_left(45)
        drive_forward(20)

        # Return to original heading
        turn_to_heading(0)

        # Face the opposite direction from reference
        turn_to_heading(180)
    """
    def _build(robot: "GenericRobot") -> Step:
        service = robot.get_service(HeadingReferenceService)
        relative_deg = service.compute_turn(degrees)

        if relative_deg >= 0:
            return turn_left(relative_deg, speed=speed)
        else:
            return turn_right(-relative_deg, speed=speed)

    return Defer(_build)
