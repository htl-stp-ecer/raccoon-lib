"""
Wall alignment step.

Drives against a wall at constant velocity without heading correction,
allowing the robot to physically align itself against the wall surface.
"""
import asyncio
from enum import Enum
from typing import TYPE_CHECKING

from libstp.foundation import ChassisVelocity

from .. import dsl
from .motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


class WallDirection(Enum):
    """Direction to drive into a wall."""
    FORWARD = "forward"
    BACKWARD = "backward"
    STRAFE_LEFT = "strafe_left"
    STRAFE_RIGHT = "strafe_right"


@dsl(hidden=True)
class WallAlign(MotionStep):
    """
    Drive into a wall at constant velocity without heading correction.

    By not applying any PID heading correction, the robot naturally
    rotates to align flush against the wall surface.
    """

    def __init__(self, direction: WallDirection, speed: float, duration: float):
        super().__init__()
        self.direction = direction
        self.speed = speed
        self.duration = duration
        self._deadline: float = 0.0

    def _generate_signature(self) -> str:
        return (
            f"WallAlign(direction={self.direction.value}, "
            f"speed={self.speed:.2f}, duration={self.duration:.2f})"
        )

    def _get_velocity(self) -> ChassisVelocity:
        if self.direction == WallDirection.FORWARD:
            return ChassisVelocity(self.speed, 0.0, 0.0)
        elif self.direction == WallDirection.BACKWARD:
            return ChassisVelocity(-self.speed, 0.0, 0.0)
        elif self.direction == WallDirection.STRAFE_LEFT:
            return ChassisVelocity(0.0, -self.speed, 0.0)
        else:  # STRAFE_RIGHT
            return ChassisVelocity(0.0, self.speed, 0.0)

    def on_start(self, robot: "GenericRobot") -> None:
        robot.drive.set_velocity(self._get_velocity())
        self._deadline = asyncio.get_event_loop().time() + self.duration

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        robot.drive.update(dt)
        return asyncio.get_event_loop().time() >= self._deadline


@dsl(tags=["motion", "wall"])
def wall_align_forward(duration: float = 1.0, speed: float = 0.5) -> WallAlign:
    """
    Drive forward into a wall to align the front of the robot.

    Applies constant forward velocity without heading correction, allowing
    the robot to naturally rotate flush against the wall surface.
    Useful for resetting heading error accumulated during a mission.

    Args:
        duration: Seconds to push against the wall (default 1.0).
        speed: Drive speed in m/s (default 0.5).

    Returns:
        A WallAlign step driving forward.

    Example::

        from libstp.step.motion import wall_align_forward, drive_forward

        # Drive to the wall, then align against it
        seq([drive_forward(40), wall_align_forward(duration=1.5)])
    """
    return WallAlign(WallDirection.FORWARD, abs(speed), duration)


@dsl(tags=["motion", "wall"])
def wall_align_backward(duration: float = 1.0, speed: float = 0.5) -> WallAlign:
    """
    Drive backward into a wall to align the back of the robot.

    Applies constant backward velocity without heading correction, allowing
    the robot to naturally rotate flush against the wall surface.
    Useful after turning around when the back of the robot faces a wall.

    Args:
        duration: Seconds to push against the wall (default 1.0).
        speed: Drive speed in m/s (default 0.5).

    Returns:
        A WallAlign step driving backward.

    Example::

        from libstp.step.motion import wall_align_backward, drive_backward

        # Drive to the wall in reverse, then align against it
        seq([drive_backward(40), wall_align_backward(duration=1.5)])
    """
    return WallAlign(WallDirection.BACKWARD, abs(speed), duration)


@dsl(tags=["motion", "wall"])
def wall_align_strafe_left(duration: float = 1.0, speed: float = 0.3) -> WallAlign:
    """
    Strafe left into a wall to align the left side of the robot.

    Applies constant leftward velocity without heading correction, allowing
    the robot to naturally rotate flush against the wall surface.
    Requires a mecanum or omni drivetrain capable of lateral movement.
    Default speed is lower (0.3 m/s) because strafing has less traction.

    Args:
        duration: Seconds to push against the wall (default 1.0).
        speed: Strafe speed in m/s (default 0.3).

    Returns:
        A WallAlign step strafing left.

    Example::

        from libstp.step.motion import wall_align_strafe_left, drive_forward

        # Drive forward, then strafe-align the left side against a wall
        seq([drive_forward(20), wall_align_strafe_left(duration=1.5)])
    """
    return WallAlign(WallDirection.STRAFE_LEFT, abs(speed), duration)


@dsl(tags=["motion", "wall"])
def wall_align_strafe_right(duration: float = 1.0, speed: float = 0.3) -> WallAlign:
    """
    Strafe right into a wall to align the right side of the robot.

    Applies constant rightward velocity without heading correction, allowing
    the robot to naturally rotate flush against the wall surface.
    Requires a mecanum or omni drivetrain capable of lateral movement.
    Default speed is lower (0.3 m/s) because strafing has less traction.

    Args:
        duration: Seconds to push against the wall (default 1.0).
        speed: Strafe speed in m/s (default 0.3).

    Returns:
        A WallAlign step strafing right.

    Example::

        from libstp.step.motion import wall_align_strafe_right, drive_forward

        # Drive forward, then strafe-align the right side against a wall
        seq([drive_forward(20), wall_align_strafe_right(duration=1.5)])
    """
    return WallAlign(WallDirection.STRAFE_RIGHT, abs(speed), duration)
