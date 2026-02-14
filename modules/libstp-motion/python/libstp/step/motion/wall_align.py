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

    Args:
        duration: How long to push against the wall in seconds (default 1.0)
        speed: Drive speed in m/s (default 0.5)
    """
    return WallAlign(WallDirection.FORWARD, abs(speed), duration)


@dsl(tags=["motion", "wall"])
def wall_align_backward(duration: float = 1.0, speed: float = 0.5) -> WallAlign:
    """
    Drive backward into a wall to align the back of the robot.

    Args:
        duration: How long to push against the wall in seconds (default 1.0)
        speed: Drive speed in m/s (default 0.5)
    """
    return WallAlign(WallDirection.BACKWARD, abs(speed), duration)


@dsl(tags=["motion", "wall"])
def wall_align_strafe_left(duration: float = 1.0, speed: float = 0.3) -> WallAlign:
    """
    Strafe left into a wall to align the left side of the robot.

    Args:
        duration: How long to push against the wall in seconds (default 1.0)
        speed: Strafe speed in m/s (default 0.3)
    """
    return WallAlign(WallDirection.STRAFE_LEFT, abs(speed), duration)


@dsl(tags=["motion", "wall"])
def wall_align_strafe_right(duration: float = 1.0, speed: float = 0.3) -> WallAlign:
    """
    Strafe right into a wall to align the right side of the robot.

    Args:
        duration: How long to push against the wall in seconds (default 1.0)
        speed: Strafe speed in m/s (default 0.3)
    """
    return WallAlign(WallDirection.STRAFE_RIGHT, abs(speed), duration)
