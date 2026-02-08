"""
Wall alignment step.

Drives against a wall at constant velocity without heading correction,
allowing the robot to physically align itself against the wall surface.
"""
import asyncio
from enum import Enum
from typing import TYPE_CHECKING

from libstp.foundation import ChassisVelocity

from .. import Step, dsl

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


class WallDirection(Enum):
    """Direction to drive into a wall."""
    FORWARD = "forward"
    BACKWARD = "backward"
    STRAFE_LEFT = "strafe_left"
    STRAFE_RIGHT = "strafe_right"


@dsl(hidden=True)
class WallAlign(Step):
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

    async def _execute_step(self, robot: "GenericRobot") -> None:
        update_rate = 1 / 100
        last_time = asyncio.get_event_loop().time() - update_rate
        start_time = asyncio.get_event_loop().time()

        robot.drive.set_velocity(self._get_velocity())

        while (asyncio.get_event_loop().time() - start_time) < self.duration:
            current_time = asyncio.get_event_loop().time()
            delta_time = max(current_time - last_time, 0.0)
            last_time = current_time

            if delta_time < 1e-4:
                await asyncio.sleep(update_rate)
                continue

            robot.drive.update(delta_time)
            await asyncio.sleep(update_rate)

        robot.drive.hard_stop()


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
