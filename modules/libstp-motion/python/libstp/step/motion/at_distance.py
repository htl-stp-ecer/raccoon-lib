"""
Wait until the robot has driven a certain distance from origin.

Designed to run inside a parallel() branch alongside a drive step:

    parallel(
        drive_forward(50),
        seq([wait_until_distance(30), servo_open()]),
    )

Multiple distance-triggered actions:

    parallel(
        drive_forward(50),
        seq([wait_until_distance(20), arm_lower()]),
        seq([wait_until_distance(30), servo_open()]),
        seq([wait_until_distance(45), servo_close()]),
    )

Polls odometry at 100Hz.  Because LinearMotion.start() resets odometry,
the distance is measured from the start of the concurrent drive step.
"""
import asyncio
from typing import TYPE_CHECKING

from .. import Step
from ..annotation import dsl

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class WaitUntilDistance(Step):
    """Block until odometry straight-line distance exceeds a threshold."""

    def __init__(self, distance_cm: float, hz: int = 100) -> None:
        super().__init__()
        self._distance_m = abs(distance_cm) / 100.0
        self._hz = hz

    def _generate_signature(self) -> str:
        return f"WaitUntilDistance(distance_m={self._distance_m:.3f})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        interval = 1.0 / self._hz

        while True:
            dist = robot.odometry.get_distance_from_origin()
            if dist.straight_line >= self._distance_m:
                self.debug(
                    f"Distance {dist.straight_line:.3f}m >= "
                    f"{self._distance_m:.3f}m — done"
                )
                return
            await asyncio.sleep(interval)


@dsl(tags=["motion", "wait"])
def wait_until_distance(cm: float) -> WaitUntilDistance:
    """
    Wait until the robot has driven at least the given distance.

    Polls odometry straight-line distance from the origin at 100 Hz.
    Designed to run inside a ``parallel()`` branch alongside a drive step,
    enabling actions to trigger at specific distances during a drive.

    Args:
        cm: Distance threshold in centimeters.

    Returns:
        A WaitUntilDistance step.

    Example::

        from libstp.step import parallel, seq
        from libstp.step.motion import drive_forward, wait_until_distance
        from libstp.step.servo import servo

        # Open a servo after driving 30 cm into a 50 cm drive
        parallel([
            drive_forward(50),
            seq([wait_until_distance(30), servo(claw, 90)]),
        ])
    """
    return WaitUntilDistance(distance_cm=cm)
