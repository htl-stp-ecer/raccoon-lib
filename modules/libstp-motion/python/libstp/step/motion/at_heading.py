"""
Wait until the robot has turned a certain number of degrees from its starting heading.

Designed to run inside a parallel() branch alongside a turn step:

    parallel(
        turn_left(90),
        seq([wait_until_degrees(45), servo_open()]),
    )

Multiple heading-triggered actions:

    parallel(
        turn_left(90),
        seq([wait_until_degrees(30), arm_lower()]),
        seq([wait_until_degrees(60), servo_open()]),
        seq([wait_until_degrees(85), servo_close()]),
    )

Polls odometry heading at 100Hz.  Because TurnMotion.start() resets odometry,
the heading is measured from the start of the concurrent turn step.
"""
import asyncio
import math
from typing import TYPE_CHECKING

from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl_step(tags=["motion", "wait"])
class WaitUntilDegrees(Step):
    """Wait until the robot has turned at least the given number of degrees.

    Polls odometry heading at 100 Hz and compares the absolute heading change
    against the threshold.  Designed to run inside a ``parallel()`` branch
    alongside a turn step, enabling actions to trigger at specific angles
    during a turn.

    Args:
        degrees: Heading-change threshold in degrees (always positive).

    Example::

        from libstp.step import parallel, seq
        from libstp.step.motion import turn_left, wait_until_degrees
        from libstp.step.servo import servo

        # Open a servo after turning 45° into a 90° turn
        parallel([
            turn_left(90),
            seq([wait_until_degrees(45), servo(claw, 90)]),
        ])
    """

    def __init__(self, degrees: float) -> None:
        super().__init__()
        self._threshold_rad = math.radians(abs(degrees))
        self._hz = 100

    def _generate_signature(self) -> str:
        return f"WaitUntilDegrees(degrees={math.degrees(self._threshold_rad):.1f})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        interval = 1.0 / self._hz
        stall_threshold_rad = math.radians(0.5)  # 0.5°
        stall_timeout_s = 0.5

        initial_heading = robot.odometry.get_heading()
        loop = asyncio.get_event_loop()
        last_progress_rad = 0.0
        last_progress_time = loop.time()

        while True:
            current_heading = robot.odometry.get_heading()
            turned_rad = abs(current_heading - initial_heading)
            now = loop.time()

            if turned_rad >= self._threshold_rad:
                self.debug(
                    f"Turned {math.degrees(turned_rad):.1f}° >= "
                    f"{math.degrees(self._threshold_rad):.1f}° — done"
                )
                return

            # Stall detection: if heading hasn't meaningfully changed,
            # the turning motion has likely finished or stalled.
            if turned_rad - last_progress_rad > stall_threshold_rad:
                last_progress_rad = turned_rad
                last_progress_time = now
            elif now - last_progress_time > stall_timeout_s:
                self.error(
                    f"Stall detected: heading stuck at "
                    f"{math.degrees(turned_rad):.1f}° for {stall_timeout_s}s, "
                    f"target was {math.degrees(self._threshold_rad):.1f}° — "
                    f"the concurrent motion likely ended early"
                )
                return

            await asyncio.sleep(interval)
