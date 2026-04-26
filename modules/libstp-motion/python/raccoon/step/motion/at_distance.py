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

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl_step(tags=["motion", "wait"])
class WaitUntilDistance(Step):
    """Wait until the robot has driven at least the given distance.

    Polls odometry straight-line distance from the origin at 100 Hz.
    Designed to run inside a ``parallel()`` branch alongside a drive step,
    enabling actions to trigger at specific distances during a drive.

    Args:
        cm: Distance threshold in centimeters.

    Example::

        from raccoon.step import parallel, seq
        from raccoon.step.motion import drive_forward, wait_until_distance
        from raccoon.step.servo import servo

        # Open a servo after driving 30 cm into a 50 cm drive
        parallel(
            [
                drive_forward(50),
                seq([wait_until_distance(30), servo(claw, 90)]),
            ]
        )
    """

    def __init__(self, cm: float) -> None:
        super().__init__()
        self._distance_m = abs(cm) / 100.0
        self._hz = 100

    def _generate_signature(self) -> str:
        return f"WaitUntilDistance(distance_m={self._distance_m:.3f})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        interval = 1.0 / self._hz
        stall_threshold_m = 0.001  # 1 mm
        stall_timeout_s = 0.5

        loop = asyncio.get_event_loop()
        last_progress_dist = 0.0
        last_progress_time = loop.time()

        while True:
            dist = robot.odometry.get_distance_from_origin()
            now = loop.time()

            if dist.straight_line >= self._distance_m:
                self.debug(
                    f"Distance {dist.straight_line:.3f}m >= " f"{self._distance_m:.3f}m — done"
                )
                return

            # Stall detection: if distance hasn't meaningfully changed,
            # the driving motion has likely finished or stalled.
            if dist.straight_line - last_progress_dist > stall_threshold_m:
                last_progress_dist = dist.straight_line
                last_progress_time = now
            elif now - last_progress_time > stall_timeout_s:
                self.error(
                    f"Stall detected: distance stuck at "
                    f"{dist.straight_line:.3f}m for {stall_timeout_s}s, "
                    f"target was {self._distance_m:.3f}m — "
                    f"the concurrent motion likely ended early"
                )
                return

            await asyncio.sleep(interval)
