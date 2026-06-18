from __future__ import annotations

import asyncio

from . import Step
from .annotation import dsl_step


@dsl_step(tags=["timing", "wait"])
class WaitForSeconds(Step):
    """Pause execution for a fixed number of seconds.

    Suspends the current step sequence for the specified duration using
    an async sleep. No hardware commands are issued during the wait, and
    other concurrent tasks (e.g. odometry updates) continue running
    normally. The wait duration is deterministic and is reflected
    accurately in simulation estimates.

    Args:
        seconds: Duration to pause in seconds. Must be non-negative.
            Passing 0 yields control to the event loop for one tick
            without any meaningful delay.

    Example::

        from raccoon.step import wait

        # Pause between two motor commands
        sequence(
            motor_power(robot.motor(0), 100),
            wait(2.5),
            motor_brake(robot.motor(0)),
        )

        # Brief yield to let sensors settle
        wait(0.1)
    """

    def __init__(self, seconds: float | int) -> None:
        super().__init__()

        if seconds < 0:
            msg = f"Wait duration cannot be negative: {seconds}"
            raise ValueError(msg)

        self.seconds = float(seconds)

    def _generate_signature(self) -> str:
        return f"Wait(seconds={self.seconds:.3f})"

    async def _execute_step(self, robot) -> None:
        """Sleep for the configured duration without touching the robot."""
        await asyncio.sleep(self.seconds)
