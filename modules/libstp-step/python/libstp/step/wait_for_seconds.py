import asyncio
from typing import Union

from . import Step, SimulationStep, SimulationStepDelta
from .annotation import dsl


@dsl(hidden=True)
class WaitForSeconds(Step):
    """
    A step that waits for a specified duration before completing.
    """

    def __init__(self, seconds: Union[float, int]) -> None:
        """
        Initialize a WaitForSeconds step.
        
        Args:
            seconds: The number of seconds to wait.
            
        Raises:
            ValueError: If seconds is negative.
        """
        super().__init__()

        if seconds < 0:
            raise ValueError(f"Wait duration cannot be negative: {seconds}")

        self.seconds = float(seconds)

    def _generate_signature(self) -> str:
        return f"Wait(seconds={self.seconds:.3f})"

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        # Wait doesn't move the robot, but has a known duration
        base.delta = SimulationStepDelta(forward=0.0, strafe=0.0, angular=0.0)
        base.average_duration_ms = self.seconds * 1000.0
        base.duration_stddev_ms = 0.0  # Wait is deterministic
        return base

    async def _execute_step(self, robot) -> None:
        """
        Sleep for the configured duration without touching the robot.
        """
        await asyncio.sleep(self.seconds)


@dsl(tags=["timing", "wait"])
def wait(seconds: float) -> WaitForSeconds:
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

    Returns:
        A ``WaitForSeconds`` step ready to be scheduled in a step
        sequence.

    Example::

        from libstp.step import wait

        # Pause between two motor commands
        sequence(
            motor_power(robot.motor(0), 100),
            wait(2.5),
            motor_brake(robot.motor(0)),
        )

        # Brief yield to let sensors settle
        wait(0.1)
    """
    return WaitForSeconds(seconds=seconds)
