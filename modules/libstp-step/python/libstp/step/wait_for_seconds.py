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
        Wait for the specified duration.
        
        Args:
            device: The device to run on (not used in this step)
            definitions: Additional definitions needed for execution
        """

        # Simply wait for the specified duration
        await asyncio.sleep(self.seconds)


@dsl(tags=["timing", "wait"])
def wait(seconds: float) -> WaitForSeconds:
    """Wait for specified seconds"""
    return WaitForSeconds(seconds=seconds)
