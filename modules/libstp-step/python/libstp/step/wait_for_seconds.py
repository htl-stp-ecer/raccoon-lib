import asyncio
from typing import Any, Union
from . import Step

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

    async def run_step(self, robot) -> None:
        """
        Wait for the specified duration.
        
        Args:
            device: The device to run on (not used in this step)
            definitions: Additional definitions needed for execution
        """
        await super().run_step(robot)

        robot.drive.hard_stop()

        # Simply wait for the specified duration
        await asyncio.sleep(self.seconds)


def wait(seconds: float) -> WaitForSeconds:
    """Wait for specified seconds"""
    return WaitForSeconds(seconds=seconds)
