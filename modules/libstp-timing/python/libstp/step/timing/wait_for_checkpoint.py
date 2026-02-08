from typing import Any, Union
from .. import Step
from ..annotation import dsl


@dsl(hidden=True)
class WaitForCheckpoint(Step):
    def __init__(self, checkpoint_seconds: Union[float, int]) -> None:
        """
        Initialize a Synchronizer step.

        Args:
            checkpoint_seconds: The number of seconds to wait before synchronizing.

        Raises:
            ValueError: If checkpoint_seconds is negative.
        """
        super().__init__()

        if checkpoint_seconds < 0:
            raise ValueError(f"Checkpoint duration cannot be negative: {checkpoint_seconds}")

        self.checkpoint_seconds = float(checkpoint_seconds)

    async def _execute_step(self, robot: 'GenericRobot') -> None:
        """
        Wait for the specified duration before synchronizing.

        Args:
            robot: The robot to run on.
        """
        await robot.synchronizer.wait_until_checkpoint(self.checkpoint_seconds)


@dsl(tags=["timing", "sync"])
def wait_for_checkpoint(checkpoint_seconds: float) -> WaitForCheckpoint:
    """Synchronize for specified seconds"""
    return WaitForCheckpoint(checkpoint_seconds=checkpoint_seconds)
