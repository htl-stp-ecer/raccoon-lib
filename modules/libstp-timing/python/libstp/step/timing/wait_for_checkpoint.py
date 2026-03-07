from typing import Any, Union
from .. import Step
from ..annotation import dsl


@dsl(hidden=True)
class WaitForCheckpoint(Step):
    """Wait until the robot synchronizer reaches a mission-relative checkpoint."""

    def __init__(self, checkpoint_seconds: Union[float, int]) -> None:
        """
        Initialize the checkpoint wait step.

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
        Delegate the wait to ``robot.synchronizer``.
        """
        await robot.synchronizer.wait_until_checkpoint(self.checkpoint_seconds)


@dsl(tags=["timing", "sync"])
def wait_for_checkpoint(checkpoint_seconds: float) -> WaitForCheckpoint:
    """
    Wait until a mission-relative time checkpoint is reached.

    Pauses execution until the robot's global synchronizer clock reaches
    the specified number of seconds since mission start. If the
    checkpoint time has already passed, the step returns immediately.
    This is useful for synchronizing actions to absolute times within a
    timed Botball run (e.g. "at T=20s, start collecting").

    Prerequisites:
        The robot must have a ``synchronizer`` configured. The synchronizer
        clock starts when the mission begins.

    Args:
        checkpoint_seconds: The mission-relative time (in seconds) to
            wait for. Must be non-negative.

    Returns:
        WaitForCheckpoint: A step that blocks until the checkpoint time.

    Example::

        from libstp.step.timing import wait_for_checkpoint

        seq([
            drive_forward(50),
            # Ensure we don't start the next action before T=10s
            wait_for_checkpoint(10.0),
            pick_up_tribble(),
            # Gate the final action to T=25s
            wait_for_checkpoint(25.0),
            drive_to_bin(),
        ])
    """
    return WaitForCheckpoint(checkpoint_seconds=checkpoint_seconds)
