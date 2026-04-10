from typing import Any, Union
from .. import Step
from ..annotation import dsl_step


@dsl_step(tags=["timing", "sync"])
class WaitForCheckpoint(Step):
    """Wait until a mission-relative time checkpoint is reached.

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

    Example::

        from raccoon.step.timing import wait_for_checkpoint

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

    def __init__(self, checkpoint_seconds: Union[float, int]) -> None:
        super().__init__()

        if checkpoint_seconds < 0:
            raise ValueError(f"Checkpoint duration cannot be negative: {checkpoint_seconds}")

        self.checkpoint_seconds = float(checkpoint_seconds)

    def _generate_signature(self) -> str:
        return f"WaitForCheckpoint(seconds={self.checkpoint_seconds:.1f})"

    async def _execute_step(self, robot: 'GenericRobot') -> None:
        await robot.synchronizer.wait_until_checkpoint(self.checkpoint_seconds)
