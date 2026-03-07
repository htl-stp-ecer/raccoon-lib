from typing import Any
from .. import Step
from ..annotation import dsl


@dsl(hidden=True)
class DoUntilCheckpoint(Step):
    """Run one child step until the robot synchronizer reaches a checkpoint."""

    def __init__(self, checkpoint: float, step) -> None:
        """Store the checkpoint deadline and the child step to cancel there."""
        super().__init__()
        self.checkpoint = checkpoint
        self.step = step

    async def _job_while_wait(self, robot: "GenericRobot"):
        """Delegate the child execution used by the synchronizer callback."""
        await self.step.run_step(robot)

    async def run_step(self, robot: "GenericRobot") -> None:
        """Record the wrapper step, then execute the child until the checkpoint."""
        await super().run_step(robot)
        await robot.synchronizer.do_until_checkpoint(self.checkpoint, self._job_while_wait, robot)


@dsl(tags=["timing", "sync"])
def do_until_checkpoint(checkpoint: float, step) -> DoUntilCheckpoint:
    """
    Run a step until a mission-relative time checkpoint, then cancel it.

    Starts executing ``step`` immediately and cancels it when the robot's
    global synchronizer clock reaches ``checkpoint`` seconds since mission
    start. If the step finishes before the checkpoint, execution continues
    without waiting. This is useful for time-boxing actions within a timed
    Botball run (e.g. "search for objects, but stop at T=45s no matter what").

    Prerequisites:
        The robot must have a ``synchronizer`` configured. The synchronizer
        clock starts when the mission begins.

    Args:
        checkpoint: The mission-relative deadline (in seconds) at which
            ``step`` will be cancelled.
        step: The step to run. Will be cancelled if still active when the
            checkpoint time is reached.

    Returns:
        DoUntilCheckpoint: A step that manages the time-boxed execution.

    Example::

        from libstp.step.timing import do_until_checkpoint
        from libstp.step.logic import loop_forever

        # Search for objects until T=45s, then move on
        search = loop_forever(seq([
            scan_for_object(),
            drive_forward(10),
        ]))
        seq([
            do_until_checkpoint(45.0, search),
            drive_to_start(),
        ])
    """
    return DoUntilCheckpoint(checkpoint=checkpoint, step=step)
