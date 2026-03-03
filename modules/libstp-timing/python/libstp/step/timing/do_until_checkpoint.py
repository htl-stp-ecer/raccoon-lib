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
    """Create a step that cancels ``step`` when ``checkpoint`` is reached."""
    return DoUntilCheckpoint(checkpoint=checkpoint, step=step)
