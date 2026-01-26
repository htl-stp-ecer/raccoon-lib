from typing import Any
from .. import Step


class DoUntilCheckpoint(Step):

    def __init__(self, checkpoint: float, step) -> None:
        super().__init__()
        self.checkpoint = checkpoint
        self.step = step

    async def _job_while_wait(self, robot: "GenericRobot"):
        await self.step.run_step(robot)

    async def run_step(self, robot: "GenericRobot") -> None:
        await super().run_step(robot)
        await robot.synchronizer.do_until_checkpoint(self.checkpoint, self._job_while_wait, robot)


def do_until_checkpoint(checkpoint: float, step) -> DoUntilCheckpoint:
    """Run a function until a checkpoint is reached"""
    return DoUntilCheckpoint(checkpoint=checkpoint, step=step)