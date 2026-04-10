from typing import Any
from .. import Step
from ..annotation import dsl_step


@dsl_step(tags=["timing", "sync"])
class DoUntilCheckpoint(Step):
    """Run a step until a mission-relative time checkpoint, then cancel it.

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

    Example::

        from raccoon.step.timing import do_until_checkpoint
        from raccoon.step.logic import loop_forever

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

    _composite = True

    def __init__(self, checkpoint: float, step) -> None:
        super().__init__()
        if not isinstance(checkpoint, (int, float)):
            raise TypeError(f"checkpoint must be a number, got {type(checkpoint).__name__}")
        if checkpoint < 0:
            raise ValueError(f"checkpoint must be >= 0, got {checkpoint}")
        from ..model import StepProtocol
        if not isinstance(step, StepProtocol):
            raise TypeError(f"step must be a Step, got {type(step).__name__}")
        self.checkpoint = float(checkpoint)
        self.step = step

    def collected_resources(self) -> frozenset[str]:
        return self.step.collected_resources()

    def _generate_signature(self) -> str:
        return f"DoUntilCheckpoint(checkpoint={self.checkpoint:.1f})"

    async def _job_while_wait(self, robot: "GenericRobot"):
        """Delegate the child execution used by the synchronizer callback."""
        await self.step.run_step(robot)

    async def run_step(self, robot: "GenericRobot") -> None:
        """Record the wrapper step, then execute the child until the checkpoint."""
        await super().run_step(robot)
        await robot.synchronizer.do_until_checkpoint(self.checkpoint, self._job_while_wait, robot)
