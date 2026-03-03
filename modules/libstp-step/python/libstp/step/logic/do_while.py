import asyncio
from asyncio import CancelledError
from typing import Any

from .. import Step
from ..annotation import dsl


@dsl(hidden=True)
class DoWhileActive(Step):
    """Run one task step only while a reference step is still active."""

    def __init__(self, reference_step: Step, task: Step) -> None:
        """Store the reference step and the concurrent task step."""
        super().__init__()
        self.reference_step = reference_step
        self.task = task

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Cancel the task step as soon as the reference step completes."""
        reference_step = asyncio.create_task(self.reference_step.run_step(robot))
        task = asyncio.create_task(self.task.run_step(robot))

        await reference_step
        task.cancel()
        try:
            await task
        except CancelledError:
            pass


@dsl(tags=["control", "concurrent"])
def do_while_active(reference_step: Step, task: Step):
    """Create a step that cancels ``task`` when ``reference_step`` finishes."""
    return DoWhileActive(reference_step=reference_step, task=task)
