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
    """
    Run a task concurrently with a reference step, cancelling the task when the reference finishes.

    Both steps start executing at the same time. When ``reference_step``
    completes (either normally or via exception), ``task`` is immediately
    cancelled. This is useful for running a background activity (e.g.
    sensor polling, motor oscillation) only for as long as a primary
    action is running.

    If ``task`` finishes before ``reference_step``, the reference step
    continues running until it completes on its own.

    Args:
        reference_step: The primary step whose lifetime controls the task.
            When this step finishes, ``task`` is cancelled.
        task: The secondary step that runs concurrently and is cancelled
            once ``reference_step`` completes.

    Returns:
        DoWhileActive: A step that manages the concurrent execution.

    Example::

        from libstp.step.logic import do_while_active, loop_forever

        # Flash an LED while the robot drives forward
        flash_led = loop_forever(seq([
            set_digital(0, True),
            wait(0.25),
            set_digital(0, False),
            wait(0.25),
        ]))
        do_while_active(drive_forward(50), flash_led)
    """
    return DoWhileActive(reference_step=reference_step, task=task)
