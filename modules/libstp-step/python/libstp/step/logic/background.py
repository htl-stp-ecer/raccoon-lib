"""Background step execution — launch steps without blocking the sequence.

A ``background(step)`` call starts the step asynchronously and returns
immediately so the next step in the sequence can proceed.  If a later
foreground step needs a resource that the background step holds, the
background step is automatically cancelled with a warning.

Use ``wait_for_background()`` to explicitly synchronise with one or all
background steps before continuing.
"""

import asyncio
from typing import Optional, TYPE_CHECKING

from .. import Step
from ..annotation import dsl
from ..base import _in_background
from ..background_manager import get_background_manager

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class Background(Step):
    """Launch a step in the background without blocking the sequence."""

    _composite = True

    def __init__(self, step: Step, name: Optional[str] = None) -> None:
        super().__init__()
        from ..model import StepProtocol

        if not isinstance(step, StepProtocol):
            raise TypeError(f"step must be a Step, got {type(step).__name__}")
        self._step = step
        self._name = name

    def collected_resources(self) -> frozenset[str]:
        # Background resources are preemptable, not exclusive — return
        # empty so Parallel/DoWhileActive validation doesn't block them.
        return frozenset()

    def _generate_signature(self) -> str:
        name_part = f", name={self._name!r}" if self._name else ""
        return f"Background({self._step._generate_signature()}{name_part})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        bg_mgr = get_background_manager(robot)
        label = self._step._generate_signature()
        resources = self._step.collected_resources()

        self.info(f"Launching background: {label}")

        # Mark the new task's context as background so its inner steps
        # skip preemption checks (only foreground steps preempt).
        token = _in_background.set(True)

        async def _run_bg() -> None:
            try:
                await self._step.run_step(robot)
            except asyncio.CancelledError:
                pass

        task = asyncio.create_task(_run_bg())

        # Reset the *parent* context — the task already captured a copy
        # with _in_background=True.
        _in_background.reset(token)

        bg_mgr.register(task, label, self._name, resources)


@dsl(hidden=True)
class WaitForBackground(Step):
    """Wait for one or all background steps to complete."""

    def __init__(self, name: Optional[str] = None) -> None:
        super().__init__()
        self._name = name

    def _generate_signature(self) -> str:
        if self._name:
            return f"WaitForBackground(name={self._name!r})"
        return "WaitForBackground(all)"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        bg_mgr = get_background_manager(robot)
        if self._name:
            await bg_mgr.wait_for_name(self._name)
        else:
            await bg_mgr.wait_all()


@dsl(tags=["control", "background"])
def background(step: Step, name: Optional[str] = None) -> Background:
    """Launch a step in the background without blocking.

    Start executing *step* immediately but continue the sequence without
    waiting for it to finish.  Use ``wait_for_background()`` to
    synchronise later, or let the step run to completion on its own.

    If a later foreground step requires exclusive access to a hardware
    resource that the background step is using, the background step is
    automatically cancelled with a warning — the foreground step takes
    priority.

    Multiple background steps can run concurrently as long as they do
    not claim the same resources.

    Args:
        step: The step to execute in the background.
        name: Optional identifier for later retrieval with
            ``wait_for_background``.  Re-using a name while the
            previous background step is still running logs a warning.

    Returns:
        A Background step that launches *step* asynchronously.

    Example::

        from libstp.step import background, wait_for_background, seq
        from libstp.step.servo import servo_move
        from libstp.step.motion import drive_forward

        # Move a servo while driving — no resource conflict
        seq([
            background(servo_move(0, 1500), name="arm"),
            drive_forward(25),
            wait_for_background("arm"),
        ])
    """
    return Background(step, name)


@dsl(tags=["control", "background"])
def wait_for_background(name: Optional[str] = None) -> WaitForBackground:
    """Wait for background steps to complete.

    Without arguments, wait for **all** running background steps.
    With a *name*, wait only for the background step registered under
    that name.  Returns immediately if the step has already finished
    or was preempted.

    Args:
        name: Identifier of a specific background step to wait for.
            If ``None``, waits for every active background step.

    Returns:
        A WaitForBackground step.

    Example::

        from libstp.step import background, wait_for_background, seq

        seq([
            background(long_running_step(), name="scan"),
            do_something_quick(),
            wait_for_background("scan"),
        ])
    """
    return WaitForBackground(name)
