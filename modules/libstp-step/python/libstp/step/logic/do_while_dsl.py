"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: do_while.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .do_while import DoWhileActive


class DoWhileActiveBuilder(StepBuilder):
    """Builder for DoWhileActive. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._reference_step = _UNSET
        self._task = _UNSET

    def reference_step(self, value: Step):
        self._reference_step = value
        return self

    def task(self, value: Step):
        self._task = value
        return self

    def _build(self):
        kwargs = {}
        if self._reference_step is not _UNSET:
            kwargs['reference_step'] = self._reference_step
        if self._task is not _UNSET:
            kwargs['task'] = self._task
        return DoWhileActive(**kwargs)


@dsl(tags=['control', 'concurrent'])
def do_while_active(reference_step: Step = _UNSET, task: Step = _UNSET):
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
        reference_step: The primary step whose lifetime controls the task. When this step finishes, ``task`` is cancelled.
        task: The secondary step that runs concurrently and is cancelled once ``reference_step`` completes.

    Returns:
        A DoWhileActiveBuilder (chainable via ``.reference_step()``, ``.task()``).

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
    b = DoWhileActiveBuilder()
    if reference_step is not _UNSET:
        b._reference_step = reference_step
    if task is not _UNSET:
        b._task = task
    return b


__all__ = ['DoWhileActiveBuilder', 'do_while_active']
