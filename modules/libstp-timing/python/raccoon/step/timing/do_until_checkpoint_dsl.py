"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: do_until_checkpoint.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .do_until_checkpoint import DoUntilCheckpoint


class DoUntilCheckpointBuilder(StepBuilder):
    """Builder for DoUntilCheckpoint. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._checkpoint = _UNSET
        self._step = _UNSET

    def checkpoint(self, value: float):
        self._checkpoint = value
        return self

    def step(self, value):
        self._step = value
        return self

    def _build(self):
        kwargs = {}
        if self._checkpoint is not _UNSET:
            kwargs["checkpoint"] = self._checkpoint
        if self._step is not _UNSET:
            kwargs["step"] = self._step
        return DoUntilCheckpoint(**kwargs)


@dsl(tags=["timing", "sync"])
def do_until_checkpoint(checkpoint: float = _UNSET, step=_UNSET):
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
        checkpoint: The mission-relative deadline (in seconds) at which ``step`` will be cancelled.
        step: The step to run. Will be cancelled if still active when the checkpoint time is reached.

    Returns:
        A DoUntilCheckpointBuilder (chainable via ``.checkpoint()``, ``.step()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.timing import do_until_checkpoint
        from raccoon.step.logic import loop_forever

        # Search for objects until T=45s, then move on
        search = loop_forever(
            seq(
                [
                    scan_for_object(),
                    drive_forward(10),
                ]
            )
        )
        seq(
            [
                do_until_checkpoint(45.0, search),
                drive_to_start(),
            ]
        )
    """
    b = DoUntilCheckpointBuilder()
    if checkpoint is not _UNSET:
        b._checkpoint = checkpoint
    if step is not _UNSET:
        b._step = step
    return b


__all__ = ["DoUntilCheckpointBuilder", "do_until_checkpoint"]
