"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: wait_for_checkpoint.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .wait_for_checkpoint import WaitForCheckpoint


class WaitForCheckpointBuilder(StepBuilder):
    """Builder for WaitForCheckpoint. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._checkpoint_seconds = _UNSET

    def checkpoint_seconds(self, value: Union[float, int]):
        self._checkpoint_seconds = value
        return self

    def _build(self):
        kwargs = {}
        if self._checkpoint_seconds is not _UNSET:
            kwargs['checkpoint_seconds'] = self._checkpoint_seconds
        return WaitForCheckpoint(**kwargs)


@dsl(tags=['timing', 'sync'])
def wait_for_checkpoint(checkpoint_seconds: Union[float, int] = _UNSET):
    """
    Wait until a mission-relative time checkpoint is reached.

    Pauses execution until the robot's global synchronizer clock reaches
    the specified number of seconds since mission start. If the
    checkpoint time has already passed, the step returns immediately.
    This is useful for synchronizing actions to absolute times within a
    timed Botball run (e.g. "at T=20s, start collecting").

    Prerequisites:
        The robot must have a ``synchronizer`` configured. The synchronizer
        clock starts when the mission begins.

    Args:
        checkpoint_seconds: The mission-relative time (in seconds) to wait for. Must be non-negative.

    Returns:
        A WaitForCheckpointBuilder (chainable via ``.checkpoint_seconds()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.timing import wait_for_checkpoint

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
    b = WaitForCheckpointBuilder()
    if checkpoint_seconds is not _UNSET:
        b._checkpoint_seconds = checkpoint_seconds
    return b


__all__ = ['WaitForCheckpointBuilder', 'wait_for_checkpoint']
