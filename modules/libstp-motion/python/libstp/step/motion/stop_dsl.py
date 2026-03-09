"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: stop.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .stop import Stop


class StopBuilder(StepBuilder):
    """Builder for Stop. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._hard = True

    def hard(self, value: bool):
        self._hard = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['hard'] = self._hard
        return Stop(**kwargs)


@dsl(tags=['motion', 'stop'])
def stop(hard: bool = True):
    """
    Stop all drive motors immediately.

    Use this between motion sequences or at the end of a mission
    to ensure the robot is stationary.

    Args:
        hard: If ``True`` (default), immediately zero motor output. If ``False``, decelerate smoothly using the drive controller.

    Returns:
        A StopBuilder (chainable via ``.hard()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import drive_forward, stop

        # Drive forward then stop
        seq([drive_forward(50), stop()])
    """
    b = StopBuilder()
    b._hard = hard
    return b


__all__ = ['StopBuilder', 'stop']
