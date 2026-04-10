"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: wait_for.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .wait_for import WaitFor


class WaitForBuilder(StepBuilder):
    """Builder for WaitFor. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._condition = _UNSET

    def condition(self, value: StopCondition):
        self._condition = value
        return self

    def _build(self):
        kwargs = {}
        if self._condition is not _UNSET:
            kwargs['condition'] = self._condition
        return WaitFor(**kwargs)


@dsl(tags=['timing', 'wait', 'sensor'])
def wait_for(condition: StopCondition = _UNSET):
    """
    Block until a stop condition is satisfied.

    Initializes the condition, then polls it at 50 Hz until it returns
    True. This lets you reuse any composable ``StopCondition`` — sensor
    triggers, timers, distance thresholds, or combinations — as a
    standalone wait without an accompanying motion.

    Args:
        condition: A ``StopCondition`` instance (or composition) to wait for.  Supports the same ``|``, ``&``, and ``+`` operators as motion ``.until()`` clauses.

    Returns:
        A WaitForBuilder (chainable via ``.condition()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step import wait_for
        from raccoon.step.condition import on_black, after_seconds

        # Wait until the front-left IR sensor sees black
        wait_for(on_black(Defs.front.left))

        # Wait for black OR a 5-second timeout
        wait_for(on_black(sensor) | after_seconds(5))
    """
    b = WaitForBuilder()
    if condition is not _UNSET:
        b._condition = condition
    return b


__all__ = ['WaitForBuilder', 'wait_for']
