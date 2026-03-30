"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: at_heading.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .at_heading import WaitUntilDegrees


class WaitUntilDegreesBuilder(StepBuilder):
    """Builder for WaitUntilDegrees. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._degrees = _UNSET

    def degrees(self, value: float):
        self._degrees = value
        return self

    def _build(self):
        kwargs = {}
        if self._degrees is not _UNSET:
            kwargs['degrees'] = self._degrees
        return WaitUntilDegrees(**kwargs)


@dsl(tags=['motion', 'wait'])
def wait_until_degrees(degrees: float = _UNSET):
    """
    Wait until the robot has turned at least the given number of degrees.

    Polls odometry heading at 100 Hz and compares the absolute heading change
    against the threshold.  Designed to run inside a ``parallel()`` branch
    alongside a turn step, enabling actions to trigger at specific angles
    during a turn.

    Args:
        degrees: Heading-change threshold in degrees (always positive).

    Returns:
        A WaitUntilDegreesBuilder (chainable via ``.degrees()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step import parallel, seq
        from libstp.step.motion import turn_left, wait_until_degrees
        from libstp.step.servo import servo

        # Open a servo after turning 45° into a 90° turn
        parallel([
            turn_left(90),
            seq([wait_until_degrees(45), servo(claw, 90)]),
        ])
    """
    b = WaitUntilDegreesBuilder()
    if degrees is not _UNSET:
        b._degrees = degrees
    return b


__all__ = ['WaitUntilDegreesBuilder', 'wait_until_degrees']
