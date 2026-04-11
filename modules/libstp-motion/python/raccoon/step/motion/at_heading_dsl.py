"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: at_heading.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .at_heading import WaitUntilDegrees, HeadingOrigin


class WaitUntilDegreesBuilder(StepBuilder):
    """Builder for WaitUntilDegrees. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._degrees = _UNSET
        self._origin = HeadingOrigin.STEP_START

    def degrees(self, value: float):
        self._degrees = value
        return self

    def origin(self, value: HeadingOrigin):
        self._origin = value
        return self

    def _build(self):
        kwargs = {}
        if self._degrees is not _UNSET:
            kwargs['degrees'] = self._degrees
        kwargs['origin'] = self._origin
        return WaitUntilDegrees(**kwargs)


@dsl(tags=['motion', 'wait'])
def wait_until_degrees(degrees: float = _UNSET, origin: HeadingOrigin = HeadingOrigin.STEP_START):
    """
    Wait until the robot has turned at least the given number of degrees.

    Polls heading at 100 Hz and compares the angular distance from the
    chosen origin against the threshold.  Designed to run inside a
    ``parallel()`` branch alongside a turn step, enabling actions to trigger
    at specific angles during a turn.

    The origin (zero point) is controlled by the ``origin`` parameter — see
    :class:`HeadingOrigin` for the three available modes.

    Args:
        degrees: Heading-change threshold in degrees (always positive).
        origin: Which reference point to count degrees from. Defaults to ``HeadingOrigin.STEP_START``.

    Returns:
        A WaitUntilDegreesBuilder (chainable via ``.degrees()``, ``.origin()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step import parallel, seq
        from raccoon.step.motion import turn_left, wait_until_degrees, HeadingOrigin
        from raccoon.step.servo import servo

        # Default: fires 45° after this step starts executing
        parallel([
            turn_left(90),
            seq([wait_until_degrees(45), servo(claw, 90)]),
        ])

        # TURN_START: fires at 45° from the turn's own start
        parallel([
            turn_left(90),
            seq([prepare_arm(), wait_until_degrees(45, origin=HeadingOrigin.TURN_START), servo(claw, 90)]),
        ])

        # HEADING_REFERENCE: fires at 45° from the global reference
        parallel([
            turn_left(90),
            seq([wait_until_degrees(45, origin=HeadingOrigin.HEADING_REFERENCE), servo(claw, 90)]),
        ])
    """
    b = WaitUntilDegreesBuilder()
    if degrees is not _UNSET:
        b._degrees = degrees
    b._origin = origin
    return b


__all__ = ['WaitUntilDegreesBuilder', 'wait_until_degrees']
