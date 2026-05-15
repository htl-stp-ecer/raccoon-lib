"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: at_distance.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .at_distance import WaitUntilDistance


class WaitUntilDistanceBuilder(StepBuilder):
    """Builder for WaitUntilDistance. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._cm = _UNSET

    def cm(self, value: float):
        self._cm = value
        return self

    def _build(self):
        kwargs = {}
        if self._cm is not _UNSET:
            kwargs["cm"] = self._cm
        return WaitUntilDistance(**kwargs)


@dsl(tags=["motion", "wait"])
def wait_until_distance(cm: float = _UNSET):
    """
    Wait until the robot has driven at least the given distance.

    Polls odometry straight-line distance from the origin at 100 Hz.
    Designed to run inside a ``parallel()`` branch alongside a drive step,
    enabling actions to trigger at specific distances during a drive.

    Args:
        cm: Distance threshold in centimeters.

    Returns:
        A WaitUntilDistanceBuilder (chainable via ``.cm()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step import parallel, seq
        from raccoon.step.motion import drive_forward, wait_until_distance
        from raccoon.step.servo import servo

        # Open a servo after driving 30 cm into a 50 cm drive
        parallel(
            [
                drive_forward(50),
                seq([wait_until_distance(30), servo(claw, 90)]),
            ]
        )
    """
    b = WaitUntilDistanceBuilder()
    if cm is not _UNSET:
        b._cm = cm
    return b


__all__ = ["WaitUntilDistanceBuilder", "wait_until_distance"]
