"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: heading_reference.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .heading_reference import MarkHeadingReference


class MarkHeadingReferenceBuilder(StepBuilder):
    """Builder for MarkHeadingReference. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()

    def _build(self):
        kwargs = {}
        return MarkHeadingReference(**kwargs)


@dsl(tags=['motion', 'turn'])
def mark_heading_reference():
    """
    Mark the current IMU heading as a reference point for absolute turns.

    Captures the robot's current absolute IMU heading and stores it as
    a reference. Subsequent calls to :func:`turn_to_heading` will compute
    turn angles relative to this stored reference, enabling absolute
    heading control even after the robot has moved and turned through
    other motion steps.

    The reference uses the raw IMU heading which is unaffected by
    odometry resets that occur during normal motion steps.

    Multiple calls overwrite the previous reference.

    Returns:
        A MarkHeadingReferenceBuilder (chainable via ).

    Example::

        from libstp.step.motion import mark_heading_reference, turn_to_heading

        # Mark current heading as 0-degree reference
        mark_heading_reference()

        # ... robot drives around ...

        # Turn to face 180 degrees from where we marked
        turn_to_heading(180)
    """
    b = MarkHeadingReferenceBuilder()
    return b


__all__ = ['MarkHeadingReferenceBuilder', 'mark_heading_reference']
