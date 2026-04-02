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
        self._origin_offset_deg = 0.0
        self._positive_direction = 'left'

    def origin_offset_deg(self, value: float):
        self._origin_offset_deg = value
        return self

    def positive_direction(self, value: Literal['left', 'right']):
        self._positive_direction = value
        return self

    def _build(self):
        kwargs = {}
        kwargs['origin_offset_deg'] = self._origin_offset_deg
        kwargs['positive_direction'] = self._positive_direction
        return MarkHeadingReference(**kwargs)


@dsl(tags=['motion', 'turn'])
def mark_heading_reference(origin_offset_deg: float = 0.0, positive_direction: Literal['left', 'right'] = 'left'):
    """
    Mark the current IMU heading as a reference point for absolute turns.

    Captures the robot's current absolute IMU heading and stores it as
    a reference. Subsequent calls to :func:`turn_to_heading_right` and
    :func:`turn_to_heading_left` will compute turn angles relative to
    this stored reference, enabling absolute heading control even after
    the robot has moved and turned through other motion steps.

    The reference uses the raw IMU heading which is unaffected by
    odometry resets that occur during normal motion steps.

    Multiple calls overwrite the previous reference.

    Place this step right after ``wait_for_light()`` so the heading
    origin is captured before the robot moves.

    Args:
        origin_offset_deg: Offset in degrees added to the captured heading. Use this to define a consistent board-relative origin regardless of the robot's physical starting rotation. For example, if the robot always starts angled 30° clockwise from "forward on the board", pass ``origin_offset_deg=-30`` so that 0° means "forward on the board".
        positive_direction: Which physical direction is treated as positive for subsequent ``turn_to_heading_left`` and ``turn_to_heading_right`` calls. ``"left"`` (default) means counter-clockwise angles are positive, matching the standard mathematical convention. ``"right"`` flips the sign so clockwise angles are positive.

    Returns:
        A MarkHeadingReferenceBuilder (chainable via ``.origin_offset_deg()``, ``.positive_direction()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import mark_heading_reference, turn_to_heading_right

        # Capture heading origin right after wait-for-light
        mark_heading_reference()

        # ... robot drives around ...

        # Turn to face 90 degrees clockwise from origin
        turn_to_heading_right(90)

        # With offset: robot starts 30° CW from board forward
        mark_heading_reference(origin_offset_deg=-30)

        # Positive direction is clockwise (right)
        mark_heading_reference(positive_direction="right")
    """
    b = MarkHeadingReferenceBuilder()
    b._origin_offset_deg = origin_offset_deg
    b._positive_direction = positive_direction
    return b


__all__ = ['MarkHeadingReferenceBuilder', 'mark_heading_reference']
