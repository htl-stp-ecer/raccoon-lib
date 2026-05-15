"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: custom_velocity.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .custom_velocity import CustomVelocity, VelocityFn


class CustomVelocityBuilder(StepBuilder):
    """Builder for CustomVelocity. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._velocity_fn = _UNSET
        self._until = None

    def velocity_fn(self, value: VelocityFn):
        self._velocity_fn = value
        return self

    def until(self, value: StopCondition | None):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        if self._velocity_fn is not _UNSET:
            kwargs["velocity_fn"] = self._velocity_fn
        kwargs["until"] = self._until
        return CustomVelocity(**kwargs)


@dsl(tags=["motion", "velocity"])
def custom_velocity(velocity_fn: VelocityFn = _UNSET, until: StopCondition | None = None):
    """
    Motion step driven by a user-supplied velocity function.

    Each update cycle ``velocity_fn(robot, dt)`` is called and the returned
    ``(vx_pct, vy_pct, omega_pct)`` fractions (-1.0 to 1.0) are multiplied
    by the configured max velocities for each axis before being applied to
    the drive velocity controller.  The loop runs until the optional
    ``until`` condition fires, at which point the base ``MotionStep`` calls
    ``on_stop`` and executes ``robot.drive.hard_stop()``.

    Args:
        velocity_fn: Callable receiving ``(robot, dt)`` and returning ``(vx_pct, vy_pct, omega_pct)`` fractions in ``[-1.0, 1.0]``.
        until: Optional stop condition checked before each velocity update.

    Returns:
        A CustomVelocityBuilder (chainable via ``.velocity_fn()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motion import custom_velocity

        custom_velocity()
    """
    b = CustomVelocityBuilder()
    if velocity_fn is not _UNSET:
        b._velocity_fn = velocity_fn
    b._until = until
    return b


__all__ = ['CustomVelocityBuilder', 'custom_velocity']
