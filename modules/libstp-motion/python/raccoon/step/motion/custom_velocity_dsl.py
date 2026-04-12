"""DSL builder and factory function for CustomVelocity."""

from __future__ import annotations

from typing import Optional, Tuple, TYPE_CHECKING

from raccoon.step.annotation import dsl
from raccoon.step.condition import StopCondition
from raccoon.step.step_builder import StepBuilder

from .custom_velocity import CustomVelocity, VelocityFn

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


class CustomVelocityBuilder(StepBuilder):
    """Builder for CustomVelocity. Provides the fluent ``.until()`` method."""

    def __init__(self) -> None:
        super().__init__()
        self._velocity_fn: Optional[VelocityFn] = None
        self._until: Optional[StopCondition] = None

    def until(self, value: StopCondition) -> "CustomVelocityBuilder":
        """Set the stop condition.

        Raises:
            TypeError: If *value* is not a ``StopCondition``.
        """
        if not isinstance(value, StopCondition):
            raise TypeError(
                f"until must be a StopCondition, got {type(value).__name__}"
            )
        self._until = value
        return self

    def _build(self) -> CustomVelocity:
        return CustomVelocity(self._velocity_fn, self._until)


@dsl(tags=["motion", "drive"])
def custom_velocity(
    velocity_fn: VelocityFn,
    until: Optional[StopCondition] = None,
) -> CustomVelocityBuilder:
    """Run a user-defined velocity function each update cycle.

    Calls ``velocity_fn(robot, dt)`` at ~100 Hz and forwards the returned
    ``(vx, vy, omega)`` directly to the chassis velocity controller.  This
    gives full control over all three axes simultaneously — forward/backward
    (``vx``), lateral (``vy``), and rotation (``omega``) — within the same
    closed-loop drive update that every other motion step uses.

    The velocity function receives the live robot handle so it can read
    sensors, odometry, and any other robot state to compute the next
    command.  ``dt`` is the measured time since the last cycle in seconds.

    The step runs until the ``until`` condition fires (or until an external
    combinator such as ``do_until_checkpoint`` cancels it).  On exit the
    drive is hard-stopped.

    Args:
        velocity_fn: Callable with signature
            ``(robot: GenericRobot, dt: float) -> tuple[float, float, float]``
            returning ``(vx, vy, omega)`` as fractions of each axis's
            configured max velocity.  -1.0 = full reverse, 0.0 = stop,
            1.0 = full forward.  Values outside ±1.0 are not clamped but
            will saturate the underlying PID controller.
        until: Optional stop condition.  Can also be set fluently via
            ``.until(condition)`` on the returned builder.

    Returns:
        A ``CustomVelocityBuilder`` (chainable via ``.until()``,
        ``.on_anomaly()``, ``.skip_timing()``).

    Raises:
        TypeError: If ``velocity_fn`` is not callable, or ``until`` is not a
            ``StopCondition``.

    Example::

        from libstp.step.motion import custom_velocity
        from libstp.step.condition import after_seconds, on_black

        # Drive forward at 40% while spinning at 20%
        custom_velocity(lambda robot, dt: (0.4, 0.0, 0.2)).until(after_seconds(3))

        # Simple P heading controller — all values in -1.0..1.0
        import math

        target_rad = math.pi / 2

        def steer_to_heading(robot, dt):
            err = target_rad - robot.odometry.get_heading()
            omega = max(-1.0, min(1.0, err / math.pi))  # normalise to ±1
            return 0.3, 0.0, omega

        custom_velocity(steer_to_heading).until(on_black(sensor))
    """
    if not callable(velocity_fn):
        raise TypeError(
            f"velocity_fn must be callable, got {type(velocity_fn).__name__}"
        )
    if until is not None and not isinstance(until, StopCondition):
        raise TypeError(
            f"until must be a StopCondition, got {type(until).__name__}"
        )
    b = CustomVelocityBuilder()
    b._velocity_fn = velocity_fn
    b._until = until
    return b


__all__ = ["CustomVelocityBuilder", "custom_velocity"]
