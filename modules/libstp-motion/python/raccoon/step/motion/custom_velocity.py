"""Motion step driven by a user-supplied velocity function.

Each update cycle the function is called with the current robot and dt;
it returns (vx, vy, omega) as fractions of each axis's configured max
velocity (-1.0 to 1.0) and these are scaled before being forwarded to
the drive velocity controller.  Stops when the optional stop condition
triggers or when an external combinator (e.g. ``do_until_checkpoint``)
cancels it.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

from raccoon.foundation import ChassisVelocity
from raccoon.step.annotation import dsl
from raccoon.step.condition import StopCondition

from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

VelocityFn = Callable[["GenericRobot", float], tuple[float, float, float]]


@dsl(hidden=True)
class CustomVelocity(MotionStep):
    """Motion step driven by a user-supplied velocity function.

    Each update cycle ``velocity_fn(robot, dt)`` is called and the returned
    ``(vx_pct, vy_pct, omega_pct)`` fractions (-1.0 to 1.0) are multiplied
    by the configured max velocities for each axis before being applied to
    the drive velocity controller.  The loop runs until the optional
    ``until`` condition fires, at which point the base ``MotionStep`` calls
    ``on_stop`` and executes ``robot.drive.hard_stop()``.
    """

    def __init__(
        self,
        velocity_fn: VelocityFn,
        until: StopCondition | None = None,
    ) -> None:
        super().__init__()
        if not callable(velocity_fn):
            msg = f"velocity_fn must be callable, got {type(velocity_fn).__name__}"
            raise TypeError(msg)
        if until is not None and not isinstance(until, StopCondition):
            msg = f"until must be a StopCondition, got {type(until).__name__}"
            raise TypeError(msg)
        self._velocity_fn = velocity_fn
        self._until = until

    def _generate_signature(self) -> str:
        fn_name = getattr(self._velocity_fn, "__name__", repr(self._velocity_fn))
        return f"CustomVelocity(fn={fn_name})"

    def on_start(self, robot: "GenericRobot") -> None:
        if self._until is not None:
            self._until.start(robot)

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        if self._until is not None and self._until.check(robot):
            return True
        cfg = robot.motion_pid_config
        vx_pct, vy_pct, omega_pct = self._velocity_fn(robot, dt)
        robot.drive.set_velocity(
            ChassisVelocity(
                vx_pct * cfg.linear.max_velocity,
                vy_pct * cfg.lateral.max_velocity,
                omega_pct * cfg.angular.max_velocity,
            )
        )
        robot.drive.update(dt)
        return False
