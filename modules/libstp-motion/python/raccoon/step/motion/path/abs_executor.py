"""Thin dispatcher for the absolute-pose plan IR — Phase 3, Commit C.

This module runs a :class:`CompiledAbsolutePlan` by reading the world pose
from ``robot.localization.get_pose()`` at each node, computing a relative
delta, and delegating to the **existing relative motion steps**
(``drive_forward``, ``turn_left``, ``turn_right``). It is deliberately
dumb: one ``Goto`` desugars to an optional turn-in-place plus a
``drive_forward`` over the straight-line distance to the target.

This is **not** the long-term execution path. Phase 4 wires the motion
classes themselves to consume absolute targets, at which point the
read-pose-then-delta dance below disappears. Until then this dispatcher
gives Phase 3 something runnable without touching :class:`PathExecutor`,
the relative IR, or any motion class.

Out of scope (intentionally):

* No cornering, splines, or smooth-path machinery.
* No side-action lifting / scheduling — non-blocking ``Action`` is a stub.
* No optimizer passes, no heading inference beyond the immediate vector.
* No ``Resync`` execution — that lands in Phase 5/6.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from .abs_compiler import CompiledAbsolutePlan
from .abs_ir import Action, Goto, Resync, TurnTo

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# Tolerance below which a turn is skipped entirely. ~1° matches the
# coarseness of the relative turn controllers and keeps unit tests stable.
_TURN_TOL_RAD = math.radians(1.0)


def _wrap_angle(rad: float) -> float:
    """Wrap ``rad`` into ``(-π, π]`` (matches the C++ ``wrapAngle`` helper).

    Convention: ``+π`` stays positive, ``-π`` is mapped to ``+π``. This
    ensures a 180° turn from heading 0 dispatches as ``turn_left(180)``.
    """
    wrapped = math.fmod(rad + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


class AbsolutePathExecutor:
    """Dispatch a compiled absolute plan onto the relative motion DSL.

    Stateless. One executor instance can run many plans sequentially.

    Usage::

        plan = compile_plan([goto(80, 0), turn_to(90), goto(80, 50)])
        await AbsolutePathExecutor().run(robot, plan)
    """

    def __init__(self) -> None:
        # Intentionally no state — every node reads pose fresh from
        # localization. Keeping this class instantiable (rather than
        # exposing a free function) leaves room for Phase-4 diagnostics
        # without breaking the call site.
        return

    async def run(self, robot: "GenericRobot", plan: CompiledAbsolutePlan) -> None:
        """Execute ``plan`` against ``robot``.

        Each node reads ``robot.localization.get_pose()`` afresh, so
        per-node errors do not accumulate the way a delta-only chain
        would. ``Resync`` raises :class:`NotImplementedError` (Phase 5/6).

        Raises:
            RuntimeError: If ``robot.localization`` is ``None`` — the
                executor cannot dispatch absolute targets without it.
            NotImplementedError: On ``Resync`` nodes, or on
                ``Action(blocking=False)`` (background scheduling lands in
                Phase 4 alongside the absolute-aware motion classes).
        """
        if getattr(robot, "localization", None) is None:
            msg = (
                "robot.localization is required for absolute path execution; "
                "ensure your robot subclass enables it (Phase 2 wiring)."
            )
            raise RuntimeError(msg)

        # Local import: keeps abs_executor importable without dragging the
        # whole DSL into module load and dodges circular-import risk via
        # ``path/__init__.py`` re-exports.
        from ..drive_dsl import drive_forward
        from ..turn_dsl import turn_left, turn_right

        for node in plan.nodes:
            if isinstance(node, TurnTo):
                pose = robot.localization.get_pose()
                delta = _wrap_angle(node.theta_rad - pose.heading)
                await self._dispatch_turn(robot, delta, turn_left, turn_right)

            elif isinstance(node, Goto):
                pose = robot.localization.get_pose()
                dx = node.x_m - pose.position[0]
                dy = node.y_m - pose.position[1]
                distance_m = math.hypot(dx, dy)

                # Heading: explicit lock if given, otherwise face the target.
                if node.theta_rad is not None:
                    target_heading = node.theta_rad
                else:
                    # If we're already on top of the target, don't rotate
                    # to a meaningless atan2(0, 0) heading.
                    target_heading = math.atan2(dy, dx) if distance_m > 1e-6 else pose.heading

                turn_delta = _wrap_angle(target_heading - pose.heading)
                await self._dispatch_turn(robot, turn_delta, turn_left, turn_right)

                if distance_m > 1e-6:
                    await drive_forward(cm=distance_m * 100.0).run_step(robot)

            elif isinstance(node, Resync):
                msg = "Resync nodes are wired up in Phase 5 (resync steps)."
                raise NotImplementedError(msg)

            elif isinstance(node, Action):
                if node.blocking:
                    await node.step.run_step(robot)
                else:
                    msg = (
                        "Action(blocking=False) is Phase-4 work — non-blocking "
                        "side-action scheduling lands with absolute-aware motion."
                    )
                    raise NotImplementedError(msg)

            else:  # pragma: no cover — compile_plan validates node types.
                msg = f"AbsolutePathExecutor: unknown node type {type(node).__name__!r}"
                raise TypeError(msg)

    @staticmethod
    async def _dispatch_turn(robot, delta_rad, turn_left, turn_right) -> None:
        """Issue ``turn_left`` / ``turn_right`` for ``delta_rad`` (signed)."""
        if abs(delta_rad) < _TURN_TOL_RAD:
            return
        delta_deg = math.degrees(abs(delta_rad))
        if delta_rad > 0.0:
            await turn_left(degrees=delta_deg).run_step(robot)
        else:
            await turn_right(degrees=delta_deg).run_step(robot)


__all__ = ["AbsolutePathExecutor"]
