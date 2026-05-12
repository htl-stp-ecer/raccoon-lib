"""User-facing factories for the absolute-pose plan IR.

Mission authors call these helpers (centimetres + degrees) instead of
constructing :mod:`abs_ir` dataclasses directly (metres + radians). The
factories handle unit conversion and nothing else — they return raw IR
nodes, ready to drop into a plan list.

Example::

    from raccoon.step.motion.path import goto, turn_to, resync, action

    plan = [
        resync("io_button", expected_x_cm=10, expected_y_cm=10, expected_theta_deg=0),
        goto(x_cm=80, y_cm=10, via="forward"),
        turn_to(theta_deg=90),
        goto(x_cm=80, y_cm=100, speed_scale=0.5),
    ]
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Literal

from .abs_ir import Action, Goto, Resync, TurnTo

if TYPE_CHECKING:
    from ... import Step


def goto(
    x_cm: float,
    y_cm: float,
    *,
    theta_deg: float | None = None,
    via: Literal["forward", "lateral", "arc", "auto"] = "auto",
    speed_scale: float = 1.0,
) -> Goto:
    """Build a :class:`Goto` node from user-friendly units.

    Drives to an absolute world-frame target. Heading is optional — leave
    ``theta_deg=None`` to let the executor infer it from the path tangent
    or previous node.

    Args:
        x_cm: Target X in centimetres (world frame).
        y_cm: Target Y in centimetres (world frame).
        theta_deg: Optional heading lock in degrees. ``None`` ⇒ unspecified.
        via: Motion primitive selector — ``"forward"``, ``"lateral"``,
            ``"arc"``, or ``"auto"`` (default; compiler picks).
        speed_scale: Velocity multiplier in ``[0, 1]``. ``1.0`` = full speed.

    Returns:
        A frozen :class:`Goto` IR node with metres / radians internally.
    """
    return Goto(
        x_m=x_cm / 100.0,
        y_m=y_cm / 100.0,
        theta_rad=None if theta_deg is None else math.radians(theta_deg),
        via=via,
        speed_scale=speed_scale,
    )


def turn_to(theta_deg: float) -> TurnTo:
    """Build a :class:`TurnTo` node from degrees.

    Args:
        theta_deg: Target heading in degrees (world frame).

    Returns:
        A frozen :class:`TurnTo` IR node with the heading in radians.
    """
    return TurnTo(theta_rad=math.radians(theta_deg))


def resync(
    method: Literal["wall_align", "find_line", "marker", "io_button"],
    *,
    expected_x_cm: float | None = None,
    expected_y_cm: float | None = None,
    expected_theta_deg: float | None = None,
    snap_axes: tuple[bool, bool, bool] = (True, True, True),
) -> Resync:
    """Build a :class:`Resync` node from user-friendly units.

    Args:
        method: How the observation is acquired — ``"wall_align"``,
            ``"find_line"``, ``"marker"``, or ``"io_button"``.
        expected_x_cm: Optional prior on world-X in centimetres.
        expected_y_cm: Optional prior on world-Y in centimetres.
        expected_theta_deg: Optional prior on heading in degrees.
        snap_axes: Per-axis snap mask ``(x, y, theta)``. ``True`` along an
            axis means the observation is treated as tight along that axis.

    Returns:
        A frozen :class:`Resync` IR node with metres / radians internally.
    """
    return Resync(
        method=method,
        expected_x_m=None if expected_x_cm is None else expected_x_cm / 100.0,
        expected_y_m=None if expected_y_cm is None else expected_y_cm / 100.0,
        expected_theta_rad=(
            None if expected_theta_deg is None else math.radians(expected_theta_deg)
        ),
        snap_axes=snap_axes,
    )


def action(step: "Step", *, blocking: bool = True) -> Action:
    """Wrap an arbitrary Step into an :class:`Action` plan node.

    Args:
        step: The Step to embed in the plan.
        blocking: ``True`` (default) blocks the plan until the step
            finishes. ``False`` dispatches the step in the background so
            it overlaps with the next motion segment.

    Returns:
        A frozen :class:`Action` IR node.
    """
    return Action(step=step, blocking=blocking)


__all__ = [
    "goto",
    "turn_to",
    "resync",
    "action",
]
