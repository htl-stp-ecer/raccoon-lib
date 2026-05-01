"""Absolute-pose plan IR for the motion path pipeline.

This module defines the **absolute** plan IR introduced in Phase 3 of the
``absolute-motion-plan`` migration. Unlike the existing relative IR in
``ir.py`` (segments + side-actions, expressed as deltas), the absolute IR
expresses a mission as a flat list of ``AbsoluteNode`` instances whose
geometry is anchored in the world frame.

Internal storage convention
---------------------------

All positional fields on the IR dataclasses are stored in **SI units**
(metres, radians). The user-facing factory layer in :mod:`abs_factory`
accepts centimetres / degrees and converts on the way in. Anything that
constructs nodes directly (compiler tests, snapshot fixtures) must hand in
SI values.

Why Python
----------

The original design sketch placed desugaring + optimizer in C++ with
pybind11 bindings. We deliberately keep Phase 3 in Python instead:
``compile_plan`` runs once at mission start (it is *not* on the control
hot-path), and the ``Action`` node wraps a Python ``Step`` instance. A C++
IR holding ``py::object`` would add binding complexity without buying any
performance. See ``docs/design/absolute-motion-plan.md`` "Implementation
Notes" for the full rationale.

Stability
---------

Every node is a frozen dataclass. Compiler passes return new lists, never
mutate in place — same contract as the relative IR.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

if TYPE_CHECKING:
    # Path: step/motion/path/abs_ir.py — `...` walks up to step/.
    from ... import Step


@dataclass(frozen=True)
class Goto:
    """Drive to an absolute world-frame target pose.

    Attributes:
        x_m: Target X in metres (world frame).
        y_m: Target Y in metres (world frame).
        theta_rad: Heading lock in radians, or ``None`` to let the executor
            infer the heading from the path tangent / previous node.
        via: Motion primitive selector — ``"forward"`` (longitudinal),
            ``"lateral"`` (strafe), ``"arc"``, or ``"auto"`` (compiler picks).
        speed_scale: Multiplier on the segment's nominal velocity. ``1.0``
            is full speed; lower values throttle for precision sections.
    """

    x_m: float
    y_m: float
    theta_rad: float | None = None
    via: Literal["forward", "lateral", "arc", "auto"] = "auto"
    speed_scale: float = 1.0


@dataclass(frozen=True)
class TurnTo:
    """Rotate in place to an absolute world-frame heading.

    Attributes:
        theta_rad: Target heading in radians (world frame).
    """

    theta_rad: float


@dataclass(frozen=True)
class Resync:
    """Inject a high-confidence observation into the localization filter.

    A ``Resync`` node does **not** hard-set the world pose. It hands the
    localization service an observation with tight per-axis variance,
    letting the filter integrate it like any other measurement. The
    ``snap_axes`` triple controls which of ``(x, y, theta)`` get the tight
    variance; the others stay loose and are kept by the filter.

    Attributes:
        method: How the observation is acquired — ``"wall_align"``,
            ``"find_line"``, ``"marker"``, or ``"io_button"``.
        expected_x_m: Optional prior on world-X in metres.
        expected_y_m: Optional prior on world-Y in metres.
        expected_theta_rad: Optional prior on heading in radians.
        snap_axes: Per-axis snap mask ``(x, y, theta)``. ``True`` ⇒ tight
            variance along that axis; ``False`` ⇒ leave to the filter.
    """

    method: Literal["wall_align", "find_line", "marker", "io_button"]
    expected_x_m: float | None = None
    expected_y_m: float | None = None
    expected_theta_rad: float | None = None
    snap_axes: tuple[bool, bool, bool] = (True, True, True)


@dataclass(frozen=True)
class Action:
    """A non-motion step pinned into the absolute plan.

    Wraps an arbitrary :class:`raccoon.step.base.Step` so servos, waits,
    sensor reads, and other side-effects can sit alongside motion nodes in
    the same IR.

    Attributes:
        step: The wrapped Step instance.
        blocking: ``True`` ⇒ the plan awaits the step before advancing.
            ``False`` ⇒ the step is dispatched in the background and runs
            in parallel with the next motion segment.
    """

    step: "Step"
    blocking: bool = True


# Tagged union of all node kinds in an absolute plan. Compiler / executor
# code should narrow on this alias rather than the individual dataclasses.
AbsoluteNode = Goto | TurnTo | Resync | Action


__all__ = [
    "Goto",
    "TurnTo",
    "Resync",
    "Action",
    "AbsoluteNode",
]
