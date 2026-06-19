"""``decompose`` pass — split a ``_Then`` leg at every known ``after_cm`` boundary.

A conditional segment whose stop condition is a sequential ``_Then`` chain that
contains a *bare relative* ``after_cm`` leaf covers a KNOWN travel distance over
that leaf, surrounded by unknown sensor-driven portions.
``drive_forward().until(after_cm(12) + over_line(sensor))`` drives a fixed 12 cm
and THEN keeps driving until it crosses a line — the first 12 cm are
geometrically known, the rest is not.  ``...until(over_line(sensor) + after_cm(5))``
drives until the line, then a known 5 cm further.

The raw lowering can't recover that known distance: ``recover_known_distance``
only promotes a *bare* ``after_cm`` (any combined ``_Then`` wrapper is rejected
because the combined condition may stop early — see ``known_distance.py``).  So
the whole leg stays ``distance_m=None, has_known_endpoint=False`` and is
invisible to the geometry passes (merge / to_absolute / splinify).

``a + b`` builds ``_Then(a, b)`` and ``+`` is LEFT-associative, so a chain
``after_cm(20) + after_cm(10) + over_line`` is the LEFT-nested tree
``_Then(_Then(after_cm(20), after_cm(10)), over_line)``.  This pass flattens the
``_Then`` tree into its ordered leaf sequence, then walks the leaves
left→right, splitting at EVERY bare-relative-``after_cm`` boundary — leading,
trailing, or interleaved.

It splits such a leg in TIME into segments of the SAME drive (identical
kind/axis/sign/speed_scale/heading), one segment per GROUP:

1. Each bare relative ``after_cm`` leaf becomes its OWN known leg — condition =
   that ``after_cm``, run through ``recover_known_distance`` so ``distance_m`` /
   ``has_known_endpoint`` get filled.  This leg is now a known-endpoint leg the
   downstream passes can absolutize / splinify.
2. Consecutive NON-(bare-relative-``after_cm``) leaves are GROUPED into one
   unknown sensor leg — their order-preserving ``_Then`` rebuilt (so
   ``over_line``'s ``on_black + on_white`` stay together as ONE leg, not split),
   with ``distance_m=None``, ``has_known_endpoint=False``.

Each split-out ``after_cm`` becomes a REAL known distance because the executor
calls ``condition.start()`` at every segment start, so each ``after_cm``
measures from its own segment's start.  Examples (sensor = on_black/over_line):

- ``after_cm(12) + over_line`` → ``[known(0.12), sensor]`` (leading).
- ``on_black + after_cm(5)`` → ``[sensor, known(0.05)]`` (trailing).
- ``on_black + after_cm + on_black + after_cm`` → ``[sensor, known, sensor, known]``.
- ``over_line + after_cm(5)`` → ``[sensor(_Then(on_black, on_white)), known(0.05)]``.
- pure ``over_line`` (no after_cm) → unchanged ``[seg]``.

Guards (passed through UNCHANGED):

- A *bare* ``after_cm`` (not wrapped in ``_Then``) — already promoted at lowering
  by ``recover_known_distance``; left alone.
- A ``_Then`` with NO bare-relative ``after_cm`` anywhere (e.g. ``over_line`` =
  ``on_black > on_white``) — may stop early; left whole, not fragmented.
- An *absolute* ``after_cm`` leaf — measures from odometry origin, not segment
  start; treated as a sensor leaf (grouped, never promoted to a known leg).
- ``turn`` / ``arc`` kinds — path length doesn't map to a usable travel distance.
- ``SideAction`` / ``None`` deferred placeholders — pass through untouched.

Representation declaration is intentionally left undeclared (defaults to
``EITHER`` / ``SAME`` / non-terminal); run this BEFORE ``to_absolute`` /
``splinify`` so the split-out known leg can be optimized.
"""

from __future__ import annotations

from dataclasses import replace

from ..ir import PathNode, Segment
from .known_distance import recover_known_distance

_DECOMPOSABLE_KINDS = ("linear", "follow_line")


def _then_flatten(cond) -> list:
    """Flatten a (possibly nested) ``_Then`` tree into ordered leaf conditions.

    ``+`` is left-associative, so ``a + b + c`` is ``_Then(_Then(a, b), c)``;
    this returns ``[a, b, c]`` in execution order.  A non-``_Then`` condition
    returns ``[cond]`` (a single leaf).
    """
    from ....condition import _Then  # deferred to avoid import cycle

    if not isinstance(cond, _Then):
        return [cond]
    return _then_flatten(cond._first) + _then_flatten(cond._second)


def _is_bare_relative_after_cm(cond) -> bool:
    """Is ``cond`` a bare relative-mode ``after_cm`` (a known travel distance)?"""
    from ....condition import after_cm  # deferred to avoid import cycle

    return isinstance(cond, after_cm) and not cond._absolute


def _then_chain(leaves: list):
    """Rebuild a left-associated ``_Then`` from an ordered leaf list.

    A single leaf is returned bare; two or more are folded left-to-right into
    ``_Then(_Then(a, b), c)`` — mirroring how ``a + b + c`` was built.
    """
    from ....condition import _Then  # deferred to avoid import cycle

    chain = leaves[0]
    for leaf in leaves[1:]:
        chain = _Then(chain, leaf)
    return chain


def _decompose_segment(seg: Segment) -> list[Segment]:
    """Split one segment along a ``_Then`` chain at EVERY bare ``after_cm`` boundary.

    Returns ``[seg]`` unchanged when ``seg`` is not a decomposable
    ``linear`` / ``follow_line`` whose condition is a ``_Then``, or when that
    ``_Then`` chain contains NO bare relative ``after_cm`` leaf (a pure-sensor
    chain like ``over_line`` = ``_Then(on_black, on_white)``).

    Otherwise the leaves are walked left→right and emitted one segment per
    GROUP:

    - Each bare relative ``after_cm`` leaf becomes its OWN known-endpoint leg
      (``recover_known_distance`` fills ``distance_m`` / ``has_known_endpoint``).
      This works wherever the leaf sits — leading, trailing, or interleaved —
      because the executor calls ``condition.start()`` at each segment start, so
      a trailing ``after_cm(N)`` measures N from where the preceding leg ended,
      a real known distance once split off.
    - Consecutive NON-(bare-relative-``after_cm``) leaves are GROUPED into one
      unknown-endpoint sensor leg, with their order-preserving ``_Then`` rebuilt
      (so ``over_line``'s ``on_black + on_white`` stay together as one leg).
    """
    if seg.kind not in _DECOMPOSABLE_KINDS:
        return [seg]
    if seg.condition is None:
        return [seg]

    from ....condition import _Then  # deferred to avoid import cycle

    # A bare after_cm is already handled by known_distance at lowering; only a
    # _Then chain is decomposable here.
    if not isinstance(seg.condition, _Then):
        return [seg]

    leaves = _then_flatten(seg.condition)

    # Nothing to extract from a pure-sensor chain (no known distance anywhere);
    # leave it whole rather than fragmenting e.g. over_line into two legs.
    if not any(_is_bare_relative_after_cm(leaf) for leaf in leaves):
        return [seg]

    result: list[Segment] = []
    sensor_group: list = []

    def _flush_sensor_group() -> None:
        # Emit the accumulated consecutive non-after_cm leaves as one unknown
        # sensor leg, preserving their relative order.
        if not sensor_group:
            return
        result.append(
            replace(
                seg,
                condition=_then_chain(list(sensor_group)),
                distance_m=None,
                has_known_endpoint=False,
            )
        )
        sensor_group.clear()

    for leaf in leaves:
        if _is_bare_relative_after_cm(leaf):
            # Boundary: close any open sensor group, then emit this known leg.
            # Each after_cm measures from its own segment start (executor
            # restarts the condition there).
            _flush_sensor_group()
            result.append(recover_known_distance(replace(seg, condition=leaf)))
        else:
            sensor_group.append(leaf)

    # Trailing non-after_cm leaves form a final sensor leg.
    _flush_sensor_group()
    return result


class DecomposePass:
    """Split a ``_Then`` leg at every bare ``after_cm`` boundary.

    Pure node→node pass.  For each ``linear`` / ``follow_line`` ``Segment``
    whose condition is a sequential ``_Then`` containing a bare relative
    ``after_cm`` leaf, splits it into one known-endpoint leg per ``after_cm``
    leaf (distance recovered) and one grouped unknown sensor leg per run of
    consecutive non-``after_cm`` leaves — leading, trailing, and interleaved.
    A ``_Then`` with no bare ``after_cm`` anywhere, and all non-decomposable
    nodes, pass through unchanged.

    Representation/terminal contract left undeclared (defaults to
    ``EITHER`` / ``SAME`` / non-terminal).
    """

    name = "decompose"

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        result: list[PathNode | None] = []
        for node in nodes:
            if isinstance(node, Segment):
                result.extend(_decompose_segment(node))
            else:
                result.append(node)
        return result
