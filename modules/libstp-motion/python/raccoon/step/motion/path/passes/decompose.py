"""``decompose`` pass — split an ``after_cm + sensor`` leg into known + rest.

A conditional segment whose stop condition is a sequential ``_Then`` chain LED
by a *bare relative* ``after_cm`` covers a KNOWN travel distance first, then an
unknown remainder.  ``drive_forward().until(after_cm(12) + over_line(sensor))``
drives a fixed 12 cm and THEN keeps driving until it crosses a line — the first
12 cm are geometrically known, the rest is not.

The raw lowering can't recover that known distance: ``recover_known_distance``
only promotes a *bare* ``after_cm`` (an ``after_cm + over_line`` is a ``_Then``
wrapper, which it intentionally rejects because the combined condition may stop
early — see ``known_distance.py``).  So the whole leg stays
``distance_m=None, has_known_endpoint=False`` and is invisible to the geometry
passes (merge / to_absolute / splinify).

``a + b`` builds ``_Then(a, b)`` and ``+`` is LEFT-associative, so a chain
``after_cm(20) + after_cm(10) + over_line`` is the LEFT-nested tree
``_Then(_Then(after_cm(20), after_cm(10)), over_line)`` — the LEADING condition
is the leftmost leaf, reached by descending ``._first``.  This pass flattens the
``_Then`` tree into its ordered leaf sequence, then peels each LEADING bare
relative ``after_cm`` off the front.

It splits such a leg in TIME into segments of the SAME drive (identical
kind/axis/sign/speed_scale/heading):

1. For each leading bare relative ``after_cm`` leaf, emit a ``seg_known`` —
   condition = that ``after_cm``, run through ``recover_known_distance`` so
   ``distance_m`` / ``has_known_endpoint`` get filled.  This leg is now a
   known-endpoint leg the downstream passes can absolutize / splinify.
2. The REMAINING leaves (from the first non-``after_cm`` leaf onward) are
   rebuilt into a left-associated ``_Then`` (or a single bare condition when one
   leaf remains) for a final ``seg_rest`` with ``distance_m=None``,
   ``has_known_endpoint=False``.

Each peeled ``after_cm`` becomes a REAL known distance because the executor calls
``condition.start()`` at every segment start, so each ``after_cm`` measures from
its own segment's start.  ``after_cm(a) + after_cm(b) + over_line`` →
``[a-known, b-known, over_line-unknown]``.  Peeling stops at the first leaf that
isn't a bare relative ``after_cm``, leaving the rest as the final unknown leg.

Guards (passed through UNCHANGED):

- A *bare* ``after_cm`` (not wrapped in ``_Then``) — already promoted at lowering
  by ``recover_known_distance``; left alone.
- A ``_Then`` NOT led by ``after_cm`` (e.g. ``over_line`` = ``on_black > on_white``,
  whose ``_first`` is a sensor) — may stop early; left whole.
- An *absolute* ``after_cm`` lead — measures from odometry origin, not segment
  start; left whole.
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
    """Split one segment along a ``_Then`` chain led by bare relative ``after_cm``.

    Returns ``[seg]`` unchanged when ``seg`` is not a decomposable
    ``linear`` / ``follow_line`` whose condition is a ``_Then`` whose LEADING
    (leftmost) leaf is a bare relative ``after_cm``.  Otherwise peels each
    leading bare-relative ``after_cm`` leaf into a known-endpoint leg and leaves
    the rest of the chain as one final unknown-endpoint leg.
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

    # Only decompose when the LEADING leaf is a known distance.  over_line
    # (led by on_black) and an absolute-after_cm lead pass through whole.
    if not _is_bare_relative_after_cm(leaves[0]):
        return [seg]

    result: list[Segment] = []
    idx = 0
    while idx < len(leaves) and _is_bare_relative_after_cm(leaves[idx]):
        # Known leg: same drive, stopped by this bare after_cm; recover its
        # distance into distance_m / has_known_endpoint.  Each after_cm measures
        # from its own segment start (executor restarts the condition there).
        result.append(recover_known_distance(replace(seg, condition=leaves[idx])))
        idx += 1

    # The remaining leaves form the final unknown-endpoint leg.  If every leaf
    # was a known after_cm (a pure after_cm + after_cm chain), there is no
    # remainder — the whole chain decomposed into known legs.
    if idx < len(leaves):
        rest_cond = _then_chain(leaves[idx:])
        result.append(
            replace(
                seg,
                condition=rest_cond,
                distance_m=None,
                has_known_endpoint=False,
            )
        )
    return result


class DecomposePass:
    """Split ``after_cm + sensor`` legs into a known leg + the remaining leg.

    Pure node→node pass.  For each ``linear`` / ``follow_line`` ``Segment``
    whose condition is a sequential ``_Then`` led by a bare relative
    ``after_cm``, splits it into a known-endpoint leg (the ``after_cm``
    distance, recovered) plus the remaining-condition leg, recursing through a
    chain of leading ``after_cm`` conditions.  Non-decomposable nodes pass
    through unchanged.

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
