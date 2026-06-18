"""Known-distance pass — recover travel distance hidden in ``after_cm``.

A ``.until(after_cm(N))`` stop condition stores a KNOWN travel distance
(``after_cm._target_m``), but lowering marks the resulting segment as
``distance_m=None, has_known_endpoint=False`` because it never inspects the
condition.  Geometry passes (merge / cut_corners / to_absolute) therefore
treat such segments as unoptimizable even though they cover a fixed distance.

This pass promotes those segments: for every conditional ``linear`` /
``follow_line`` segment whose condition is a *bare* relative-mode ``after_cm``,
it fills in ``distance_m = sign * target_m`` and sets
``has_known_endpoint = True``.  The condition is left in place — it remains the
canonical runtime odometer stop, and for a single-axis move it is equivalent to
the promoted distance because ``after_cm`` measures path length.

Correctness guards (no promotion):

- **Combined conditions** (``after_cm + over_line``, ``cond_a | cond_b``, …)
  may stop EARLY, so the endpoint is not known.  These are *not* ``after_cm``
  instances — they are ``_Then`` / ``_AnyOf`` / ``_AllOf`` wrappers produced by
  the ``+`` / ``>`` / ``|`` / ``&`` combinators (see ``condition.py``).  Only a
  bare ``isinstance(cond, after_cm)`` is promoted.
- **Absolute mode** (``after_cm(N, absolute=True)``): the distance is measured
  from the odometry origin, not the segment start — not a segment-local
  travel distance.  Skipped.
- **Non-linear kinds** (``turn`` / ``arc``): path-length distance does not map
  to a usable travel distance.  Skipped.
- Segments that already have a known endpoint, ``SideAction``s, and ``None``
  placeholders are left untouched.
"""

from __future__ import annotations

from dataclasses import replace

from ..ir import PathNode, Segment

_PROMOTABLE_KINDS = ("linear", "follow_line")


def _promote(seg: Segment) -> Segment:
    """Return a promoted copy of ``seg`` if eligible, else ``seg`` unchanged."""
    # Already known, or no condition to recover a distance from.
    if seg.has_known_endpoint or seg.condition is None:
        return seg

    # Only path-length kinds map cleanly to a travel distance.
    if seg.kind not in _PROMOTABLE_KINDS:
        return seg

    # Import here to avoid a module-level dependency from the passes package
    # into the step.condition module.
    from ....condition import after_cm

    cond = seg.condition

    # A *bare* after_cm is a known endpoint.  Combined conditions
    # (_Then / _AnyOf / _AllOf from + / > / | / &) are different classes and
    # are intentionally rejected — they may fire early.
    if not isinstance(cond, after_cm):
        return seg

    # Absolute mode measures from the odometry origin, not the segment start.
    if cond._absolute:
        return seg

    target_m = cond._target_m
    return replace(
        seg,
        distance_m=seg.sign * target_m,
        has_known_endpoint=True,
    )


def run_known_distance(nodes: list[PathNode | None]) -> list[PathNode | None]:
    """Promote bare relative ``after_cm`` segments to known-endpoint segments."""
    result: list[PathNode | None] = []
    for node in nodes:
        if isinstance(node, Segment):
            result.append(_promote(node))
        else:
            result.append(node)
    return result


class KnownDistancePass:
    """Compiler pass that recovers known distances from ``after_cm`` conditions."""

    name = "known_distance"

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        return run_known_distance(nodes)
