"""Recover travel distance hidden in an ``after_cm`` stop condition.

A ``.until(after_cm(N))`` stop condition stores a KNOWN travel distance
(``after_cm._target_m``), but the raw lowering produces ``distance_m=None,
has_known_endpoint=False`` because the segment factory never inspects the
condition.  Geometry passes (merge / cut_corners / to_absolute) would then
treat such a segment as unoptimizable even though it covers a fixed distance.

This recovery is applied **unconditionally at lowering time** (see
``extract_segment``): there is no reason a known distance should ever be hidden
from the optimizer, so it is not an opt-in pass.  For every conditional
``linear`` / ``follow_line`` segment whose condition is a *bare* relative-mode
``after_cm``, it fills in ``distance_m = sign * target_m`` and sets
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
  from the odometry origin, not the segment start — skipped.
- **Non-linear kinds** (``turn`` / ``arc``): path-length distance does not map
  to a usable travel distance — skipped.
- Segments that already have a known endpoint are left untouched.
"""

from __future__ import annotations

from dataclasses import replace

from ..ir import Segment

_PROMOTABLE_KINDS = ("linear", "follow_line")


def recover_known_distance(seg: Segment) -> Segment:
    """Return a copy of ``seg`` with a recovered ``after_cm`` distance, else ``seg``.

    Eligible only when ``seg`` is a conditional ``linear`` / ``follow_line``
    whose condition is a bare relative-mode ``after_cm``.  Applied at lowering
    time to every segment, so the optimizer never sees a hidden known distance.
    """
    # Already known, or no condition to recover a distance from.
    if seg.has_known_endpoint or seg.condition is None:
        return seg

    # Only path-length kinds map cleanly to a travel distance.
    if seg.kind not in _PROMOTABLE_KINDS:
        return seg

    # Imported here to avoid a module-level dependency from the passes package
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

    return replace(
        seg,
        distance_m=seg.sign * cond._target_m,
        has_known_endpoint=True,
    )
