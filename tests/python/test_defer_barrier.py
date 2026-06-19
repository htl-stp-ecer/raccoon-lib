"""Unit tests for the Defer hard-barrier contract in the lowering pass.

A ``Defer`` step is an opaque barrier: it cannot be planned at compile time,
so the lowering pass must emit a ``None`` placeholder for it and record an
entry in ``deferred`` for runtime resolution.

The public ``defer(...)`` factory returns a ``DeferBuilder`` (not a raw
``Defer``). These tests guard that the builder lowers identically to a raw
``Defer`` — i.e. the lowering pass resolves builders *before* its isinstance
checks, so ``defer()`` is not mistakenly pinned as an inline ``SideAction``.
"""

from __future__ import annotations

import importlib.util

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


# ---------------------------------------------------------------------------
# defer() builder lowers to a None placeholder + deferred entry
# ---------------------------------------------------------------------------


@requires_libstp
def test_defer_builder_lowers_to_none_placeholder():
    from raccoon.step import defer, seq
    from raccoon.step.logic.defer import Defer
    from raccoon.step.motion.path.passes.lowering import flatten_steps

    nodes, deferred = flatten_steps([defer(lambda robot: seq([]))])

    assert nodes == [None]
    assert len(deferred) == 1
    index, defer_step = deferred[0]
    assert index == 0
    assert isinstance(defer_step, Defer)


# ---------------------------------------------------------------------------
# A defer between two drives is a barrier (None) at the correct index
# ---------------------------------------------------------------------------


@requires_libstp
def test_defer_between_drives_is_barrier():
    from raccoon.step import defer, seq
    from raccoon.step.logic.defer import Defer
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import flatten_steps

    nodes, deferred = flatten_steps(
        [
            drive_forward(30),
            defer(lambda robot: seq([])),
            drive_forward(20),
        ]
    )

    assert isinstance(nodes[0], Segment) and nodes[0].kind == "linear"
    assert nodes[1] is None
    assert isinstance(nodes[2], Segment) and nodes[2].kind == "linear"

    assert len(deferred) == 1
    index, defer_step = deferred[0]
    assert index == 1
    assert isinstance(defer_step, Defer)


# ---------------------------------------------------------------------------
# Regression: a RAW Defer and the builder lower identically
# ---------------------------------------------------------------------------


@requires_libstp
def test_raw_defer_and_builder_lower_identically():
    from raccoon.step import defer, seq
    from raccoon.step.logic.defer import Defer
    from raccoon.step.motion.path.passes.lowering import flatten_steps

    raw_nodes, raw_deferred = flatten_steps([Defer(factory=lambda robot: seq([]))])
    builder_nodes, builder_deferred = flatten_steps([defer(lambda robot: seq([]))])

    # Both yield exactly one None placeholder and one deferred entry at index 0.
    assert raw_nodes == [None]
    assert builder_nodes == [None]

    assert len(raw_deferred) == 1
    assert len(builder_deferred) == 1
    assert raw_deferred[0][0] == 0
    assert builder_deferred[0][0] == 0
    assert isinstance(raw_deferred[0][1], Defer)
    assert isinstance(builder_deferred[0][1], Defer)
