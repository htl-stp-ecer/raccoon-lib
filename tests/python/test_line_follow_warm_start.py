"""Warm-start type-compatibility for line-follow segments.

Regression test for the M040 divergence: a hold-heading line-follow must NOT
warm-start straight into a free-heading one (correct_lateral(hold_heading=
False)). They share the follow_line kind, so the old is_same_type() treated
them as interchangeable and skipped the hard stop between them — the
free-heading leg then swung the chassis up to 90° while still carrying the
previous leg's forward velocity, throwing every downstream sensor strafe off.
"""

from __future__ import annotations

import importlib.util
import types

import pytest


def _has_raccoon() -> bool:
    return importlib.util.find_spec("raccoon") is not None


pytestmark = pytest.mark.skipif(not _has_raccoon(), reason="raccoon not importable")


def _follow_segment(heading_hold: bool):
    """A follow_line Segment whose opaque step reports the given heading_hold."""
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.path.ir import Segment

    opaque = types.SimpleNamespace(config=types.SimpleNamespace(heading_hold=heading_hold))
    return Segment(kind="follow_line", axis=LinearAxis.Forward, opaque_step=opaque)


def test_same_heading_hold_follows_warm_start():
    from raccoon.step.motion.path.passes.lowering import is_same_type

    assert is_same_type(_follow_segment(True), _follow_segment(True))
    assert is_same_type(_follow_segment(False), _follow_segment(False))


def test_mixed_heading_hold_follows_cold_start():
    from raccoon.step.motion.path.passes.lowering import is_same_type

    # hold → free and free → hold must both refuse warm-start.
    assert not is_same_type(_follow_segment(True), _follow_segment(False))
    assert not is_same_type(_follow_segment(False), _follow_segment(True))


def test_follow_without_config_defaults_to_hold():
    """A follow_line whose opaque step exposes no config defaults to hold=True,
    so it stays warm-start-compatible with other hold-heading follows."""
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.path.ir import Segment
    from raccoon.step.motion.path.passes.lowering import is_same_type

    bare = Segment(kind="follow_line", axis=LinearAxis.Forward, opaque_step=object())
    assert is_same_type(bare, _follow_segment(True))
