"""Tests for ``_with_heading_offset`` — the executor's path-start H0 offset.

``AbsoluteHeadingPass`` stamps ``target_heading_rad`` in the PATH-START frame
(start = 0). The executor consumes it as an ABSOLUTE raw-odometry world heading,
so it must add the path-start world heading H0 to every stamped value. This
verifies that helper in isolation.
"""

from __future__ import annotations

import math

from raccoon.motion import LinearAxis
from raccoon.step.motion.path.executor import _with_heading_offset
from raccoon.step.motion.path.ir import Segment


def test_offset_added_to_target_heading():
    theta = 0.5
    h0 = 1.25
    seg = Segment(
        kind="linear",
        axis=LinearAxis.Forward,
        sign=1.0,
        distance_m=0.5,
        target_heading_rad=theta,
        has_known_endpoint=True,
    )
    out = _with_heading_offset(seg, h0)
    assert math.isclose(out.target_heading_rad, theta + h0, abs_tol=1e-12)
    # Original is not mutated (pass output is reused across runs / explain).
    assert math.isclose(seg.target_heading_rad, theta, abs_tol=1e-12)
    assert out is not seg


def test_none_target_heading_returned_unchanged():
    seg = Segment(
        kind="linear",
        axis=LinearAxis.Forward,
        sign=1.0,
        distance_m=0.5,
        target_heading_rad=None,
        has_known_endpoint=True,
    )
    out = _with_heading_offset(seg, 1.25)
    # No-op for the common case: same object returned, still None.
    assert out is seg
    assert out.target_heading_rad is None


def test_zero_offset_preserves_value():
    seg = Segment(
        kind="linear",
        axis=LinearAxis.Forward,
        sign=1.0,
        distance_m=0.5,
        target_heading_rad=-math.pi / 2,
        has_known_endpoint=True,
    )
    out = _with_heading_offset(seg, 0.0)
    assert math.isclose(out.target_heading_rad, -math.pi / 2, abs_tol=1e-12)
