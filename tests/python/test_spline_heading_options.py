"""Argument validation of the ``spline()`` heading options."""

from __future__ import annotations

import importlib.util

import pytest

requires_libstp = pytest.mark.skipif(
    importlib.util.find_spec("raccoon") is None,
    reason="raccoon native module not installed",
)

TANGENT = ((30, 0), (50, 20))
EXPLICIT = ((30, 0, 0), (50, 20, 45))


@requires_libstp
@pytest.mark.parametrize(
    ("waypoints", "options", "message"),
    [
        (TANGENT, {"absolute_heading": True}, "requires final_heading"),
        (EXPLICIT, {"absolute_heading": True}, "requires final_heading"),
        (EXPLICIT, {"final_heading": 0}, "cannot combine final_heading"),
        (EXPLICIT, {"absolute_heading": True, "final_heading": 0}, "cannot combine final_heading"),
    ],
)
def test_rejects_heading_options_without_effect(waypoints, options, message):
    from raccoon.step.motion.spline_path import spline

    with pytest.raises(ValueError, match=message):
        spline(*waypoints, **options)


@requires_libstp
@pytest.mark.parametrize(
    ("waypoints", "options"),
    [
        (TANGENT, {}),
        (EXPLICIT, {}),
        (TANGENT, {"final_heading": 90}),
        (TANGENT, {"absolute_heading": True, "final_heading": 0}),
    ],
)
def test_accepts_effective_heading_options(waypoints, options):
    from raccoon.step.motion.spline_path import spline

    spline(*waypoints, **options)
