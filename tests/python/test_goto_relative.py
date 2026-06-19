"""Unit tests for ``goto_relative`` and its pure ``_anchor_to_world_target``.

``_anchor_to_world_target`` composes a body-frame delta onto an anchor pose to
produce an absolute world target.  It is pure (no robot) and tested with a tiny
pose-like duck type.  A construction/signature test (guarded by
``requires_libstp``) exercises the public ``goto_relative()`` factory.

Conventions under test (matching ``goto`` / the HAL ``ChassisVelocity`` and the
spline integration):
  - +forward is the anchor heading direction
  - +left is 90° CCW of that
  - +dtheta is counter-clockwise; target heading = anchor.heading + dtheta
"""

from __future__ import annotations

import importlib.util
import math

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


class FakePose:
    def __init__(self, x: float, y: float, heading: float) -> None:
        self.position = (x, y, 0.0)
        self.heading = heading


# goto.py imports native modules at module scope, so even the pure helper needs
# the raccoon package present.
pytestmark = requires_libstp


@pytest.fixture(scope="module")
def anchor_to_world_target():
    import importlib

    mod = importlib.import_module("raccoon.step.motion.goto")
    return mod._anchor_to_world_target


@pytest.fixture(scope="module")
def waypoints_to_world_targets():
    import importlib

    mod = importlib.import_module("raccoon.step.motion.goto")
    return mod._waypoints_to_world_targets


# ---------------------------------------------------------------------------
# Pure composition tests
# ---------------------------------------------------------------------------


def test_anchor_origin_forward_maps_to_plus_x(anchor_to_world_target):
    anchor = FakePose(0.0, 0.0, 0.0)
    x, y, theta = anchor_to_world_target(anchor, 1.0, 0.0, None)
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert theta is None


def test_anchor_origin_left_maps_to_plus_y(anchor_to_world_target):
    anchor = FakePose(0.0, 0.0, 0.0)
    x, y, theta = anchor_to_world_target(anchor, 0.0, 1.0, None)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(1.0)


def test_anchor_origin_right_maps_to_minus_y(anchor_to_world_target):
    anchor = FakePose(0.0, 0.0, 0.0)
    x, y, _ = anchor_to_world_target(anchor, 0.0, -1.0, None)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(-1.0)


def test_anchor_rotated_90_forward_maps_to_world_plus_y(anchor_to_world_target):
    # Anchor facing world +Y (heading 90°). A body-forward step lands on world +Y.
    anchor = FakePose(0.0, 0.0, math.radians(90.0))
    x, y, _ = anchor_to_world_target(anchor, 1.0, 0.0, None)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(1.0)


def test_anchor_rotated_90_left_maps_to_world_minus_x(anchor_to_world_target):
    # Anchor heading 90°; +left is world -X.
    anchor = FakePose(0.0, 0.0, math.radians(90.0))
    x, y, _ = anchor_to_world_target(anchor, 0.0, 1.0, None)
    assert x == pytest.approx(-1.0)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_offset_anchor_adds_to_position(anchor_to_world_target):
    anchor = FakePose(2.0, 3.0, 0.0)
    x, y, _ = anchor_to_world_target(anchor, 1.0, 0.5, None)
    assert x == pytest.approx(3.0)
    assert y == pytest.approx(3.5)


def test_dtheta_adds_to_anchor_heading(anchor_to_world_target):
    anchor = FakePose(0.0, 0.0, math.radians(30.0))
    _, _, theta = anchor_to_world_target(anchor, 1.0, 0.0, math.radians(45.0))
    assert theta == pytest.approx(math.radians(75.0))


def test_dtheta_none_leaves_heading_unset(anchor_to_world_target):
    anchor = FakePose(0.0, 0.0, math.radians(30.0))
    _, _, theta = anchor_to_world_target(anchor, 1.0, 0.0, None)
    assert theta is None


# ---------------------------------------------------------------------------
# Construction / factory test
# ---------------------------------------------------------------------------


@requires_libstp
def test_goto_relative_factory_builds_and_converts_units():
    from raccoon.step.motion import goto_relative

    step = goto_relative(50, left_cm=20, dtheta_deg=90)
    assert step._forward_m == pytest.approx(0.5)
    assert step._left_m == pytest.approx(0.2)
    assert step._dtheta_rad == pytest.approx(math.radians(90.0))
    assert step._pos_tol_m == pytest.approx(0.02)
    assert "localization" in step.required_resources()
    sig = step._generate_signature()
    assert sig.startswith("GotoRelative(")
    assert "fwd=0.50" in sig
    assert "left=0.20" in sig
    assert "90.0" in sig


@requires_libstp
def test_goto_relative_factory_no_dtheta_signature():
    from raccoon.step.motion import goto_relative

    step = goto_relative(30)
    assert step._left_m == pytest.approx(0.0)
    assert step._dtheta_rad is None
    assert "dtheta=None" in step._generate_signature()


@requires_libstp
def test_goto_relative_rejects_bad_speed():
    from raccoon.step.motion import goto_relative

    with pytest.raises(ValueError):
        goto_relative(30, speed=0.0)
    with pytest.raises(ValueError):
        goto_relative(30, speed=1.5)


# ---------------------------------------------------------------------------
# GotoWaypoints — pure _waypoints_to_world_targets composition
# ---------------------------------------------------------------------------


def test_waypoints_origin_forward_maps_to_plus_x(waypoints_to_world_targets):
    # Anchor at origin facing +X: a relative forward waypoint maps to +x world.
    anchor = FakePose(0.0, 0.0, 0.0)
    targets = waypoints_to_world_targets(anchor, [(1.0, 0.0, 0.0, 0.0, "rel", 0.0)])
    assert len(targets) == 1
    x, y, theta = targets[0]
    assert x == pytest.approx(1.0)
    assert y == pytest.approx(0.0, abs=1e-9)
    assert theta == pytest.approx(0.0, abs=1e-9)


def test_waypoints_rotated_90_forward_maps_to_plus_y(waypoints_to_world_targets):
    # Anchor rotated 90°: a relative forward waypoint maps to +y world.
    anchor = FakePose(0.0, 0.0, math.radians(90.0))
    targets = waypoints_to_world_targets(anchor, [(1.0, 0.0, 0.0, 0.0, "rel", 0.0)])
    x, y, _ = targets[0]
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(1.0)


def test_waypoints_absolute_leg_ignores_anchor_heading(waypoints_to_world_targets):
    # An ABSOLUTE leg's displacement is already in the localization frame (NOT
    # rotated by the anchor heading) and its heading is absolute — even with the
    # anchor rotated 90°, +0.3 abs_dx stays +0.3 in world X.
    anchor = FakePose(1.0, 2.0, math.radians(90.0))
    targets = waypoints_to_world_targets(anchor, [(0.0, 0.0, 0.3, 0.0, "abs", 0.0)])
    x, y, theta = targets[0]
    assert x == pytest.approx(1.3)
    assert y == pytest.approx(2.0, abs=1e-9)
    assert theta == pytest.approx(0.0, abs=1e-9)


def test_waypoints_all_share_one_anchor_not_chained(waypoints_to_world_targets):
    # Two relative waypoints, each expressed in the run-start frame (not chained).
    # The second (0.5 fwd, -0.3 left, -90°) resolves against the SAME origin
    # anchor — not relative to the first target.
    anchor = FakePose(0.0, 0.0, 0.0)
    targets = waypoints_to_world_targets(
        anchor,
        [(0.5, 0.0, 0.0, 0.0, "rel", 0.0), (0.5, -0.3, 0.0, 0.0, "rel", math.radians(-90.0))],
    )
    assert len(targets) == 2
    assert targets[0][0] == pytest.approx(0.5)
    assert targets[0][1] == pytest.approx(0.0, abs=1e-9)
    # Second: world (0.5 + 0.0, 0.0 + (-0.3 left rotated by 0 heading)) =
    # x = 0.5*1 - (-0.3)*0 = 0.5 ; y = 0.5*0 + (-0.3)*1 = -0.3.
    assert targets[1][0] == pytest.approx(0.5)
    assert targets[1][1] == pytest.approx(-0.3)
    assert targets[1][2] == pytest.approx(math.radians(-90.0))


def test_waypoints_offset_anchor_applies_to_each(waypoints_to_world_targets):
    anchor = FakePose(2.0, 3.0, 0.0)
    targets = waypoints_to_world_targets(
        anchor, [(1.0, 0.0, 0.0, 0.0, "rel", 0.0), (2.0, 0.0, 0.0, 0.0, "rel", 0.0)]
    )
    assert targets[0][0] == pytest.approx(3.0)
    assert targets[1][0] == pytest.approx(4.0)
    assert targets[0][1] == pytest.approx(3.0)


# ---------------------------------------------------------------------------
# GotoWaypoints — construction / signature
# ---------------------------------------------------------------------------


@requires_libstp
def test_goto_waypoints_builds_and_signature():
    from raccoon.step.motion import GotoWaypoints

    step = GotoWaypoints(
        [(0.5, 0.0, 0.0, 0.0, "rel", 0.0), (0.5, -0.3, 0.0, 0.0, "rel", math.radians(-90.0))]
    )
    assert len(step._waypoints) == 2
    assert "localization" in step.required_resources()
    sig = step._generate_signature()
    assert sig.startswith("GotoWaypoints(n=2")
    assert "abs=0" in sig


@requires_libstp
def test_goto_waypoints_rejects_empty():
    from raccoon.step.motion import GotoWaypoints

    with pytest.raises(ValueError):
        GotoWaypoints([])


@requires_libstp
def test_goto_waypoints_rejects_bad_speed():
    from raccoon.step.motion import GotoWaypoints

    with pytest.raises(ValueError):
        GotoWaypoints([(0.5, 0.0, 0.0, 0.0, "rel", 0.0)], speed=0.0)
    with pytest.raises(ValueError):
        GotoWaypoints([(0.5, 0.0, 0.0, 0.0, "rel", 0.0)], speed=1.5)
