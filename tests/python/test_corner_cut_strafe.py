"""Corner-cut on strafes: symmetric strafe turn corners + holonomic crab corners.

Beyond the forward ``linear+turn+linear`` corner (see
``test_corner_cut_seethrough.py``), ``cut_corners`` rounds:

* a ``strafe+turn+strafe`` corner (matching-axis lateral legs) → lateral ``arc``,
* a ``forward↔strafe`` corner with NO turn → constant-heading ``crab_arc``.

These tests exercise the pass geometry only (no robot). The ``CrabArcAdapter``
runtime is covered by ``test_crab_arc_adapter.py``.
"""

from __future__ import annotations

import importlib.util
import math

import pytest


def _has_raccoon() -> bool:
    return importlib.util.find_spec("raccoon") is not None


pytestmark = pytest.mark.skipif(not _has_raccoon(), reason="raccoon not importable")


def _lin(dist_m=0.5, sign=1.0, axis=None, cond=None, known=True, speed=1.0):
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.path.ir import Segment

    return Segment(
        kind="linear",
        axis=axis if axis is not None else LinearAxis.Forward,
        sign=sign,
        distance_m=dist_m,
        condition=cond,
        has_known_endpoint=known,
        speed_scale=speed,
    )


def _fwd(dist_m=0.5, **kw):
    from raccoon.motion import LinearAxis

    return _lin(dist_m=dist_m, sign=math.copysign(1.0, dist_m), axis=LinearAxis.Forward, **kw)


def _strafe(dist_m=0.3, right=True, **kw):
    from raccoon.motion import LinearAxis

    sign = 1.0 if right else -1.0
    return _lin(dist_m=math.copysign(dist_m, sign), sign=sign, axis=LinearAxis.Lateral, **kw)


def _turn(deg=90.0, known=True):
    from raccoon.step.motion.path.ir import Segment

    return Segment(
        kind="turn",
        angle_rad=(math.radians(deg) if deg is not None else None),
        has_known_endpoint=known,
    )


def _kinds(nodes):
    from raccoon.step.motion.path.ir import Segment

    return [n.kind if isinstance(n, Segment) else "side" for n in nodes]


# ---------------------------------------------------------------------------
# Symmetric strafe turn corner — lateral arc
# ---------------------------------------------------------------------------


def test_strafe_turn_strafe_cuts_to_lateral_arc():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_strafe(0.5, right=True), _turn(90), _strafe(0.5, right=True)], cut_m=0.1)
    assert _kinds(out) == ["linear", "arc", "linear"]
    arc = out[1]
    assert arc.lateral is True
    assert math.isclose(arc.radius_m, 0.1, rel_tol=1e-6)
    assert math.isclose(arc.arc_angle_rad, math.radians(90), rel_tol=1e-6)
    # Legs trimmed toward the corner: strafe-right (+0.5) → +0.4.
    assert math.isclose(out[0].distance_m, 0.4, rel_tol=1e-6)
    assert math.isclose(out[2].distance_m, 0.4, rel_tol=1e-6)


def test_backward_turn_backward_cuts_to_drive_arc():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_fwd(-0.5), _turn(-90), _fwd(-0.5)], cut_m=0.1)
    assert _kinds(out) == ["linear", "arc", "linear"]
    arc = out[1]
    assert arc.lateral is False
    # Backward legs (-0.5) trim toward the corner → -0.4.
    assert math.isclose(out[0].distance_m, -0.4, rel_tol=1e-6)
    assert math.isclose(out[2].distance_m, -0.4, rel_tol=1e-6)


def test_mixed_axis_turn_corner_still_not_cut():
    # forward + turn + strafe has a turn between perpendicular legs: a single
    # heading-rotating arc can't express it, and there's a turn so it's not a
    # crab corner either. Stays sharp (unchanged).
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_fwd(0.5), _turn(90), _strafe(0.5)], cut_m=0.1)
    assert _kinds(out) == ["linear", "turn", "linear"]


# ---------------------------------------------------------------------------
# Crab corner — constant-heading forward↔strafe, no turn
# ---------------------------------------------------------------------------


def test_forward_then_strafe_cuts_to_crab_arc():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_fwd(0.5), _strafe(0.3, right=False)], cut_m=0.1)
    assert _kinds(out) == ["linear", "crab_arc", "linear"]
    crab = out[1]
    # 90° corner → R = cut / tan(45°) = cut.
    assert math.isclose(crab.radius_m, 0.1, rel_tol=1e-6)
    assert math.isclose(crab.arc_angle_rad, math.pi / 2, rel_tol=1e-6)
    # Entry = forward (+vx); exit = strafe left (-vy).
    assert crab.crab_from == (1.0, 0.0)
    assert crab.crab_to == (0.0, -1.0)
    # Legs trimmed by the cut.
    assert math.isclose(out[0].distance_m, 0.4, rel_tol=1e-6)
    assert math.isclose(out[2].distance_m, -0.2, rel_tol=1e-6)


def test_strafe_then_forward_cuts_to_crab_arc():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_strafe(0.3, right=True), _fwd(0.5)], cut_m=0.1)
    assert _kinds(out) == ["linear", "crab_arc", "linear"]
    crab = out[1]
    assert crab.crab_from == (0.0, 1.0)  # strafe right (+vy)
    assert crab.crab_to == (1.0, 0.0)  # forward (+vx)


def test_crab_speed_scale_is_min_of_legs():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut(
        [_fwd(0.5, speed=0.8), _strafe(0.3, right=True, speed=0.5)],
        cut_m=0.1,
    )
    assert math.isclose(out[1].speed_scale, 0.5, rel_tol=1e-6)


def test_same_axis_no_turn_is_not_a_crab_corner():
    # Two forward legs that somehow survived merge (e.g. distinct speed) are a
    # straight run, not a corner — must not be cut.
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_fwd(0.5, speed=0.5), _fwd(0.5, speed=1.0)], cut_m=0.1)
    assert _kinds(out) == ["linear", "linear"]


def test_crab_short_leg_not_cut():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_fwd(0.05), _strafe(0.5)], cut_m=0.1)
    assert _kinds(out) == ["linear", "linear"]


def test_crab_condition_leg_not_cut():
    # Sensor-bounded strafe has no known endpoint → known-endpoint legs only.
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    sensor_strafe = _lin(
        dist_m=None, sign=-1.0, axis=LinearAxis.Lateral, cond=object(), known=False
    )
    out = run_corner_cut([_fwd(0.5), sensor_strafe], cut_m=0.1)
    assert _kinds(out) == ["linear", "linear"]


def test_crab_corner_keeps_background_side_action():
    from raccoon.step.motion.path.ir import SideAction
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    bg = SideAction(step=object(), is_background=True)
    out = run_corner_cut([_fwd(0.5), bg, _strafe(0.3)], cut_m=0.1)
    assert _kinds(out) == ["linear", "side", "crab_arc", "linear"]


def test_crab_corner_blocking_side_action_is_barrier():
    from raccoon.step.motion.path.ir import SideAction
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    blocking = SideAction(step=object(), is_background=False)
    out = run_corner_cut([_fwd(0.5), blocking, _strafe(0.3)], cut_m=0.1)
    # The inline action stops the robot → no smooth corner.
    assert _kinds(out) == ["linear", "side", "linear"]


# ---------------------------------------------------------------------------
# cut_until: round a corner whose EXIT leg is a sensor-bounded .until() leg
# ---------------------------------------------------------------------------


def _sensor_strafe(right=True):
    from raccoon.motion import LinearAxis

    sign = 1.0 if right else -1.0
    return _lin(dist_m=None, sign=sign, axis=LinearAxis.Lateral, cond=object(), known=False)


def _sensor_fwd(sign=1.0):
    from raccoon.motion import LinearAxis

    return _lin(dist_m=None, sign=sign, axis=LinearAxis.Forward, cond=object(), known=False)


def test_crab_sensor_exit_not_cut_by_default():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_fwd(-0.18), _sensor_strafe(right=False)], cut_m=0.05)
    assert _kinds(out) == ["linear", "linear"]


def test_crab_sensor_exit_cut_with_flag():
    # drive_backward(18) + strafe_left().until(over_line) — the M100 pattern.
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_fwd(-0.18), _sensor_strafe(right=False)], cut_m=0.05, cut_until=True)
    assert _kinds(out) == ["linear", "crab_arc", "linear"]
    # Entry leg trimmed; sensor exit leg kept untrimmed (still condition-driven).
    assert math.isclose(out[0].distance_m, -0.13, rel_tol=1e-6)
    assert out[2].distance_m is None
    assert out[2].condition is not None
    assert out[1].crab_from == (-1.0, 0.0)  # backward
    assert out[1].crab_to == (0.0, -1.0)  # strafe left


def test_turn_sensor_exit_cut_with_flag():
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_fwd(0.5), _turn(90), _sensor_fwd()], cut_m=0.1, cut_until=True)
    assert _kinds(out) == ["linear", "arc", "linear"]
    assert math.isclose(out[0].distance_m, 0.4, rel_tol=1e-6)
    assert out[2].distance_m is None
    assert out[2].condition is not None


def test_sensor_entry_leg_never_cut_even_with_flag():
    # The entry leg can't be anticipated — a fillet would start before the
    # sensor fires. Stays sharp regardless of the flag.
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_sensor_fwd(), _strafe(0.3, right=True)], cut_m=0.05, cut_until=True)
    assert _kinds(out) == ["linear", "linear"]

    out2 = run_corner_cut([_sensor_fwd(), _turn(90), _fwd(0.5)], cut_m=0.1, cut_until=True)
    assert _kinds(out2) == ["linear", "turn", "linear"]


def test_sensor_exit_short_entry_not_cut_with_flag():
    # Entry still needs enough known distance to trim.
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut([_fwd(0.03), _sensor_strafe()], cut_m=0.1, cut_until=True)
    assert _kinds(out) == ["linear", "linear"]


def test_crab_then_continue_into_turn_corner():
    # forward + strafe + turn + strafe: the crab corner consumes forward+strafe;
    # the trailing turn+strafe lacks a leading leg, so only the crab cut fires.
    from raccoon.step.motion.path.passes.corner_cut import run_corner_cut

    out = run_corner_cut(
        [_fwd(0.5), _strafe(0.5, right=True), _turn(90), _strafe(0.5, right=True)],
        cut_m=0.1,
    )
    assert _kinds(out)[:2] == ["linear", "crab_arc"]
    assert "crab_arc" in _kinds(out)
