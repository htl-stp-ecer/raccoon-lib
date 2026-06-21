"""Time-optimal velocity profiling pass (VelocityProfilePass) — two-sweep math.

The pass stamps feasible entry/exit speeds (m/s) on every motion Segment so the
robot carries speed through the whole path and only drops to a stop at real
barriers (turns, reversals, sensor-bounded leg ends, blocking side actions, the
last leg) and corner/curvature limits. These tests pin that math directly on
synthetic node lists — no robot, no sim.
"""

from __future__ import annotations

import importlib.util
import math

import pytest


def _has_raccoon() -> bool:
    return importlib.util.find_spec("raccoon") is not None


pytestmark = pytest.mark.skipif(not _has_raccoon(), reason="raccoon not importable")


def _lin(dist_m, sign=1.0, scale=1.0, condition=None):
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.path.ir import Segment

    return Segment(kind="linear", axis=LinearAxis.Forward, distance_m=dist_m,
                   sign=sign, speed_scale=scale, condition=condition)


def _turn(angle=math.pi / 2):
    from raccoon.step.motion.path.ir import Segment

    return Segment(kind="turn", angle_rad=angle)


def _arc(radius_m, angle=math.pi / 2):
    from raccoon.step.motion.path.ir import Segment

    return Segment(kind="arc", radius_m=radius_m, arc_angle_rad=angle)


def _profile(nodes, **kw):
    from raccoon.step.motion.path.passes import run_velocity_profile

    return run_velocity_profile(nodes, **kw)


def test_starts_and_ends_at_rest():
    out = _profile([_lin(1.0), _lin(1.0)])
    assert out[0].entry_speed_mps == pytest.approx(0.0)
    assert out[-1].exit_speed_mps == pytest.approx(0.0)


def test_collinear_seam_carries_speed():
    # Two long forward linears with nothing between → the middle seam carries a
    # positive speed (the robot does NOT stop between them).
    out = _profile([_lin(2.0), _lin(2.0)], max_speed_mps=1.0, accel_mps2=0.6)
    assert out[0].exit_speed_mps > 0.1
    assert out[1].entry_speed_mps == pytest.approx(out[0].exit_speed_mps)


def test_turn_forces_full_stop_on_both_sides():
    out = _profile([_lin(2.0), _turn(), _lin(2.0)])
    # the linear before the turn must exit at rest, and after the turn enter at rest
    assert out[0].exit_speed_mps == pytest.approx(0.0)
    assert out[2].entry_speed_mps == pytest.approx(0.0)
    # the turn itself carries no translational speed
    assert out[1].entry_speed_mps == pytest.approx(0.0)
    assert out[1].exit_speed_mps == pytest.approx(0.0)


def test_speed_carries_through_a_tangent_arc():
    # linear + arc + linear (as cut_corners emits) carries speed through the arc.
    out = _profile([_lin(3.0), _arc(0.5, math.pi / 2), _lin(3.0)],
                   max_speed_mps=1.0, accel_mps2=0.6, lateral_accel_mps2=0.5)
    assert out[0].exit_speed_mps > 0.0
    assert out[1].entry_speed_mps > 0.0
    assert out[1].exit_speed_mps > 0.0
    assert out[2].entry_speed_mps > 0.0


def test_arc_curvature_caps_through_speed():
    # A tight arc caps the through-speed at sqrt(lat_accel * R).
    lat, r = 0.5, 0.08
    out = _profile([_lin(5.0), _arc(r, math.pi / 2), _lin(5.0)],
                   max_speed_mps=2.0, accel_mps2=2.0, lateral_accel_mps2=lat)
    cap = math.sqrt(lat * r)
    assert out[1].entry_speed_mps <= cap + 1e-6
    assert out[1].exit_speed_mps <= cap + 1e-6
    assert out[1].entry_speed_mps == pytest.approx(cap, abs=1e-3)


def test_sensor_leg_carries_through_into_same_direction_successor():
    # "On steroids": a sensor-bounded leg followed by a same-direction leg no
    # longer brakes to zero — it FLOWS THROUGH at the sensor-carry cap.
    out = _profile([_lin(2.0, condition=object()), _lin(2.0)], sensor_carry_mps=0.35)
    assert out[0].exit_speed_mps == pytest.approx(0.35)
    assert out[1].entry_speed_mps == pytest.approx(0.35)


def test_sensor_carry_capped_not_full_cruise():
    # The carry is throttled to the cap, never the full cruise, so the sensor is
    # sampled densely and the post-trigger slip stays bounded.
    out = _profile(
        [_lin(5.0, condition=object()), _lin(5.0)],
        max_speed_mps=1.0, accel_mps2=0.6, sensor_carry_mps=0.3,
    )
    assert out[0].exit_speed_mps == pytest.approx(0.3)


def test_sensor_carry_zero_restores_brake_at_sensor():
    # sensor_carry_mps=0 → the legacy behaviour: stop at every sensor boundary.
    out = _profile([_lin(2.0, condition=object()), _lin(2.0)], sensor_carry_mps=0.0)
    assert out[0].exit_speed_mps == pytest.approx(0.0)


def test_sensor_leg_stops_before_incompatible_successor():
    # A sensor leg followed by a TURN or a direction reversal still stops — the
    # seam is discontinuous, so the carry never applies.
    out = _profile([_lin(2.0, condition=object()), _turn()], sensor_carry_mps=0.35)
    assert out[0].exit_speed_mps == pytest.approx(0.0)
    # forward sensor leg → backward leg (negative distance) is a reversal.
    rev = _profile(
        [_lin(2.0, condition=object()), _lin(-2.0, sign=-1.0)], sensor_carry_mps=0.35
    )
    assert rev[0].exit_speed_mps == pytest.approx(0.0)


def test_blocking_side_action_breaks_the_carry():
    from raccoon.step.motion.path.ir import SideAction

    blocker = SideAction(step=object(), is_background=False)
    out = _profile([_lin(2.0), blocker, _lin(2.0)])
    # robot stops to run the inline side action.
    assert out[0].exit_speed_mps == pytest.approx(0.0)
    assert out[2].entry_speed_mps == pytest.approx(0.0)


def test_background_side_action_does_not_break_carry():
    from raccoon.step.motion.path.ir import SideAction

    bg = SideAction(step=object(), is_background=True)
    out = _profile([_lin(2.0), bg, _lin(2.0)], max_speed_mps=1.0, accel_mps2=0.6)
    assert out[0].exit_speed_mps > 0.0  # background doesn't stop the spine


def test_direction_reversal_forces_stop():
    # A backward leg carries the sign on distance_m (drive_backward → -2.0).
    out = _profile([_lin(2.0), _lin(-2.0)])
    assert out[0].exit_speed_mps == pytest.approx(0.0)
    assert out[1].entry_speed_mps == pytest.approx(0.0)


def test_short_leg_before_stop_limits_entry_for_braking():
    # A short final leg can only be entered slowly enough to brake to 0 within it.
    short = 0.05
    out = _profile([_lin(2.0), _lin(short)], max_speed_mps=2.0, accel_mps2=0.6)
    brakeable = math.sqrt(2 * 0.6 * short)
    assert out[1].entry_speed_mps <= brakeable + 1e-6
    assert out[1].exit_speed_mps == pytest.approx(0.0)


def _compiled_nodes(opt):
    from raccoon.step.motion.path.compiler import PathCompiler

    return PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes


def test_end_to_end_carries_speed_through_a_cut_corner_arc():
    # The headline win: optimize(...).cut_corners().time_optimal() rounds the
    # corner into an arc AND carries speed straight through it, where the
    # un-profiled path stops dead at the corner.
    from raccoon.step.motion import optimize
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.turn_dsl import turn_right

    steps = [drive_forward(60), turn_right(90), drive_forward(60)]

    # Without time_optimal: corner-cut arc exists but carries no profile.
    plain = _compiled_nodes(optimize(steps).cut_corners(10))
    arc_plain = next(n for n in plain if getattr(n, "kind", None) == "arc")
    assert arc_plain.entry_speed_mps is None
    assert arc_plain.exit_speed_mps is None

    # With time_optimal: the arc is entered AND exited at speed > 0.
    profiled = _compiled_nodes(optimize(steps).cut_corners(10).time_optimal())
    arc = next(n for n in profiled if getattr(n, "kind", None) == "arc")
    assert arc.entry_speed_mps > 0.0
    assert arc.exit_speed_mps > 0.0
    # capped by the arc curvature: v = sqrt(lat_accel * R), R = cut/tan(45°).
    r_m = 0.10 / math.tan(math.radians(45.0))
    cap = math.sqrt(0.50 * r_m)
    assert arc.entry_speed_mps == pytest.approx(cap, abs=1e-3)
    # path-preserving: first leg still starts from rest, last leg ends at rest.
    assert profiled[0].entry_speed_mps == pytest.approx(0.0)
    assert profiled[-1].exit_speed_mps == pytest.approx(0.0)


def test_unprofiled_fields_default_none():
    # A freshly built segment has no profile until the pass runs.
    seg = _lin(1.0)
    assert seg.entry_speed_mps is None
    assert seg.exit_speed_mps is None
    out = _profile([seg])
    assert out[0].entry_speed_mps is not None
