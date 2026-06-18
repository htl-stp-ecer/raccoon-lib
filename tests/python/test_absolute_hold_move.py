"""Unit tests for ``AbsoluteHoldMove`` and its pure ``_hold_velocity`` law.

``AbsoluteHoldMove`` is the closed-loop-on-localization step that to_absolute
emits for a SENSOR-bounded single-axis drive (e.g. ``strafe_left().until(
on_black)``).  It drives ONE free body-axis open-loop while holding the CROSS
axis position + heading on their absolute (localization) targets, until the
``.until()`` condition fires.

The control law lives in the pure ``_hold_velocity`` helper, tested here with a
tiny pose-like duck type (no robot / C++ runtime).  Conventions under test
(matching ``goto`` / the HAL ``ChassisVelocity`` and the spline integration):

  - ChassisVelocity: vx forward, vy>0 RIGHT, wz>0 CCW.
  - Free-axis world dir: Forward -> (cos h, sin h); Lateral -> +left (-sin h,
    cos h) (consistent with ``_run_to_waypoints``).
  - Body frame via goto's ``_world_to_body``.
"""

from __future__ import annotations

import importlib
import importlib.util
import math

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)

# goto.py imports native modules at module scope, so even the pure helper needs
# the raccoon package present.
pytestmark = requires_libstp


class FakePose:
    def __init__(self, x: float, y: float, heading: float) -> None:
        self.position = (x, y, 0.0)
        self.heading = heading


class _FakeIRSensor:
    """Minimal IRSensor stand-in for ``over_line`` / ``on_black``."""

    def __init__(self, black: float = 0.0) -> None:
        self._black = black

    def probabilityOfBlack(self) -> float:
        return self._black

    def probabilityOfWhite(self) -> float:
        return 1.0 - self._black


@pytest.fixture(scope="module")
def goto_mod():
    return importlib.import_module("raccoon.step.motion.goto")


@pytest.fixture(scope="module")
def hold_velocity(goto_mod):
    return goto_mod._hold_velocity


@pytest.fixture(scope="module")
def LinearAxis():
    from raccoon.motion import LinearAxis as _LA

    return _LA


# ---------------------------------------------------------------------------
# Pure control-law tests — _hold_velocity
# ---------------------------------------------------------------------------


def test_on_line_aligned_forward_drives_free_axis(hold_velocity, LinearAxis):
    # Robot exactly on the intended line, heading aligned with anchor.
    anchor = FakePose(0.0, 0.0, 0.0)
    current = FakePose(0.5, 0.0, 0.0)  # advanced along the free (forward) axis
    vx, vy, wz = hold_velocity(anchor, current, LinearAxis.Forward, 1.0, 1.0)
    # Free axis is body-forward: vx = sign*speed, ~0 cross, ~0 wz.
    assert vx == pytest.approx(1.0)
    assert vy == pytest.approx(0.0, abs=1e-9)
    assert wz == pytest.approx(0.0, abs=1e-9)


def test_negative_sign_reverses_free_axis(hold_velocity, LinearAxis):
    anchor = FakePose(0.0, 0.0, 0.0)
    current = FakePose(0.0, 0.0, 0.0)
    vx, vy, wz = hold_velocity(anchor, current, LinearAxis.Forward, -1.0, 1.0)
    assert vx == pytest.approx(-1.0)
    assert vy == pytest.approx(0.0, abs=1e-9)


def test_lateral_free_axis_commands_strafe(hold_velocity, LinearAxis):
    # Lateral free axis, sign -1 (like strafe_left). Free world dir = +left
    # (0, 1); sign*speed = -1 -> world velocity (0, -1) = right. Body strafe is
    # +right per ChassisVelocity, so vy should be +1, vx ~0.
    anchor = FakePose(0.0, 0.0, 0.0)
    current = FakePose(0.0, 0.0, 0.0)
    vx, vy, wz = hold_velocity(anchor, current, LinearAxis.Lateral, -1.0, 1.0)
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(1.0)
    assert wz == pytest.approx(0.0, abs=1e-9)


def test_cross_drift_opposes_drift_forward(hold_velocity, LinearAxis):
    # Free axis = forward (world +x). Cross dir = +y (90 CCW). Robot drifted to
    # +cross (+y). The cross correction must push back toward -y. With heading 0,
    # body +right (vy) is world -y, so a +cross drift -> +vy correction.
    anchor = FakePose(0.0, 0.0, 0.0)
    current = FakePose(0.0, 0.1, 0.0)  # +0.1 m off the line in +cross
    vx, vy, wz = hold_velocity(anchor, current, LinearAxis.Forward, 1.0, 1.0)
    assert vx == pytest.approx(1.0)  # free axis unaffected
    assert vy > 0.0  # corrects the +cross drift (world -y = body +right)
    assert wz == pytest.approx(0.0, abs=1e-9)


def test_negative_cross_drift_correction_flips(hold_velocity, LinearAxis):
    anchor = FakePose(0.0, 0.0, 0.0)
    current = FakePose(0.0, -0.1, 0.0)  # -cross drift
    _vx, vy, _wz = hold_velocity(anchor, current, LinearAxis.Forward, 1.0, 1.0)
    assert vy < 0.0  # opposite sign vs +cross drift


def test_heading_off_produces_correcting_wz(hold_velocity, LinearAxis):
    # Anchor heading 0, current heading -10deg -> heading_err = +10deg -> wz > 0
    # (CCW) to rotate back to anchor heading.
    anchor = FakePose(0.0, 0.0, 0.0)
    current = FakePose(0.0, 0.0, math.radians(-10.0))
    _vx, _vy, wz = hold_velocity(anchor, current, LinearAxis.Forward, 1.0, 1.0)
    assert wz > 0.0


def test_heading_off_other_way_flips_wz(hold_velocity, LinearAxis):
    anchor = FakePose(0.0, 0.0, 0.0)
    current = FakePose(0.0, 0.0, math.radians(10.0))
    _vx, _vy, wz = hold_velocity(anchor, current, LinearAxis.Forward, 1.0, 1.0)
    assert wz < 0.0


def test_anchor_rotated_90_rotates_body_command(hold_velocity, LinearAxis):
    # Anchor + current both facing world +Y (heading 90deg), on the line. The
    # free axis (forward) world dir is (0, 1). The body-frame command should be
    # pure body-forward = sign*speed, with ~0 strafe and ~0 wz (NOT leak into vy
    # despite the world velocity being along +Y).
    h = math.radians(90.0)
    anchor = FakePose(0.0, 0.0, h)
    current = FakePose(0.0, 0.5, h)
    vx, vy, wz = hold_velocity(anchor, current, LinearAxis.Forward, 1.0, 1.0)
    assert vx == pytest.approx(1.0)
    assert vy == pytest.approx(0.0, abs=1e-9)
    assert wz == pytest.approx(0.0, abs=1e-9)


def test_speed_scales_free_axis(hold_velocity, LinearAxis):
    anchor = FakePose(0.0, 0.0, 0.0)
    current = FakePose(0.0, 0.0, 0.0)
    vx, _vy, _wz = hold_velocity(anchor, current, LinearAxis.Forward, 1.0, 0.5)
    assert vx == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# Construction / signature
# ---------------------------------------------------------------------------


def test_absolute_hold_move_builds_and_signature(LinearAxis):
    from raccoon.step.condition import after_seconds
    from raccoon.step.motion import AbsoluteHoldMove

    cond = after_seconds(1.0)
    step = AbsoluteHoldMove(LinearAxis.Lateral, -1.0, 1.0, cond)
    assert "localization" in step.required_resources()
    assert "drive" in step.required_resources()
    sig = step._generate_signature()
    assert sig.startswith("AbsoluteHoldMove(")
    assert "free=Lateral" in sig
    assert "sign=-1" in sig
    assert "until=..." in sig


# ---------------------------------------------------------------------------
# Pass conversion — to_absolute emits AbsoluteHoldMove for a sensor leg
# ---------------------------------------------------------------------------


def _imports():
    from raccoon.motion import LinearAxis as _LA
    from raccoon.step.condition import after_cm, on_black
    from raccoon.step.motion import AbsoluteHoldMove, GotoWaypoints
    from raccoon.step.motion.arc_dsl import drive_arc_left
    from raccoon.step.motion.drive_dsl import drive_forward, strafe_left
    from raccoon.step.motion.line_follow_dsl import follow_line
    from raccoon.step.motion.path.ir import Segment, SideAction
    from raccoon.step.motion.path.passes import ToAbsolutePass, flatten_steps

    return {
        "LinearAxis": _LA,
        "after_cm": after_cm,
        "on_black": on_black,
        "AbsoluteHoldMove": AbsoluteHoldMove,
        "GotoWaypoints": GotoWaypoints,
        "drive_arc_left": drive_arc_left,
        "drive_forward": drive_forward,
        "strafe_left": strafe_left,
        "follow_line": follow_line,
        "Segment": Segment,
        "SideAction": SideAction,
        "ToAbsolutePass": ToAbsolutePass,
        "flatten_steps": flatten_steps,
    }


def _run_pass(steps, imp):
    nodes, _deferred = imp["flatten_steps"](steps)
    return imp["ToAbsolutePass"]().run(nodes)


def _classify(nodes, imp):
    """Return ordered list of node kinds for the GotoWaypoints/AbsoluteHoldMove
    motion side actions (ignoring non-motion side actions)."""
    out = []
    for n in nodes:
        if isinstance(n, imp["SideAction"]):
            if isinstance(n.step, imp["GotoWaypoints"]):
                out.append("GotoWaypoints")
            elif isinstance(n.step, imp["AbsoluteHoldMove"]):
                out.append("AbsoluteHoldMove")
    return out


def test_single_sensor_leg_converts_to_one_hold():
    imp = _imports()
    sensor = _FakeIRSensor()
    nodes = _run_pass([imp["strafe_left"]().until(imp["on_black"](sensor))], imp)
    assert not any(isinstance(n, imp["Segment"]) for n in nodes)
    holds = [
        n.step
        for n in nodes
        if isinstance(n, imp["SideAction"]) and isinstance(n.step, imp["AbsoluteHoldMove"])
    ]
    assert len(holds) == 1
    hold = holds[0]
    assert hold._free_axis == imp["LinearAxis"].Lateral
    # strafe_left sign convention.
    assert hold._sign == pytest.approx(-1.0)


def test_mixed_run_flushes_then_holds_then_reanchors():
    imp = _imports()
    sensor = _FakeIRSensor()
    nodes = _run_pass(
        [
            imp["drive_forward"](50),
            imp["strafe_left"]().until(imp["on_black"](sensor)),
            imp["drive_forward"](30),
        ],
        imp,
    )
    assert _classify(nodes, imp) == ["GotoWaypoints", "AbsoluteHoldMove", "GotoWaypoints"]
    assert not any(isinstance(n, imp["Segment"]) for n in nodes)


def test_baked_after_cm_stays_goto_waypoints():
    """Regression: a baked relative ``.until(after_cm)`` leg keeps a known
    endpoint, so it becomes a GotoWaypoints waypoint, NOT an AbsoluteHoldMove."""
    imp = _imports()
    nodes = _run_pass([imp["drive_forward"]().until(imp["after_cm"](12))], imp)
    assert _classify(nodes, imp) == ["GotoWaypoints"]
    holds = [
        n
        for n in nodes
        if isinstance(n, imp["SideAction"]) and isinstance(n.step, imp["AbsoluteHoldMove"])
    ]
    assert not holds


def test_follow_line_sensor_leg_untouched():
    imp = _imports()
    sensor = _FakeIRSensor()
    step = imp["follow_line"](sensor, sensor).until(imp["on_black"](sensor))
    nodes = _run_pass([step], imp)
    # follow_line stays a Segment (kind follow_line) — NOT converted.
    segs = [n for n in nodes if isinstance(n, imp["Segment"])]
    assert len(segs) == 1
    assert segs[0].kind == "follow_line"
    assert not _classify(nodes, imp)


def test_arc_sensor_leg_untouched():
    imp = _imports()
    step = imp["drive_arc_left"](radius_cm=20, degrees=90)
    nodes = _run_pass([step], imp)
    segs = [n for n in nodes if isinstance(n, imp["Segment"])]
    assert len(segs) == 1
    assert segs[0].kind == "arc"
    assert not _classify(nodes, imp)
