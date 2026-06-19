"""Unit tests for ``SplineFollow`` — the continuous pure-pursuit spline follower.

The pursuit math lives in three pure helpers (no robot required) that are tested
directly against a tiny pose-like duck type:

  - ``_project_index``  — monotonic projection of the robot onto the polyline.
  - ``_lookahead_point`` — the carrot ``lookahead_m`` of arc length ahead.
  - ``_pursuit_velocity`` — body-frame ``(vx, vy, wz)`` toward the carrot, with
    an INDEPENDENT heading channel.

Conventions under test (match ``goto`` / the HAL ``ChassisVelocity`` and
``_world_to_body``):
  - +vx is body-forward.
  - +vy is to the robot's RIGHT (so a carrot to the LEFT gives -vy).
  - +wz is counter-clockwise.
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

# goto.py imports native modules at module scope, so even the pure helpers need
# the raccoon package present.
pytestmark = requires_libstp


class FakePose:
    def __init__(self, x: float, y: float, heading: float) -> None:
        self.position = (x, y, 0.0)
        self.heading = heading


@pytest.fixture(scope="module")
def goto_mod():
    return importlib.import_module("raccoon.step.motion.goto")


@pytest.fixture(scope="module")
def project_index(goto_mod):
    return goto_mod._project_index


@pytest.fixture(scope="module")
def lookahead_point(goto_mod):
    return goto_mod._lookahead_point


@pytest.fixture(scope="module")
def pursuit_velocity(goto_mod):
    return goto_mod._pursuit_velocity


@pytest.fixture(scope="module")
def linear_gain(goto_mod):
    return goto_mod._LINEAR_GAIN


@pytest.fixture(scope="module")
def angular_gain(goto_mod):
    return goto_mod._ANGULAR_GAIN


def _straight_path(n: int = 11, spacing: float = 0.1):
    """Straight polyline along +X: (0,0), (0.1,0), ..., ((n-1)*spacing, 0)."""
    return [(i * spacing, 0.0) for i in range(n)]


# ---------------------------------------------------------------------------
# _project_index
# ---------------------------------------------------------------------------


def test_project_index_nearest_on_straight_line(project_index):
    path = _straight_path()
    # Robot near x=0.52 → nearest index is 5 (x=0.5).
    idx = project_index(path, (0.52, 0.01), start_idx=0, window=20)
    assert idx == 5


def test_project_index_is_monotonic(project_index):
    path = _straight_path()
    # Robot physically near index 2, but start_idx pins us at 6 — never go back.
    idx = project_index(path, (0.2, 0.0), start_idx=6, window=20)
    assert idx >= 6
    assert idx == 6


def test_project_index_advances_within_window(project_index):
    path = _straight_path()
    idx = project_index(path, (0.71, 0.0), start_idx=3, window=20)
    assert idx == 7


def test_project_index_window_limits_advance(project_index):
    path = _straight_path()
    # Robot is near index 9 but the window only allows +2 from start_idx=3.
    idx = project_index(path, (0.95, 0.0), start_idx=3, window=2)
    assert idx == 5


def test_project_index_clamps_start_idx(project_index):
    path = _straight_path(n=4)
    idx = project_index(path, (0.3, 0.0), start_idx=100, window=5)
    assert idx == 3


# ---------------------------------------------------------------------------
# _lookahead_point
# ---------------------------------------------------------------------------


def test_lookahead_point_walks_arc_length(lookahead_point):
    path = _straight_path()  # spacing 0.1 along +X
    # From idx 0, walk 0.25 m → x = 0.25.
    x, y = lookahead_point(path, 0, 0.25)
    assert x == pytest.approx(0.25)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_lookahead_point_from_nonzero_idx(lookahead_point):
    path = _straight_path()
    # From idx 4 (x=0.4), walk 0.15 → x = 0.55.
    x, y = lookahead_point(path, 4, 0.15)
    assert x == pytest.approx(0.55)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_lookahead_point_clamps_to_last_near_end(lookahead_point):
    path = _straight_path(n=11)  # last x = 1.0
    # From idx 9 (x=0.9), a lookahead of 0.5 overshoots → clamp to last point.
    x, y = lookahead_point(path, 9, 0.5)
    assert (x, y) == pytest.approx(path[-1])


def test_lookahead_point_at_last_index_returns_last(lookahead_point):
    path = _straight_path(n=6)
    x, y = lookahead_point(path, 5, 0.2)
    assert (x, y) == pytest.approx(path[-1])


# ---------------------------------------------------------------------------
# _pursuit_velocity
# ---------------------------------------------------------------------------


def test_pursuit_robot_behind_drives_forward(pursuit_velocity, linear_gain):
    pose = FakePose(0.0, 0.0, 0.0)
    # Carrot straight ahead in +X; heading already aligned.
    vx, vy, wz = pursuit_velocity(pose, (0.3, 0.0), heading_target=0.0, speed=1.0)
    assert vx == pytest.approx(linear_gain * 0.3)
    assert vy == pytest.approx(0.0, abs=1e-9)
    assert wz == pytest.approx(0.0, abs=1e-9)


def test_pursuit_carrot_left_gives_negative_vy(pursuit_velocity, linear_gain):
    pose = FakePose(0.0, 0.0, 0.0)
    # Carrot to the robot's LEFT = world +Y at heading 0. Per HAL right-positive,
    # left motion is -vy.
    vx, vy, wz = pursuit_velocity(pose, (0.0, 0.3), heading_target=0.0, speed=1.0)
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(-linear_gain * 0.3)


def test_pursuit_carrot_right_gives_positive_vy(pursuit_velocity, linear_gain):
    pose = FakePose(0.0, 0.0, 0.0)
    # Carrot to the robot's RIGHT = world -Y at heading 0 → +vy.
    vx, vy, _ = pursuit_velocity(pose, (0.0, -0.3), heading_target=0.0, speed=1.0)
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(linear_gain * 0.3)


def test_pursuit_heading_error_corrects_ccw(pursuit_velocity, angular_gain):
    pose = FakePose(0.0, 0.0, 0.0)
    # Heading target +30° (CCW) → positive wz.
    _, _, wz = pursuit_velocity(pose, (0.3, 0.0), heading_target=math.radians(30.0), speed=1.0)
    assert wz == pytest.approx(angular_gain * math.radians(30.0))
    assert wz > 0.0


def test_pursuit_rotated_robot_90_rotates_body_command(pursuit_velocity, linear_gain):
    # Robot facing world +Y (heading 90°). A carrot at world (0, 0.3) is straight
    # AHEAD of the robot → all forward vx, no strafe.
    pose = FakePose(0.0, 0.0, math.radians(90.0))
    vx, vy, _ = pursuit_velocity(pose, (0.0, 0.3), heading_target=math.radians(90.0), speed=1.0)
    assert vx == pytest.approx(linear_gain * 0.3)
    assert vy == pytest.approx(0.0, abs=1e-9)


def test_pursuit_rotated_robot_90_carrot_to_world_plus_x_is_right(pursuit_velocity, linear_gain):
    # Robot facing world +Y. A carrot at world (0.3, 0) is to the robot's RIGHT
    # (heading 90°, +X is the robot's right) → +vy, no forward.
    pose = FakePose(0.0, 0.0, math.radians(90.0))
    vx, vy, _ = pursuit_velocity(pose, (0.3, 0.0), heading_target=math.radians(90.0), speed=1.0)
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(linear_gain * 0.3)


def test_pursuit_speed_scales_command(pursuit_velocity, linear_gain):
    pose = FakePose(0.0, 0.0, 0.0)
    vx, _, _ = pursuit_velocity(pose, (0.4, 0.0), heading_target=0.0, speed=0.5)
    assert vx == pytest.approx(linear_gain * 0.4 * 0.5)


# ---------------------------------------------------------------------------
# Heading channel modes (via the step's _heading_target)
# ---------------------------------------------------------------------------


@requires_libstp
def test_heading_mode_hold_targets_anchor(goto_mod):
    step = goto_mod.SplineFollow([(0.0, 0.0), (1.0, 0.0)], heading_mode="hold")
    step._anchor_heading = math.radians(40.0)
    step._path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
    step._idx = 1
    # Hold → target is always the anchor heading regardless of progress.
    assert step._heading_target(math.radians(10.0)) == pytest.approx(math.radians(40.0))


@requires_libstp
def test_heading_mode_hold_wz_drives_toward_anchor(goto_mod, angular_gain):
    step = goto_mod.SplineFollow([(0.0, 0.0), (1.0, 0.0)], heading_mode="hold")
    step._anchor_heading = math.radians(30.0)
    step._path = [(0.0, 0.0), (1.0, 0.0)]
    step._idx = 0
    target = step._heading_target(math.radians(0.0))
    pose = FakePose(0.0, 0.0, math.radians(0.0))
    _, _, wz = goto_mod._pursuit_velocity(pose, (0.3, 0.0), target, 1.0)
    # Current 0°, anchor 30° → positive (CCW) correction.
    assert wz == pytest.approx(angular_gain * math.radians(30.0))
    assert wz > 0.0


@requires_libstp
def test_heading_mode_goal_interpolates_with_progress(goto_mod):
    goal = math.radians(90.0)
    step = goto_mod.SplineFollow([(0.0, 0.0), (1.0, 0.0)], heading_mode=goal)
    step._anchor_heading = 0.0
    # 5-point path → progress = idx / 4.
    step._path = [(0.0, 0.0), (0.25, 0.0), (0.5, 0.0), (0.75, 0.0), (1.0, 0.0)]

    step._idx = 0
    assert step._heading_target(0.0) == pytest.approx(0.0)

    step._idx = 2  # progress 0.5
    assert step._heading_target(0.0) == pytest.approx(math.radians(45.0))

    step._idx = 4  # progress 1.0
    assert step._heading_target(0.0) == pytest.approx(math.radians(90.0))


@requires_libstp
def test_heading_mode_tangent_faces_curve_direction(goto_mod):
    step = goto_mod.SplineFollow([(0.0, 0.0), (1.0, 0.0)], heading_mode="tangent")
    # A polyline going up +Y → tangent heading is +90°.
    step._path = [(0.0, 0.0), (0.0, 0.5), (0.0, 1.0)]
    step._idx = 0
    assert step._heading_target(0.0) == pytest.approx(math.radians(90.0))


# ---------------------------------------------------------------------------
# Construction / signature / required resources
# ---------------------------------------------------------------------------


@requires_libstp
def test_spline_follow_builds_and_signature():
    from raccoon.step.motion import SplineFollow

    step = SplineFollow([(0.0, 0.0), (0.5, 0.2), (1.0, 0.0)])
    assert "localization" in step.required_resources()
    assert "drive" in step.required_resources()
    sig = step._generate_signature()
    assert sig.startswith("SplineFollow(")
    assert "n_ctrl=3" in sig
    assert "heading=hold" in sig


@requires_libstp
def test_spline_follow_goal_angle_signature():
    from raccoon.step.motion import SplineFollow

    step = SplineFollow([(0.0, 0.0), (1.0, 0.0)], heading_mode=math.radians(90.0))
    assert "heading=90.0" in step._generate_signature()


@requires_libstp
def test_spline_follow_rejects_too_few_points():
    from raccoon.step.motion import SplineFollow

    with pytest.raises(ValueError):
        SplineFollow([(0.0, 0.0)])


@requires_libstp
def test_spline_follow_rejects_bad_speed():
    from raccoon.step.motion import SplineFollow

    with pytest.raises(ValueError):
        SplineFollow([(0.0, 0.0), (1.0, 0.0)], speed=0.0)
    with pytest.raises(ValueError):
        SplineFollow([(0.0, 0.0), (1.0, 0.0)], speed=1.5)


@requires_libstp
def test_spline_follow_rejects_bad_heading_mode():
    from raccoon.step.motion import SplineFollow

    with pytest.raises(ValueError):
        SplineFollow([(0.0, 0.0), (1.0, 0.0)], heading_mode="sideways")


@requires_libstp
def test_spline_follow_requires_localization():
    from raccoon.step.motion import SplineFollow

    class _NoLocRobot:
        localization = None

    step = SplineFollow([(0.0, 0.0), (1.0, 0.0)])
    with pytest.raises(RuntimeError):
        step.on_start(_NoLocRobot())


# ---------------------------------------------------------------------------
# End-to-end of the helpers: fake poses advancing along a straight polyline
# ---------------------------------------------------------------------------


@requires_libstp
def test_end_to_end_helpers_drive_forward_and_finish(
    project_index, lookahead_point, pursuit_velocity
):
    path = _straight_path(n=11, spacing=0.1)  # straight +X, ends at (1.0, 0)
    lookahead_m = 0.15
    pos_tol = 0.02

    idx = 0
    done = False
    # March the robot along the line in 0.1 m steps.
    for step_x in [i * 0.1 for i in range(12)]:
        pose = FakePose(step_x, 0.0, 0.0)
        point = (pose.position[0], pose.position[1])
        idx = project_index(path, point, idx, window=20)

        last = path[-1]
        dist_to_end = math.hypot(last[0] - point[0], last[1] - point[1])
        if idx >= len(path) - 2 and dist_to_end <= pos_tol:
            done = True
            break

        carrot = lookahead_point(path, idx, lookahead_m)
        vx, vy, wz = pursuit_velocity(pose, carrot, heading_target=0.0, speed=1.0)
        # Mid-path the command always points forward, no strafe / rotation.
        assert vx > 0.0
        assert vy == pytest.approx(0.0, abs=1e-9)
        assert wz == pytest.approx(0.0, abs=1e-9)

    assert done, "pursuit should report done when the robot reaches the path end"
    # Index ended monotonically at (or near) the last sample.
    assert idx >= len(path) - 2
