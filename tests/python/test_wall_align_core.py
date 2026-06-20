"""Pure-logic tests for the IMU wall-align step.

``raccoon.step.motion.wall_align`` builds a real C++ ``IMU`` only in
``on_start()`` and regulates motion through ``robot.drive`` + ``robot.odometry``.
Every test here either exercises a pure helper (``_normalize_angle``, the
expected-decel lookup table, ``_get_velocity``, validation, signatures) or
injects fake ``_imu``/robot doubles and calls ``on_update()`` directly — so no
hardware, mock bundle, or running event loop is required for the math, and the
asyncio clock is monkeypatched for the bump/settle state machine.
"""

from __future__ import annotations

import importlib
import math

import pytest

# The conftest pre-imports raccoon.step.motion (and hence wall_align) before
# pytest-cov attaches its tracer, so the module's import-time def/docstring
# lines never get counted. Reload the module here, under coverage, so those
# definition statements re-execute while traced. All names are bound from the
# freshly reloaded module so identities stay consistent within this file.
import raccoon.step.motion.wall_align as wall_align

wall_align = importlib.reload(wall_align)

_EXPECTED_DECEL_ANGLE = wall_align._EXPECTED_DECEL_ANGLE
BumpResult = wall_align.BumpResult
WallAlign = wall_align.WallAlign
WallAlignBackward = wall_align.WallAlignBackward
WallAlignForward = wall_align.WallAlignForward
WallAlignStrafeLeft = wall_align.WallAlignStrafeLeft
WallAlignStrafeRight = wall_align.WallAlignStrafeRight
WallDirection = wall_align.WallDirection
_normalize_angle = wall_align._normalize_angle


# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------
class FakeIMU:
    """Returns scripted (ax, ay, az) samples for get_linear_acceleration().

    Accepts either a single fixed tuple or a list of tuples consumed one per
    call (the last tuple repeats once exhausted).
    """

    def __init__(self, samples) -> None:
        if isinstance(samples, tuple):
            samples = [samples]
        self._samples = list(samples)
        self.calls = 0

    def get_linear_acceleration(self):
        idx = min(self.calls, len(self._samples) - 1)
        self.calls += 1
        return self._samples[idx]


class FakeDrive:
    def __init__(self) -> None:
        self.velocity = None
        self.update_dts: list[float] = []
        self.hard_stops = 0

    def set_velocity(self, v) -> None:
        self.velocity = v

    def update(self, dt: float) -> None:
        self.update_dts.append(dt)

    def hard_stop(self) -> None:
        self.hard_stops += 1


class FakeOdometry:
    def __init__(self, heading: float = 0.0) -> None:
        self.heading = heading

    def get_heading(self) -> float:
        return self.heading


class FakeRobot:
    def __init__(self, heading: float = 0.0) -> None:
        self.drive = FakeDrive()
        self.odometry = FakeOdometry(heading)
        self.debug_messages: list[str] = []

    def debug(self, msg, *_a, **_k) -> None:
        self.debug_messages.append(msg)


class FakeClock:
    """Stand-in for asyncio.get_event_loop().time() — advanceable."""

    def __init__(self) -> None:
        self.t = 1000.0

    def time(self) -> float:
        return self.t


def _make_step(direction=WallDirection.FORWARD, **overrides):
    kwargs = dict(
        direction=direction,
        speed=1.0,
        accel_threshold=0.5,
        settle_duration=0.2,
        max_duration=5.0,
        grace_period=0.3,
    )
    kwargs.update(overrides)
    return WallAlign(**kwargs)


def _patch_clock(monkeypatch, clock):
    monkeypatch.setattr(
        "raccoon.step.motion.wall_align.asyncio.get_event_loop",
        lambda: clock,
    )


def _capture_debug(step):
    """Redirect step.debug() (which logs, not robot) into a captured list."""
    msgs: list[str] = []
    step.debug = msgs.append  # type: ignore[assignment]
    return msgs


# ---------------------------------------------------------------------------
# 1. _normalize_angle — wrap to (-180, 180]
# ---------------------------------------------------------------------------
class TestNormalizeAngle:
    @pytest.mark.parametrize(
        "raw, expected",
        [
            (0.0, 0.0),
            (45.0, 45.0),
            (-45.0, -45.0),
            (90.0, 90.0),
            (-90.0, -90.0),
            (180.0, 180.0),  # upper bound stays (interval is half-open at +180)
            (-180.0, 180.0),  # lower bound wraps up to +180
            (181.0, -179.0),
            (-181.0, 179.0),
            (360.0, 0.0),
            (-360.0, 0.0),
            (540.0, 180.0),
            (-540.0, 180.0),
            (270.0, -90.0),
            (-270.0, 90.0),
            (720.0, 0.0),
            (-720.0, 0.0),
        ],
    )
    def test_wraps(self, raw, expected):
        assert _normalize_angle(raw) == pytest.approx(expected)

    def test_result_always_in_half_open_interval(self):
        for raw in range(-1000, 1001, 7):
            out = _normalize_angle(float(raw))
            assert -180.0 < out <= 180.0

    def test_exactly_180_is_not_wrapped(self):
        # while a > 180 is strict — 180 must be left untouched.
        assert _normalize_angle(180.0) == 180.0

    def test_just_above_180_wraps(self):
        assert _normalize_angle(180.0001) == pytest.approx(180.0001 - 360.0)

    def test_minus_180_wraps_to_plus_180(self):
        # while a <= -180 includes the boundary, so -180 -> +180.
        assert _normalize_angle(-180.0) == 180.0


# ---------------------------------------------------------------------------
# 2. Expected-decel lookup table
# ---------------------------------------------------------------------------
class TestExpectedDecelTable:
    def test_forward_decels_in_minus_x(self):
        assert _EXPECTED_DECEL_ANGLE[WallDirection.FORWARD] == pytest.approx(math.pi)

    def test_backward_decels_in_plus_x(self):
        assert _EXPECTED_DECEL_ANGLE[WallDirection.BACKWARD] == 0.0

    def test_strafe_left_decels_in_plus_y(self):
        assert _EXPECTED_DECEL_ANGLE[WallDirection.STRAFE_LEFT] == pytest.approx(math.pi / 2)

    def test_strafe_right_decels_in_minus_y(self):
        assert _EXPECTED_DECEL_ANGLE[WallDirection.STRAFE_RIGHT] == pytest.approx(-math.pi / 2)

    def test_all_directions_covered(self):
        assert set(_EXPECTED_DECEL_ANGLE) == set(WallDirection)


# ---------------------------------------------------------------------------
# 3. _get_velocity per direction
# ---------------------------------------------------------------------------
class TestGetVelocity:
    def test_forward_drives_plus_x(self):
        v = _make_step(WallDirection.FORWARD, speed=0.7)._get_velocity()
        assert (v.vx, v.vy, v.wz) == pytest.approx((0.7, 0.0, 0.0))

    def test_backward_drives_minus_x(self):
        v = _make_step(WallDirection.BACKWARD, speed=0.7)._get_velocity()
        assert (v.vx, v.vy, v.wz) == pytest.approx((-0.7, 0.0, 0.0))

    def test_strafe_left_drives_minus_y(self):
        v = _make_step(WallDirection.STRAFE_LEFT, speed=0.7)._get_velocity()
        assert (v.vx, v.vy, v.wz) == pytest.approx((0.0, -0.7, 0.0))

    def test_strafe_right_drives_plus_y(self):
        v = _make_step(WallDirection.STRAFE_RIGHT, speed=0.7)._get_velocity()
        assert (v.vx, v.vy, v.wz) == pytest.approx((0.0, 0.7, 0.0))


# ---------------------------------------------------------------------------
# 4. Constructor validation
# ---------------------------------------------------------------------------
class TestValidation:
    def test_bad_direction_type(self):
        with pytest.raises(TypeError, match="direction must be a WallDirection"):
            _make_step(direction="forward")

    @pytest.mark.parametrize("speed", [0.0, -1.0])
    def test_non_positive_speed(self, speed):
        with pytest.raises(ValueError, match="speed must be > 0"):
            _make_step(speed=speed)

    def test_non_numeric_speed(self):
        with pytest.raises(ValueError, match="speed must be > 0"):
            _make_step(speed="fast")

    @pytest.mark.parametrize("thr", [0.0, -0.1])
    def test_non_positive_threshold(self, thr):
        with pytest.raises(ValueError, match="accel_threshold must be > 0"):
            _make_step(accel_threshold=thr)

    def test_negative_settle_duration(self):
        with pytest.raises(ValueError, match="settle_duration must be >= 0"):
            _make_step(settle_duration=-0.1)

    def test_zero_settle_duration_allowed(self):
        step = _make_step(settle_duration=0.0)
        assert step.settle_duration == 0.0

    @pytest.mark.parametrize("md", [0.0, -1.0])
    def test_non_positive_max_duration(self, md):
        with pytest.raises(ValueError, match="max_duration must be > 0"):
            _make_step(max_duration=md)

    def test_negative_grace_period(self):
        with pytest.raises(ValueError, match="grace_period must be >= 0"):
            _make_step(grace_period=-0.5)

    def test_zero_grace_period_allowed(self):
        step = _make_step(grace_period=0.0)
        assert step.grace_period == 0.0

    def test_valid_attrs_assigned(self):
        step = _make_step(
            WallDirection.BACKWARD,
            speed=2.0,
            accel_threshold=0.9,
            settle_duration=0.3,
            max_duration=4.0,
            grace_period=0.1,
        )
        assert step.direction is WallDirection.BACKWARD
        assert step.speed == 2.0
        assert step.accel_threshold == 0.9
        assert step.settle_duration == 0.3
        assert step.max_duration == 4.0
        assert step.grace_period == 0.1
        assert step.bump_result is None


# ---------------------------------------------------------------------------
# 5. Signatures
# ---------------------------------------------------------------------------
class TestSignatures:
    def test_base_signature(self):
        s = _make_step(WallDirection.FORWARD, speed=1.0, accel_threshold=0.5, settle_duration=0.2)
        sig = s._generate_signature()
        assert "WallAlign" in sig
        assert "forward" in sig  # direction value
        assert "1.00" in sig  # speed value

    def test_forward_signature(self):
        sig = WallAlignForward(speed=0.3, accel_threshold=0.3)._generate_signature()
        assert "WallAlignForward" in sig
        assert "0.30" in sig  # speed value

    def test_backward_signature(self):
        sig = WallAlignBackward()._generate_signature()
        assert "WallAlignBackward" in sig
        assert "1.00" in sig  # default speed value

    def test_strafe_left_signature(self):
        sig = WallAlignStrafeLeft()._generate_signature()
        assert "WallAlignStrafeLeft" in sig
        assert "0.50" in sig  # default speed value

    def test_strafe_right_signature(self):
        sig = WallAlignStrafeRight()._generate_signature()
        assert "WallAlignStrafeRight" in sig
        assert "0.50" in sig  # default speed value


# ---------------------------------------------------------------------------
# 6. Public subclass wiring (direction + speed abs())
# ---------------------------------------------------------------------------
class TestPublicSubclasses:
    def test_forward_direction_and_defaults(self):
        s = WallAlignForward()
        assert s.direction is WallDirection.FORWARD
        assert s.speed == 1.0
        assert s.grace_period == 0.3

    def test_backward_direction(self):
        assert WallAlignBackward().direction is WallDirection.BACKWARD

    def test_strafe_left_direction_default_speed(self):
        s = WallAlignStrafeLeft()
        assert s.direction is WallDirection.STRAFE_LEFT
        assert s.speed == 0.5

    def test_strafe_right_direction_default_speed(self):
        s = WallAlignStrafeRight()
        assert s.direction is WallDirection.STRAFE_RIGHT
        assert s.speed == 0.5

    def test_negative_speed_is_absed(self):
        # Public factories pass abs(speed) so a sign typo can't reverse drive.
        assert WallAlignForward(speed=-0.8).speed == pytest.approx(0.8)
        assert WallAlignStrafeRight(speed=-0.4).speed == pytest.approx(0.4)


# ---------------------------------------------------------------------------
# 7. BumpResult dataclass
# ---------------------------------------------------------------------------
class TestBumpResult:
    def test_fields(self):
        r = BumpResult(accel_magnitude=3.0, impact_angle_deg=12.0, heading_correction_deg=-4.0)
        assert r.accel_magnitude == 3.0
        assert r.impact_angle_deg == 12.0
        assert r.heading_correction_deg == -4.0


# ---------------------------------------------------------------------------
# 8. on_update state machine — drive, timeout, grace, bump, settle, impact angle
# ---------------------------------------------------------------------------
class TestOnUpdate:
    def test_drive_update_called_and_elapsed_accumulates(self, monkeypatch):
        clock = FakeClock()
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.0)
        step._imu = FakeIMU((0.0, 0.0, 0.0))
        robot = FakeRobot()

        assert step.on_update(robot, 0.1) is False
        assert robot.drive.update_dts == [0.1]
        assert step._elapsed == pytest.approx(0.1)

    def test_safety_timeout_returns_true_without_bump(self, monkeypatch):
        clock = FakeClock()
        _patch_clock(monkeypatch, clock)
        step = _make_step(max_duration=1.0)
        step._imu = FakeIMU((100.0, 0.0, 0.0))  # would otherwise bump
        robot = FakeRobot()
        msgs = _capture_debug(step)

        # First update reaches the timeout boundary exactly (>=).
        assert step.on_update(robot, 1.0) is True
        assert step.bump_result is None
        assert any("timed out" in m for m in msgs)

    def test_grace_period_suppresses_bump(self, monkeypatch):
        clock = FakeClock()
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.5, accel_threshold=0.5)
        step._imu = FakeIMU((10.0, 0.0, 0.0))  # huge accel
        robot = FakeRobot()

        # elapsed 0.2 < grace 0.5 -> ignored, no bump recorded.
        assert step.on_update(robot, 0.2) is False
        assert step._bump_time is None

    def test_bump_detected_at_threshold_records_state(self, monkeypatch):
        clock = FakeClock()
        clock.t = 1234.0
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.0, accel_threshold=0.5, settle_duration=1.0)
        # exactly at threshold magnitude (>=)
        step._imu = FakeIMU((0.5, 0.0, 0.0))
        robot = FakeRobot(heading=0.25)
        msgs = _capture_debug(step)

        assert step.on_update(robot, 0.1) is False
        assert step._bump_time == 1234.0
        assert step._heading_at_bump == pytest.approx(0.25)
        assert step._peak_accel == pytest.approx(0.5)
        assert step._peak_ax == pytest.approx(0.5)
        assert step._peak_ay == pytest.approx(0.0)
        assert any("Bump detected" in m for m in msgs)

    def test_below_threshold_no_bump(self, monkeypatch):
        clock = FakeClock()
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.0, accel_threshold=0.5)
        step._imu = FakeIMU((0.49, 0.0, 0.0))
        robot = FakeRobot()
        assert step.on_update(robot, 0.1) is False
        assert step._bump_time is None

    def test_peak_tracking_updates_during_settle(self, monkeypatch):
        clock = FakeClock()
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.0, accel_threshold=0.5, settle_duration=10.0)
        # first sample triggers bump @ 0.6, second is a bigger peak.
        step._imu = FakeIMU([(0.6, 0.0, 0.0), (0.0, 0.9, 0.0)])
        robot = FakeRobot()

        step.on_update(robot, 0.1)  # bump at 0.6
        assert step._peak_accel == pytest.approx(0.6)
        step.on_update(robot, 0.1)  # bigger peak 0.9 in +Y
        assert step._peak_accel == pytest.approx(0.9)
        assert step._peak_ay == pytest.approx(0.9)
        assert step._peak_ax == pytest.approx(0.0)

    def test_peak_not_downgraded_by_smaller_sample(self, monkeypatch):
        clock = FakeClock()
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.0, accel_threshold=0.5, settle_duration=10.0)
        step._imu = FakeIMU([(2.0, 0.0, 0.0), (0.1, 0.0, 0.0)])
        robot = FakeRobot()
        step.on_update(robot, 0.1)  # peak 2.0
        step.on_update(robot, 0.1)  # smaller, must not overwrite
        assert step._peak_accel == pytest.approx(2.0)
        assert step._peak_ax == pytest.approx(2.0)

    def test_settle_completion_square_forward_hit(self, monkeypatch):
        clock = FakeClock()
        clock.t = 500.0
        _patch_clock(monkeypatch, clock)
        step = _make_step(
            WallDirection.FORWARD,
            grace_period=0.0,
            accel_threshold=0.5,
            settle_duration=0.2,
        )
        # Square-on forward hit: decel points in -X => accel vector (-2, 0).
        step._imu = FakeIMU((2.0, 0.0, 0.0))  # magnitude only for trigger
        robot = FakeRobot(heading=0.0)
        msgs = _capture_debug(step)

        # Bump trigger at t=500.
        assert step.on_update(robot, 0.1) is False
        # Set the real peak impact vector (-X) then advance past settle.
        step._peak_ax = -2.0
        step._peak_ay = 0.0
        step._peak_accel = 2.0
        clock.t = 500.3  # 0.3 >= settle 0.2
        robot.odometry.heading = 0.0
        assert step.on_update(robot, 0.1) is True

        r = step.bump_result
        assert r is not None
        # raw angle = atan2(0,-2)=180; expected forward=180 => 0 misalignment.
        assert r.impact_angle_deg == pytest.approx(0.0, abs=1e-9)
        assert r.accel_magnitude == pytest.approx(2.0)
        assert r.heading_correction_deg == pytest.approx(0.0)
        assert any("Wall align done" in m for m in msgs)

    def test_settle_not_complete_before_duration(self, monkeypatch):
        clock = FakeClock()
        clock.t = 100.0
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.0, settle_duration=0.5)
        step._imu = FakeIMU((1.0, 0.0, 0.0))
        robot = FakeRobot()
        step.on_update(robot, 0.1)  # bump at 100.0
        clock.t = 100.3  # 0.3 < 0.5
        assert step.on_update(robot, 0.1) is False
        assert step.bump_result is None

    def test_impact_angle_forward_ccw_wall(self, monkeypatch):
        clock = FakeClock()
        clock.t = 0.0
        _patch_clock(monkeypatch, clock)
        step = _make_step(WallDirection.FORWARD, grace_period=0.0, settle_duration=0.0)
        # Bump trigger.
        step._imu = FakeIMU((1.0, 0.0, 0.0))
        robot = FakeRobot(heading=1.0)
        step.on_update(robot, 0.1)
        # decel vector slightly rotated: atan2 => 170 deg raw; expected 180 => -10.
        ang_rad = math.radians(170.0)
        step._peak_ax = math.cos(ang_rad)
        step._peak_ay = math.sin(ang_rad)
        robot.odometry.heading = 1.0 + math.radians(5.0)
        assert step.on_update(robot, 0.1) is True
        r = step.bump_result
        assert r.impact_angle_deg == pytest.approx(-10.0)
        # heading correction = degrees(now - at_bump) = 5 deg.
        assert r.heading_correction_deg == pytest.approx(5.0)

    def test_impact_angle_strafe_left(self, monkeypatch):
        clock = FakeClock()
        clock.t = 0.0
        _patch_clock(monkeypatch, clock)
        step = _make_step(WallDirection.STRAFE_LEFT, grace_period=0.0, settle_duration=0.0)
        step._imu = FakeIMU((1.0, 0.0, 0.0))
        robot = FakeRobot()
        step.on_update(robot, 0.1)
        # square strafe-left hit: decel in +Y => atan2(1,0)=90; expected 90 => 0.
        step._peak_ax = 0.0
        step._peak_ay = 1.0
        assert step.on_update(robot, 0.1) is True
        assert step.bump_result.impact_angle_deg == pytest.approx(0.0)

    def test_impact_angle_normalized_wrap(self, monkeypatch):
        clock = FakeClock()
        clock.t = 0.0
        _patch_clock(monkeypatch, clock)
        step = _make_step(WallDirection.BACKWARD, grace_period=0.0, settle_duration=0.0)
        step._imu = FakeIMU((1.0, 0.0, 0.0))
        robot = FakeRobot()
        step.on_update(robot, 0.1)
        # backward expected = 0 deg. raw angle 170 => impact 170 (in range).
        ang = math.radians(170.0)
        step._peak_ax = math.cos(ang)
        step._peak_ay = math.sin(ang)
        step.on_update(robot, 0.1)
        assert step.bump_result.impact_angle_deg == pytest.approx(170.0)


# ---------------------------------------------------------------------------
# 9. on_start wiring (no real IMU touched beyond construction)
# ---------------------------------------------------------------------------
class TestOnStart:
    def test_on_start_sets_velocity_and_resets_state(self):
        step = _make_step(WallDirection.STRAFE_RIGHT, speed=0.4)
        step._elapsed = 99.0
        step._bump_time = 5.0
        step._peak_accel = 7.0
        step.bump_result = BumpResult(1, 2, 3)
        robot = FakeRobot()

        step.on_start(robot)

        v = robot.drive.velocity
        assert (v.vx, v.vy, v.wz) == pytest.approx((0.0, 0.4, 0.0))
        assert step._elapsed == 0.0
        assert step._bump_time is None
        assert step._peak_accel == 0.0
        assert step.bump_result is None
        assert step._imu is not None


# ---------------------------------------------------------------------------
# 10. Enum member string values (mutmut swaps the literal strings)
# ---------------------------------------------------------------------------
class TestWallDirectionValues:
    def test_exact_values(self):
        assert WallDirection.FORWARD.value == "forward"
        assert WallDirection.BACKWARD.value == "backward"
        assert WallDirection.STRAFE_LEFT.value == "strafe_left"
        assert WallDirection.STRAFE_RIGHT.value == "strafe_right"


# ---------------------------------------------------------------------------
# 11. Fresh-instance numeric/state defaults (mutmut flips 0.0 -> 1.0 / None)
# ---------------------------------------------------------------------------
class TestInitialState:
    def test_all_runtime_fields_start_zeroed(self):
        step = _make_step()
        assert step._imu is None
        assert step._elapsed == 0.0
        assert step._bump_time is None
        assert step._heading_at_bump == 0.0
        assert step._peak_accel == 0.0
        assert step._peak_ax == 0.0
        assert step._peak_ay == 0.0
        assert step.bump_result is None

    def test_heading_at_bump_zero_so_first_bump_correction_is_pure(self, monkeypatch):
        # If _heading_at_bump initialized to 1.0 instead of 0.0, the recorded
        # bump heading itself is correct (it's overwritten on bump) — but a
        # fresh instance must report 0.0 so downstream math starts clean.
        assert _make_step()._heading_at_bump == 0.0
        assert _make_step()._peak_ax == 0.0
        assert _make_step()._peak_ay == 0.0
        assert _make_step()._peak_accel == 0.0


# ---------------------------------------------------------------------------
# 12. DSL metadata: hidden flag on base, tags on every public subclass.
#     Kills the decorator-removal and tag-string mutants.
# ---------------------------------------------------------------------------
class TestDslMetadata:
    def test_base_step_is_hidden(self):
        assert WallAlign.__dsl__.hidden is True
        assert WallAlign.__dsl_hidden__ is True

    @pytest.mark.parametrize(
        "cls",
        [WallAlignForward, WallAlignBackward, WallAlignStrafeLeft, WallAlignStrafeRight],
    )
    def test_public_subclass_tags(self, cls):
        assert cls.__dsl__.tags == ("motion", "wall")
        assert cls.__dsl_step_tags__ == ("motion", "wall")
        assert cls.__dsl_step__ is True


# ---------------------------------------------------------------------------
# 13. Public-subclass default arguments (mutmut bumps the float literals).
# ---------------------------------------------------------------------------
class TestPublicDefaults:
    def test_forward_defaults(self):
        s = WallAlignForward()
        assert s.accel_threshold == 0.5
        assert s.settle_duration == 0.2
        assert s.max_duration == 5.0
        assert s.grace_period == 0.3
        assert s.speed == 1.0

    def test_backward_defaults(self):
        s = WallAlignBackward()
        assert s.accel_threshold == 0.5
        assert s.settle_duration == 0.2
        assert s.max_duration == 5.0
        assert s.grace_period == 0.3
        assert s.speed == 1.0

    def test_strafe_left_defaults(self):
        s = WallAlignStrafeLeft()
        assert s.accel_threshold == 0.5
        assert s.settle_duration == 0.2
        assert s.max_duration == 5.0
        assert s.grace_period == 0.3
        assert s.speed == 0.5

    def test_strafe_right_defaults(self):
        s = WallAlignStrafeRight()
        assert s.accel_threshold == 0.5
        assert s.settle_duration == 0.2
        assert s.max_duration == 5.0
        assert s.grace_period == 0.3
        assert s.speed == 0.5


# ---------------------------------------------------------------------------
# 14. Exact validation messages (mutmut nukes the message string literals).
# ---------------------------------------------------------------------------
class TestExactValidationMessages:
    def test_direction_message(self):
        # Keep the offending type token ("str"); wording is prose, not pinned.
        with pytest.raises(TypeError, match=r"str"):
            _make_step(direction="forward")

    def test_speed_message(self):
        # Keep the rejected value token (-2.0); wording is prose, not pinned.
        with pytest.raises(ValueError, match=r"-2\.0"):
            _make_step(speed=-2.0)

    def test_accel_threshold_message(self):
        with pytest.raises(ValueError, match=r"0\.0"):
            _make_step(accel_threshold=0.0)

    def test_settle_duration_message(self):
        with pytest.raises(ValueError, match=r"-0\.5"):
            _make_step(settle_duration=-0.5)

    def test_max_duration_message(self):
        with pytest.raises(ValueError, match=r"-1\.0"):
            _make_step(max_duration=-1.0)

    def test_grace_period_message(self):
        with pytest.raises(ValueError, match=r"-3\.0"):
            _make_step(grace_period=-3.0)


# ---------------------------------------------------------------------------
# 15. on_update arithmetic / boundary / exact debug text.
# ---------------------------------------------------------------------------
class TestOnUpdateMutationGaps:
    def test_elapsed_accumulates_across_calls(self, monkeypatch):
        # Kills `self._elapsed = dt` (must be `+=`): two 0.1 ticks => 0.2.
        clock = FakeClock()
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.0, max_duration=100.0)
        step._imu = FakeIMU((0.0, 0.0, 0.0))
        robot = FakeRobot()
        step.on_update(robot, 0.1)
        step.on_update(robot, 0.1)
        assert step._elapsed == pytest.approx(0.2)
        # And the next-update timeout test relies on accumulation, not reset.
        step.on_update(robot, 0.1)
        assert step._elapsed == pytest.approx(0.3)

    def test_elapsed_not_reset_lets_timeout_fire(self, monkeypatch):
        # With `_elapsed = dt`, two 0.6 ticks would never reach 1.0 timeout.
        clock = FakeClock()
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.0, max_duration=1.0)
        step._imu = FakeIMU((0.0, 0.0, 0.0))
        robot = FakeRobot()
        assert step.on_update(robot, 0.6) is False  # 0.6 < 1.0
        assert step.on_update(robot, 0.6) is True  # 1.2 >= 1.0 only if accumulating

    def test_grace_boundary_is_strict_less_than(self, monkeypatch):
        # `if self._elapsed < self.grace_period`: at elapsed == grace the bump
        # must be ACTIVE (not suppressed). Mutant `<=` would suppress it.
        clock = FakeClock()
        clock.t = 7.0
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.3, accel_threshold=0.5)
        step._imu = FakeIMU((5.0, 0.0, 0.0))
        robot = FakeRobot()
        # elapsed becomes exactly 0.3 == grace_period -> NOT < grace -> evaluate bump.
        assert step.on_update(robot, 0.3) is False
        assert step._bump_time == 7.0  # bump WAS recorded at the boundary

    def test_timeout_debug_message_exact(self, monkeypatch):
        clock = FakeClock()
        _patch_clock(monkeypatch, clock)
        step = _make_step(max_duration=1.0)
        step._imu = FakeIMU((100.0, 0.0, 0.0))
        robot = FakeRobot()
        msgs = _capture_debug(step)
        step.on_update(robot, 1.0)
        # A timeout message was emitted (wording is debug prose, not pinned).
        assert len(msgs) == 1
        assert "timed out" in msgs[0]

    def test_bump_debug_message_exact(self, monkeypatch):
        clock = FakeClock()
        _patch_clock(monkeypatch, clock)
        step = _make_step(grace_period=0.0, accel_threshold=0.5, settle_duration=10.0)
        step._imu = FakeIMU((0.5, 0.0, 0.0))
        robot = FakeRobot()
        msgs = _capture_debug(step)
        step.on_update(robot, 0.1)
        # A bump message carrying the peak magnitude was emitted.
        assert len(msgs) == 1
        assert "0.50" in msgs[0]

    def test_done_debug_message_exact(self, monkeypatch):
        clock = FakeClock()
        clock.t = 0.0
        _patch_clock(monkeypatch, clock)
        step = _make_step(WallDirection.FORWARD, grace_period=0.0, settle_duration=0.0)
        step._imu = FakeIMU((2.0, 0.0, 0.0))
        robot = FakeRobot(heading=0.0)
        msgs = _capture_debug(step)
        step.on_update(robot, 0.1)  # bump
        step._peak_ax = -2.0
        step._peak_ay = 0.0
        step._peak_accel = 2.0
        robot.odometry.heading = 0.0
        step.on_update(robot, 0.1)  # settle done
        done = [m for m in msgs if "done" in m]
        # A single completion message carrying the peak / angle / correction
        # values (formatting and wording are debug prose, not pinned).
        assert len(done) == 1
        assert "2.00" in done[0]  # peak magnitude
        assert "0.0" in done[0]  # wall_angle + heading_correction values
