"""Spec-driven construction tests for the turn and arc motion steps.

Both ``raccoon.step.motion.turn`` and ``raccoon.step.motion.arc`` build their
native C++ motion controllers (``TurnMotion`` / ``ArcMotion``) only inside
``on_start()``.  Everything this suite cares about — argument validation, the
degrees -> radians conversion, the CCW(+)/CW(-) sign convention, the
centimetre -> metre radius conversion and the speed bound — happens in
``__init__`` and writes into the pure config objects (``TurnConfig`` /
``ArcMotionConfig``) or into ``lower_to_segments()``.  So no hardware, mock
bundle, or running event loop is needed here.

Every expected value below is derived INDEPENDENTLY from the docstrings and
unit conventions (degrees, centimetres, the sign rules) and computed by hand /
from first principles — never copied from the implementation's output.
"""

from __future__ import annotations

import importlib
import math

import pytest

import raccoon.step.motion.arc as _arc

# The conftest pre-imports raccoon.step.motion (and hence turn/arc) before
# pytest-cov attaches its tracer, so the modules' import-time def/docstring
# lines never get counted. Reload them here, under coverage, so those
# definition statements re-execute while traced. All names are bound from the
# freshly reloaded modules so identities stay consistent within this file.
import raccoon.step.motion.turn as _turn
from raccoon.step.condition import StopCondition

_turn = importlib.reload(_turn)
_arc = importlib.reload(_arc)

TurnLeft = _turn.TurnLeft
TurnRight = _turn.TurnRight
_ConditionalTurn = _turn._ConditionalTurn

Arc = _arc.Arc
DriveArc = _arc.DriveArc
DriveArcLeft = _arc.DriveArcLeft
DriveArcRight = _arc.DriveArcRight
StrafeArc = _arc.StrafeArc
StrafeArcLeft = _arc.StrafeArcLeft
StrafeArcRight = _arc.StrafeArcRight

# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------


class FakeCondition(StopCondition):
    """Minimal StopCondition stand-in; turn construction only type-checks it."""

    def __init__(self, *, triggers: bool = False) -> None:
        self.triggers = triggers
        self.started = False
        self.check_count = 0

    def start(self, robot) -> None:  # pragma: no cover - not exercised here
        self.started = True

    def check(self, robot) -> bool:  # pragma: no cover - not exercised here
        self.check_count += 1
        return self.triggers


class FakeTurnMotion:
    """Stand-in for the native ``TurnMotion`` used by an angle-based turn."""

    def __init__(self, finished: bool = False) -> None:
        self.updates: list[float] = []
        self._finished = finished
        self.started = False

    def start(self) -> None:
        self.started = True

    def update(self, dt: float) -> None:
        self.updates.append(dt)

    def is_finished(self) -> bool:
        return self._finished


class FakeDrive:
    def __init__(self) -> None:
        self.velocity = None
        self.update_dts: list[float] = []

    def set_velocity(self, v) -> None:
        self.velocity = v

    def update(self, dt: float) -> None:
        self.update_dts.append(dt)


class _AngularLimits:
    def __init__(self, max_velocity: float) -> None:
        self.max_velocity = max_velocity


class _PidConfig:
    def __init__(self, max_w: float) -> None:
        self.angular = _AngularLimits(max_w)


class FakeRobot:
    def __init__(self, max_w: float = 3.0) -> None:
        self.drive = FakeDrive()
        self.motion_pid_config = _PidConfig(max_w)


# Independently computed radian references (math.radians == deg * pi / 180).
_PI = math.pi
RAD = {
    1.0: _PI / 180.0,
    45.0: _PI / 4.0,
    90.0: _PI / 2.0,
    135.0: 3.0 * _PI / 4.0,
    180.0: _PI,
    270.0: 3.0 * _PI / 2.0,
    360.0: 2.0 * _PI,
}


# ===========================================================================
# turn.py — degrees > 0 validation
# ===========================================================================


@pytest.mark.parametrize("cls", [TurnLeft, TurnRight])
class TestTurnDegreesValidation:
    @pytest.mark.parametrize("bad", [0.0, -1.0, -90.0, -0.0001])
    def test_non_positive_degrees_rejected(self, cls, bad):
        # Docstring/spec: an angle turn needs a strictly positive magnitude.
        # 0 and negatives are invalid (sign carries direction, not magnitude).
        with pytest.raises(ValueError, match=r"degrees must be > 0"):
            cls(bad)

    def test_zero_is_boundary_and_rejected(self, cls):
        # Exactly 0 is the boundary; "> 0" is strict so 0 must raise.
        with pytest.raises(ValueError, match=r"> 0"):
            cls(0.0)

    @pytest.mark.parametrize("good", [0.0001, 1.0, 90.0, 360.0, 1000.0])
    def test_positive_degrees_accepted(self, cls, good):
        step = cls(good)
        assert step._degrees == pytest.approx(good)

    def test_neither_degrees_nor_until_raises(self, cls):
        with pytest.raises(ValueError, match=r"requires either 'degrees' or 'until'"):
            cls()

    @pytest.mark.parametrize("bad_type", ["90", [90], {"d": 1}])
    def test_non_numeric_degrees_type_error(self, cls, bad_type):
        with pytest.raises(TypeError, match=r"degrees must be a number"):
            cls(bad_type)

    def test_bool_degrees_accepted_as_number(self, cls):
        # bool is a subclass of int, so True (==1, > 0) is a valid number.
        step = cls(True)
        assert step._degrees is True


# ===========================================================================
# turn.py — speed bound (0.0, 1.0]
# ===========================================================================


@pytest.mark.parametrize("cls", [TurnLeft, TurnRight])
class TestTurnSpeedBound:
    @pytest.mark.parametrize("good", [0.0001, 0.5, 1.0])
    def test_speed_in_range_accepted(self, cls, good):
        step = cls(90, speed=good)
        assert step._speed == pytest.approx(good)

    def test_upper_bound_inclusive(self, cls):
        # spec: (0.0, 1.0] — 1.0 is allowed.
        assert cls(90, speed=1.0)._speed == pytest.approx(1.0)

    @pytest.mark.parametrize("bad", [0.0, -0.5, 1.0001, 2.0])
    def test_speed_out_of_range_rejected(self, cls, bad):
        # 0.0 (lower open bound) and anything > 1.0 must raise.
        with pytest.raises(ValueError, match=r"speed must be in"):
            cls(90, speed=bad)

    def test_lower_bound_is_open(self, cls):
        with pytest.raises(ValueError, match=r"\(0\.0, 1\.0\]"):
            cls(90, speed=0.0)

    @pytest.mark.parametrize("bad_type", ["1.0", None, [0.5]])
    def test_non_numeric_speed_type_error(self, cls, bad_type):
        with pytest.raises(TypeError, match=r"speed must be a number"):
            cls(90, speed=bad_type)

    def test_default_speed_is_one(self, cls):
        assert cls(90)._speed == pytest.approx(1.0)


# ===========================================================================
# turn.py — `until` validation and modes
# ===========================================================================


@pytest.mark.parametrize("cls", [TurnLeft, TurnRight])
class TestTurnUntil:
    def test_until_only_is_valid(self, cls):
        cond = FakeCondition()
        step = cls(until=cond)
        assert step._until is cond
        assert step._degrees is None

    def test_until_must_be_stop_condition(self, cls):
        with pytest.raises(TypeError, match=r"until must be a StopCondition"):
            cls(90, until=object())

    def test_degrees_and_until_together(self, cls):
        cond = FakeCondition()
        step = cls(90, until=cond)
        assert step._until is cond
        assert step._degrees == pytest.approx(90)


# ===========================================================================
# turn.py — sign convention: TurnLeft = CCW (+), TurnRight = CW (-)
# ===========================================================================


class TestTurnSignConvention:
    def test_turn_left_class_sign_is_positive(self):
        # CCW (left) is the positive mathematical rotation direction.
        assert TurnLeft._sign == pytest.approx(1.0)

    def test_turn_right_class_sign_is_negative(self):
        # CW (right) is negative.
        assert TurnRight._sign == pytest.approx(-1.0)

    def test_base_default_sign_is_positive(self):
        assert _ConditionalTurn._sign == pytest.approx(1.0)


# ===========================================================================
# turn.py — degrees -> radians conversion + sign, via lower_to_segments()
#
# lower_to_segments writes  angle_rad = sign * radians(degrees),
# which is exactly the value on_start() pushes into TurnConfig.target_angle_rad.
# Asserting it here exercises the conversion math without building TurnMotion.
# ===========================================================================


class TestTurnLowerToSegmentsMath:
    @pytest.mark.parametrize("deg", [1.0, 45.0, 90.0, 135.0, 180.0, 270.0, 360.0])
    def test_left_angle_is_positive_radians(self, deg):
        seg = TurnLeft(deg).lower_to_segments()[0]
        assert seg.angle_rad == pytest.approx(RAD[deg])  # +1 * radians(deg)
        assert seg.angle_rad > 0.0
        assert seg.sign == pytest.approx(1.0)

    @pytest.mark.parametrize("deg", [1.0, 45.0, 90.0, 135.0, 180.0, 270.0, 360.0])
    def test_right_angle_is_negative_radians(self, deg):
        seg = TurnRight(deg).lower_to_segments()[0]
        assert seg.angle_rad == pytest.approx(-RAD[deg])  # -1 * radians(deg)
        assert seg.angle_rad < 0.0
        assert seg.sign == pytest.approx(-1.0)

    def test_left_and_right_are_exact_opposites(self):
        left = TurnLeft(73.0).lower_to_segments()[0].angle_rad
        right = TurnRight(73.0).lower_to_segments()[0].angle_rad
        assert left == pytest.approx(-right)

    def test_90_degree_left_is_quarter_turn(self):
        # Sanity anchor: a quarter turn left is +pi/2, NOT degrees, NOT pi/4.
        seg = TurnLeft(90).lower_to_segments()[0]
        assert seg.angle_rad == pytest.approx(1.5707963267948966)

    def test_segment_kind_and_speed_and_known_endpoint(self):
        seg = TurnLeft(90, speed=0.5).lower_to_segments()[0]
        assert seg.kind == "turn"
        assert seg.speed_scale == pytest.approx(0.5)
        assert seg.has_known_endpoint is True
        assert seg.condition is None

    def test_until_segment_has_no_angle_and_unknown_endpoint(self):
        cond = FakeCondition()
        seg = TurnRight(until=cond).lower_to_segments()[0]
        assert seg.angle_rad is None
        assert seg.has_known_endpoint is False
        assert seg.condition is cond
        assert seg.sign == pytest.approx(-1.0)

    def test_lower_to_segments_returns_single_segment(self):
        assert len(TurnLeft(30).lower_to_segments()) == 1


# ===========================================================================
# turn.py — signature is informative but NOT prose-pinned
# ===========================================================================


class TestTurnSignature:
    def test_angle_signature_mentions_degrees_value(self):
        sig = TurnLeft(90)._generate_signature()
        assert "90" in sig
        assert "until" not in sig

    def test_until_signature_mentions_until(self):
        sig = TurnRight(until=FakeCondition())._generate_signature()
        assert "until" in sig


# ===========================================================================
# arc.py — radius cm -> m conversion
# ===========================================================================


_ARC_PUBLIC = [DriveArcLeft, DriveArcRight, StrafeArcLeft, StrafeArcRight]
_ARC_ALL = _ARC_PUBLIC + [DriveArc, StrafeArc]


@pytest.mark.parametrize("cls", _ARC_ALL)
class TestArcRadiusConversion:
    @pytest.mark.parametrize(
        ("radius_cm", "expected_m"),
        [
            (30.0, 0.30),  # 30 cm = 0.30 m
            (50.0, 0.50),
            (100.0, 1.0),  # 1 m
            (1.0, 0.01),
            (0.0, 0.0),  # degenerate radius still divides by 100
            (250.0, 2.5),
        ],
    )
    def test_radius_cm_divided_by_100(self, cls, radius_cm, expected_m):
        step = cls(radius_cm=radius_cm, degrees=90)
        assert step.config.radius_m == pytest.approx(expected_m)

    def test_radius_stored_in_cm_attr(self, cls):
        step = cls(radius_cm=42.0, degrees=90)
        assert step._radius_cm == pytest.approx(42.0)


# ===========================================================================
# arc.py — degrees -> radians + per-direction sign
#
# Spec from docstrings/comments:
#   DriveArcLeft / StrafeArcLeft  -> +radians(degrees)  (CCW / left)
#   DriveArcRight / StrafeArcRight -> -radians(degrees)  (CW / right)
#   DriveArc / StrafeArc (hidden) -> +radians(degrees)  (signed by caller)
# ===========================================================================


_LEFT_OR_SIGNED = [DriveArcLeft, StrafeArcLeft, DriveArc, StrafeArc]
_RIGHT = [DriveArcRight, StrafeArcRight]


class TestArcAngleSign:
    @pytest.mark.parametrize("cls", _LEFT_OR_SIGNED)
    @pytest.mark.parametrize("deg", [1.0, 45.0, 90.0, 180.0, 360.0])
    def test_left_and_signed_positive_radians(self, cls, deg):
        step = cls(radius_cm=30, degrees=deg)
        assert step.config.arc_angle_rad == pytest.approx(RAD[deg])
        assert step.config.arc_angle_rad > 0.0

    @pytest.mark.parametrize("cls", _RIGHT)
    @pytest.mark.parametrize("deg", [1.0, 45.0, 90.0, 180.0, 360.0])
    def test_right_negative_radians(self, cls, deg):
        step = cls(radius_cm=30, degrees=deg)
        assert step.config.arc_angle_rad == pytest.approx(-RAD[deg])
        assert step.config.arc_angle_rad < 0.0

    def test_left_and_right_are_exact_opposites(self):
        left = DriveArcLeft(radius_cm=30, degrees=85).config.arc_angle_rad
        right = DriveArcRight(radius_cm=30, degrees=85).config.arc_angle_rad
        assert left == pytest.approx(-right)

    def test_quarter_circle_left_is_half_pi(self):
        # 90 deg left arc -> +pi/2 (NOT pi/4, NOT 90).
        assert DriveArcLeft(radius_cm=30, degrees=90).config.arc_angle_rad == pytest.approx(
            1.5707963267948966
        )

    def test_signed_driver_negative_degrees_goes_cw(self):
        # DriveArc is direction-by-sign: negative degrees => negative (CW).
        step = DriveArc(radius_cm=30, degrees=-90)
        assert step.config.arc_angle_rad == pytest.approx(-_PI / 2.0)

    def test_signed_driver_positive_degrees_goes_ccw(self):
        step = StrafeArc(radius_cm=30, degrees=90)
        assert step.config.arc_angle_rad == pytest.approx(_PI / 2.0)

    def test_right_with_negative_degrees_double_negates(self):
        # DriveArcRight applies -radians(degrees); radians(-90) = -pi/2,
        # negated => +pi/2.  (Documents the literal spec, no special-casing.)
        step = DriveArcRight(radius_cm=30, degrees=-90)
        assert step.config.arc_angle_rad == pytest.approx(_PI / 2.0)


# ===========================================================================
# arc.py — speed wiring + lateral flag (strafe vs drive)
# ===========================================================================


class TestArcSpeedAndLateral:
    @pytest.mark.parametrize("cls", _ARC_ALL)
    def test_speed_default_is_one(self, cls):
        assert cls(radius_cm=30, degrees=90).config.speed_scale == pytest.approx(1.0)

    @pytest.mark.parametrize("cls", _ARC_ALL)
    def test_speed_passthrough(self, cls):
        step = cls(radius_cm=30, degrees=90, speed=0.5)
        assert step.config.speed_scale == pytest.approx(0.5)
        assert step._speed == pytest.approx(0.5)

    @pytest.mark.parametrize("cls", [DriveArcLeft, DriveArcRight, DriveArc])
    def test_drive_arcs_are_not_lateral(self, cls):
        assert cls(radius_cm=30, degrees=90).config.lateral is False

    @pytest.mark.parametrize("cls", [StrafeArcLeft, StrafeArcRight, StrafeArc])
    def test_strafe_arcs_are_lateral(self, cls):
        assert cls(radius_cm=30, degrees=90).config.lateral is True


# ===========================================================================
# arc.py — lower_to_segments mirrors the config (drive vs strafe lateral)
# ===========================================================================


class TestArcLowerToSegments:
    def test_drive_left_segment(self):
        seg = DriveArcLeft(radius_cm=50, degrees=90, speed=0.5).lower_to_segments()[0]
        assert seg.kind == "arc"
        assert seg.radius_m == pytest.approx(0.5)
        assert seg.arc_angle_rad == pytest.approx(_PI / 2.0)
        assert seg.speed_scale == pytest.approx(0.5)
        assert seg.lateral is False
        assert seg.has_known_endpoint is True

    def test_strafe_right_segment(self):
        seg = StrafeArcRight(radius_cm=20, degrees=180).lower_to_segments()[0]
        assert seg.radius_m == pytest.approx(0.2)
        assert seg.arc_angle_rad == pytest.approx(-_PI)
        assert seg.lateral is True

    def test_single_segment_returned(self):
        assert len(DriveArc(radius_cm=10, degrees=30).lower_to_segments()) == 1


# ===========================================================================
# arc.py — signatures (informative, not prose-pinned)
# ===========================================================================


class TestArcSignatures:
    @pytest.mark.parametrize(
        ("cls", "name"),
        [
            (DriveArcLeft, "DriveArcLeft"),
            (DriveArcRight, "DriveArcRight"),
            (DriveArc, "DriveArc"),
            (StrafeArcLeft, "StrafeArcLeft"),
            (StrafeArcRight, "StrafeArcRight"),
            (StrafeArc, "StrafeArc"),
        ],
    )
    def test_signature_reports_class_radius_degrees(self, cls, name):
        sig = cls(radius_cm=30, degrees=90, speed=0.5)._generate_signature()
        assert name in sig
        assert "30" in sig  # radius cm value
        assert "90" in sig  # degrees value

    def test_base_arc_signature_reports_radius_in_cm(self):
        # Base Arc.__init__ takes a config directly; signature recomputes
        # radius_m * 100 -> cm and arc_angle_rad -> degrees.
        from raccoon.motion import ArcMotionConfig

        cfg = ArcMotionConfig()
        cfg.radius_m = 0.30
        cfg.arc_angle_rad = _PI / 2.0
        cfg.speed_scale = 1.0
        sig = Arc(cfg)._generate_signature()
        assert "30" in sig  # 0.30 m -> 30.0 cm
        assert "90" in sig  # pi/2 rad -> 90.0 deg


# ===========================================================================
# turn.py — on_start (condition mode) + on_update lifecycle, via fakes
#
# The angle-mode on_start() builds a real C++ TurnMotion, so it is not
# exercised here; instead we drive on_update() with an injected fake _motion,
# and exercise the constant-angular-velocity on_start() branch with a fake
# robot (no motion controller built).
# ===========================================================================


class TestTurnLifecycle:
    def test_on_start_until_mode_sets_ccw_velocity(self):
        # Left + until => constant +omega = +1 * speed * max_w.
        step = TurnLeft(speed=0.5, until=FakeCondition())
        robot = FakeRobot(max_w=3.0)
        step.on_start(robot)
        v = robot.drive.velocity
        assert (v.vx, v.vy) == pytest.approx((0.0, 0.0))
        assert v.wz == pytest.approx(0.5 * 3.0)  # +1 * 0.5 * 3.0 = 1.5

    def test_on_start_until_mode_right_is_cw_velocity(self):
        # Right => negative omega.
        step = TurnRight(speed=0.5, until=FakeCondition())
        robot = FakeRobot(max_w=3.0)
        step.on_start(robot)
        assert robot.drive.velocity.wz == pytest.approx(-1.5)  # -1 * 0.5 * 3.0

    def test_on_start_until_mode_starts_condition(self):
        cond = FakeCondition()
        step = TurnLeft(speed=1.0, until=cond)
        step.on_start(FakeRobot())
        assert cond.started is True

    def test_on_update_angle_mode_advances_motion(self):
        step = TurnLeft(90)
        step._motion = FakeTurnMotion(finished=False)
        assert step.on_update(FakeRobot(), 0.02) is False
        assert step._motion.updates == [pytest.approx(0.02)]

    def test_on_update_angle_mode_finished_propagates(self):
        step = TurnRight(90)
        step._motion = FakeTurnMotion(finished=True)
        assert step.on_update(FakeRobot(), 0.02) is True

    def test_on_update_until_triggered_short_circuits(self):
        cond = FakeCondition(triggers=True)
        step = TurnLeft(speed=1.0, until=cond)
        # No _motion in condition mode; until=True returns True immediately.
        assert step.on_update(FakeRobot(), 0.01) is True
        assert cond.check_count == 1

    def test_on_update_until_not_triggered_updates_drive(self):
        cond = FakeCondition(triggers=False)
        step = TurnLeft(speed=1.0, until=cond)
        robot = FakeRobot()
        # No _motion -> falls through to robot.drive.update(dt), returns False.
        assert step.on_update(robot, 0.05) is False
        assert robot.drive.update_dts == [pytest.approx(0.05)]


# ===========================================================================
# arc.py — on_update lifecycle via injected fake motion
# ===========================================================================


class TestArcLifecycle:
    def test_on_update_advances_and_reports_finished(self):
        step = DriveArcLeft(radius_cm=30, degrees=90)
        step._motion = FakeTurnMotion(finished=True)
        assert step.on_update(FakeRobot(), 0.03) is True
        assert step._motion.updates == [pytest.approx(0.03)]

    def test_on_update_not_finished(self):
        step = StrafeArcRight(radius_cm=30, degrees=90)
        step._motion = FakeTurnMotion(finished=False)
        assert step.on_update(FakeRobot(), 0.03) is False


# ===========================================================================
# arc.py — on_start() builds + starts the ArcMotion, wired with the step's
# own config.  ArcMotion normally needs a real drivetrain, so patch the
# module-level name with a recording double.
# ===========================================================================


class _RecordingArcMotion:
    instances: list = []

    def __init__(self, drive, odometry, pid_config, config) -> None:
        self.drive = drive
        self.odometry = odometry
        self.pid_config = pid_config
        self.config = config
        self.started = False
        _RecordingArcMotion.instances.append(self)

    def start(self) -> None:
        self.started = True


@pytest.fixture
def patched_arc(monkeypatch):
    _RecordingArcMotion.instances = []
    monkeypatch.setattr(_arc, "ArcMotion", _RecordingArcMotion)
    return _RecordingArcMotion


class TestArcOnStart:
    def test_on_start_builds_and_starts_motion(self, patched_arc):
        step = DriveArcLeft(radius_cm=30, degrees=90, speed=0.5)
        robot = FakeRobot()
        robot.odometry = object()
        step.on_start(robot)
        assert len(patched_arc.instances) == 1
        m = patched_arc.instances[0]
        assert step._motion is m
        assert m.started is True

    def test_on_start_passes_step_config_through(self, patched_arc):
        step = DriveArcRight(radius_cm=30, degrees=90, speed=0.5)
        robot = FakeRobot()
        robot.odometry = object()
        step.on_start(robot)
        m = patched_arc.instances[0]
        # the very config the step built (negative angle, correct radius)
        assert m.config is step.config
        assert m.config.arc_angle_rad == pytest.approx(-_PI / 2.0)
        assert m.config.radius_m == pytest.approx(0.30)

    def test_on_start_wires_robot_components(self, patched_arc):
        step = StrafeArcLeft(radius_cm=30, degrees=90)
        robot = FakeRobot()
        odo = object()
        robot.odometry = odo
        step.on_start(robot)
        m = patched_arc.instances[0]
        assert m.drive is robot.drive
        assert m.odometry is odo
        assert m.pid_config is robot.motion_pid_config


# ===========================================================================
# DSL metadata (kills decorator/tag mutants)
# ===========================================================================


class TestDslMetadata:
    def test_turn_public_steps_are_hidden_base_steps(self):
        # The base _ConditionalTurn is an undecorated plain class; the public
        # TurnLeft/TurnRight carry the @dsl_step metadata.
        assert not hasattr(_ConditionalTurn, "__dsl__")
        assert TurnLeft.__dsl_step__ is True
        assert TurnRight.__dsl_step__ is True

    @pytest.mark.parametrize("cls", [TurnLeft, TurnRight])
    def test_turn_public_tags(self, cls):
        assert cls.__dsl__.tags == ("motion", "turn")

    def test_arc_base_hidden(self):
        assert Arc.__dsl__.hidden is True
        assert DriveArc.__dsl__.hidden is True
        assert StrafeArc.__dsl__.hidden is True

    @pytest.mark.parametrize("cls", [DriveArc, StrafeArc])
    def test_hidden_arc_carries_its_own_dsl_decorator(self, cls):
        # DriveArc / StrafeArc must each be *directly* decorated with
        # @dsl(hidden=True): their own class dict carries __dsl__ rather than
        # merely inheriting Arc's.  (Removing the decorator would fall back to
        # the inherited Arc.__dsl__, so this checks the own-dict entry.)
        assert "__dsl__" in cls.__dict__
        assert cls.__dict__["__dsl__"].hidden is True

    @pytest.mark.parametrize("cls", [DriveArcLeft, DriveArcRight])
    def test_drive_arc_tags(self, cls):
        assert cls.__dsl__.tags == ("motion", "arc")

    @pytest.mark.parametrize("cls", [StrafeArcLeft, StrafeArcRight])
    def test_strafe_arc_tags(self, cls):
        assert cls.__dsl__.tags == ("motion", "strafe", "arc")


# ===========================================================================
# turn.py — angle-mode on_start() via monkeypatched TurnMotion/TurnConfig
#
# Angle-mode on_start() normally builds the native C++ TurnMotion (which needs
# a real drivetrain).  We patch the module-level TurnMotion + TurnConfig names
# with pure-Python doubles so we can assert the exact TurnConfig values the
# step writes (target_angle_rad = sign * radians(degrees); speed_scale = speed)
# and that the motion is constructed + started — without any hardware.
# ===========================================================================


class _FakeTurnConfig:
    """Pure-Python stand-in for the native TurnConfig (plain attr bag)."""

    def __init__(self) -> None:
        self.target_angle_rad = None
        self.speed_scale = None


class _RecordingTurnMotion:
    """Captures the ctor args (incl. the config) and records start()."""

    instances: list = []

    def __init__(self, drive, odometry, pid_config, config) -> None:
        self.drive = drive
        self.odometry = odometry
        self.pid_config = pid_config
        self.config = config
        self.started = False
        _RecordingTurnMotion.instances.append(self)

    def start(self) -> None:
        self.started = True


@pytest.fixture
def patched_turn(monkeypatch):
    """Patch turn.TurnMotion + turn.TurnConfig with the doubles above."""
    _RecordingTurnMotion.instances = []
    monkeypatch.setattr(_turn, "TurnConfig", _FakeTurnConfig)
    monkeypatch.setattr(_turn, "TurnMotion", _RecordingTurnMotion)
    return _RecordingTurnMotion


class TestTurnAngleOnStart:
    def test_left_builds_motion_with_positive_target(self, patched_turn):
        step = TurnLeft(90, speed=0.5)
        robot = FakeRobot()
        robot.odometry = object()
        step.on_start(robot)
        # exactly one TurnMotion built and started
        assert len(patched_turn.instances) == 1
        m = patched_turn.instances[0]
        assert m.started is True
        assert step._motion is m
        # +1 * radians(90) = +pi/2
        assert m.config.target_angle_rad == pytest.approx(_PI / 2.0)
        assert m.config.speed_scale == pytest.approx(0.5)
        # angle mode must NOT also command a raw velocity
        assert robot.drive.velocity is None

    def test_right_builds_motion_with_negative_target(self, patched_turn):
        step = TurnRight(90, speed=1.0)
        robot = FakeRobot()
        robot.odometry = object()
        step.on_start(robot)
        m = patched_turn.instances[0]
        # -1 * radians(90) = -pi/2
        assert m.config.target_angle_rad == pytest.approx(-_PI / 2.0)
        assert m.config.speed_scale == pytest.approx(1.0)

    def test_target_uses_radians_and_multiplication(self, patched_turn):
        # 180 deg left -> +pi (kills the * -> / mutation: sign/radians(180) != pi).
        step = TurnLeft(180)
        robot = FakeRobot()
        robot.odometry = object()
        step.on_start(robot)
        assert patched_turn.instances[0].config.target_angle_rad == pytest.approx(_PI)

    def test_motion_built_with_robot_wiring(self, patched_turn):
        step = TurnLeft(45)
        robot = FakeRobot()
        odo = object()
        robot.odometry = odo
        step.on_start(robot)
        m = patched_turn.instances[0]
        assert m.drive is robot.drive
        assert m.odometry is odo
        assert m.pid_config is robot.motion_pid_config

    def test_angle_and_until_starts_both(self, patched_turn):
        cond = FakeCondition()
        step = TurnLeft(90, until=cond)
        robot = FakeRobot()
        robot.odometry = object()
        step.on_start(robot)
        # profiled motion built (degrees present) AND condition started
        assert step._motion is patched_turn.instances[0]
        assert cond.started is True


# ===========================================================================
# turn.py — base _ConditionalTurn instantiated directly (kills base-default
# mutants on the shared __init__ that the public subclasses never reach).
# ===========================================================================


class TestConditionalTurnBaseDefaults:
    def test_base_default_speed_is_one(self):
        # The base __init__ default speed is 1.0; if it were > 1.0 the bound
        # check (0.0, 1.0] would reject the default-speed construction.
        step = _ConditionalTurn(degrees=90)
        assert step._speed == pytest.approx(1.0)

    def test_base_until_only(self):
        cond = FakeCondition()
        step = _ConditionalTurn(until=cond)
        assert step._until is cond
        assert step._degrees is None

    def test_base_requires_degrees_or_until(self):
        with pytest.raises(ValueError):
            _ConditionalTurn()
