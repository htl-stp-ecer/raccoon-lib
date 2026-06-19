"""Comprehensive tests for the line_follow system.

The line_follow component is mission-critical, so this suite exercises the
full public surface of ``raccoon.step.motion.line_follow`` /
``line_follow_builder`` *without* a real (C++) drivetrain:

1. Sign / helper math (``_forward_correction_sign_for_lateral_follow``,
   ``_correction_mode_name``).
2. The ``@dsl_step`` factory classes — argument wiring into their config
   dataclasses, validation, defaults, and ``_generate_signature()``.
3. The auto-generated ``*_dsl`` builders + factory functions and the way
   they resolve into the real step.
4. The fluent ``ConfigurableLineFollowBuilder`` — every configuration knob,
   correction-mode selection, sign composition, and validation guard.
5. The actual *runtime* error/steering logic of ``on_update()`` — the
   integral behaviour — driven through injected fake motion/PID/condition
   doubles so we assert the exact error value and control wiring.
6. ``to_simulation_step()`` delta geometry.

The C++ ``LinearMotion`` / ``DirectionalLineFollowMotion`` are only created
in ``on_start()``; every test here either skips ``on_start()`` (injecting a
fake ``_motion``) or asserts on the pure-Python config, so no hardware,
mock bundle, or event loop is required.
"""

from __future__ import annotations

import math

import pytest

from raccoon.step.condition import StopCondition
from raccoon.step.motion import (
    DirectionalFollowLine,
    DirectionalFollowLineSingle,
    DirectionalLineFollow,
    DirectionalSingleLineFollow,
    FollowLine,
    FollowLineSingle,
    LateralFollowLine,
    LateralFollowLineSingle,
    LineSide,
    StrafeFollowLine,
    StrafeFollowLineSingle,
    directional_follow_line,
    directional_follow_line_single,
    follow_line,
    follow_line_single,
    lateral_follow_line,
    lateral_follow_line_single,
    line_follow,
    strafe_follow_line,
    strafe_follow_line_single,
)
from raccoon.step.motion.line_follow import (
    _correction_mode_name,
    _forward_correction_sign_for_lateral_follow,
)
from raccoon.step.motion.line_follow_builder import (
    ConfigurableLineFollowBuilder,
    FollowCorrection,
)

# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------


class FakeSensor:
    """Minimal IR sensor stand-in (duck-typed against ``IRSensor``)."""

    def __init__(self, black: float = 0.5, raw: int = 512) -> None:
        self._black = black
        self._raw = raw

    def probabilityOfBlack(self) -> float:
        return self._black

    def read(self) -> int:
        return self._raw


class FakePid:
    """Records every (error, dt) fed to it and returns a fixed output."""

    def __init__(self, output: float = 0.42) -> None:
        self.output = output
        self.calls: list[tuple[float, float]] = []

    def update(self, error: float, dt: float) -> float:
        self.calls.append((error, dt))
        return self.output

    @property
    def last_error(self) -> float:
        return self.calls[-1][0]


class FakeLinearMotion:
    """Stand-in for the profiled ``LinearMotion`` used by FollowLine."""

    def __init__(self, finished: bool = False) -> None:
        self.omega_overrides: list[float] = []
        self.updates: list[float] = []
        self._finished = finished
        self.started = False

    def start(self) -> None:
        self.started = True

    def set_omega_override(self, wz: float) -> None:
        self.omega_overrides.append(wz)

    def update(self, dt: float) -> None:
        self.updates.append(dt)

    def is_finished(self) -> bool:
        return self._finished


class _TelemetrySample:
    def __init__(self, correction: float) -> None:
        self.correction = correction


class FakeDirectionalMotion:
    """Stand-in for the direct-velocity ``DirectionalLineFollowMotion``."""

    def __init__(self, finished: bool = False, telemetry: list | None = None) -> None:
        self.sensor_errors: list[float] = []
        self.updates: list[float] = []
        self._finished = finished
        self._telemetry = telemetry if telemetry is not None else []
        self.started = False

    def start(self) -> None:
        self.started = True

    def set_sensor_error(self, error: float) -> None:
        self.sensor_errors.append(error)

    def update(self, dt: float) -> None:
        self.updates.append(dt)

    def get_telemetry(self) -> list:
        return self._telemetry

    def is_finished(self) -> bool:
        return self._finished


class FakeCondition(StopCondition):
    """Controllable stop condition that records lifecycle calls."""

    def __init__(self, *, triggers: bool = False) -> None:
        self.triggers = triggers
        self.started = False
        self.check_count = 0

    def start(self, robot) -> None:
        self.started = True

    def check(self, robot) -> bool:
        self.check_count += 1
        return self.triggers


_ROBOT = object()  # on_update never touches the robot unless `until` is set


# ===========================================================================
# 1. Sign / helper math
# ===========================================================================


@pytest.mark.parametrize(
    ("strafe", "expected"),
    [
        (0.5, -1.0),  # strafing right -> correction normal is -vx
        (0.0, -1.0),  # boundary: >= 0 counts as "right"
        (1e-9, -1.0),
        (-0.5, 1.0),  # strafing left -> +vx
        (-1e-9, 1.0),
    ],
)
def test_forward_correction_sign_for_lateral_follow(strafe: float, expected: float) -> None:
    assert _forward_correction_sign_for_lateral_follow(strafe) == pytest.approx(expected)


def test_correction_mode_name_priority() -> None:
    cfg = DirectionalFollowLine(FakeSensor(), FakeSensor(), distance_cm=10).config
    assert _correction_mode_name(cfg) == "angular"

    cfg = StrafeFollowLine(FakeSensor(), FakeSensor(), distance_cm=10).config
    assert cfg.lateral_correction is True
    assert _correction_mode_name(cfg) == "lateral"

    cfg = LateralFollowLine(FakeSensor(), FakeSensor(), distance_cm=10).config
    assert cfg.forward_correction is True
    assert _correction_mode_name(cfg) == "forward"


def test_correction_mode_name_forward_wins_over_lateral() -> None:
    # If both flags were ever set, "forward" must take precedence.
    cfg = DirectionalFollowLine(FakeSensor(), FakeSensor(), distance_cm=10).config
    cfg.lateral_correction = True
    cfg.forward_correction = True
    assert _correction_mode_name(cfg) == "forward"


# ===========================================================================
# 2. @dsl_step factory classes — construction, validation, config wiring
# ===========================================================================


def test_line_side_enum_values() -> None:
    assert LineSide.LEFT.value == "left"
    assert LineSide.RIGHT.value == "right"
    assert LineSide.LEFT is not LineSide.RIGHT


@pytest.mark.parametrize(
    "factory",
    [
        lambda: FollowLine(FakeSensor(), FakeSensor()),
        lambda: FollowLineSingle(FakeSensor()),
        lambda: DirectionalFollowLine(FakeSensor(), FakeSensor()),
        lambda: StrafeFollowLine(FakeSensor(), FakeSensor()),
        lambda: StrafeFollowLineSingle(FakeSensor()),
        lambda: LateralFollowLine(FakeSensor(), FakeSensor()),
        lambda: LateralFollowLineSingle(FakeSensor()),
        lambda: DirectionalFollowLineSingle(FakeSensor()),
    ],
)
def test_step_requires_distance_or_until(factory) -> None:
    with pytest.raises(ValueError, match="either 'distance_cm' or 'until'"):
        factory()


def test_step_accepts_until_only() -> None:
    cond = FakeCondition()
    step = FollowLine(FakeSensor(), FakeSensor(), until=cond)
    assert step._until is cond
    assert step.config.distance_cm is None


def test_step_accepts_distance_and_until_together() -> None:
    cond = FakeCondition()
    step = FollowLine(FakeSensor(), FakeSensor(), distance_cm=70, until=cond)
    assert step._until is cond
    assert step.config.distance_cm == pytest.approx(70)


def test_follow_line_config_defaults() -> None:
    step = FollowLine(FakeSensor(), FakeSensor(), distance_cm=50)
    cfg = step.config
    assert cfg.speed_scale == pytest.approx(0.5)
    assert cfg.kp == pytest.approx(0.4)
    assert cfg.ki == pytest.approx(0.0)
    assert cfg.kd == pytest.approx(0.1)
    assert cfg.distance_cm == pytest.approx(50)


def test_follow_line_config_custom_gains() -> None:
    left, right = FakeSensor(), FakeSensor()
    step = FollowLine(left, right, distance_cm=120, speed=0.7, kp=1.1, ki=0.3, kd=0.6)
    cfg = step.config
    assert cfg.left_sensor is left
    assert cfg.right_sensor is right
    assert cfg.speed_scale == pytest.approx(0.7)
    assert (cfg.kp, cfg.ki, cfg.kd) == pytest.approx((1.1, 0.3, 0.6))


def test_follow_line_single_side_and_defaults() -> None:
    step = FollowLineSingle(FakeSensor(), distance_cm=40, side=LineSide.RIGHT)
    assert step.config.side is LineSide.RIGHT
    # Default side is LEFT.
    assert FollowLineSingle(FakeSensor(), distance_cm=40).config.side is LineSide.LEFT


def test_directional_follow_line_independent_speeds() -> None:
    step = DirectionalFollowLine(
        FakeSensor(), FakeSensor(), distance_cm=50, heading_speed=0.3, strafe_speed=-0.4
    )
    cfg = step.config
    assert cfg.heading_speed == pytest.approx(0.3)
    assert cfg.strafe_speed == pytest.approx(-0.4)
    # Plain directional follow corrects via angular velocity.
    assert cfg.lateral_correction is False
    assert cfg.forward_correction is False
    assert cfg.heading_hold is True
    assert cfg.correction_sign == pytest.approx(1.0)


def test_strafe_follow_line_is_lateral_correction() -> None:
    step = StrafeFollowLine(FakeSensor(), FakeSensor(), distance_cm=40, speed=0.6)
    cfg = step.config
    assert cfg.lateral_correction is True
    assert cfg.forward_correction is False
    assert cfg.heading_speed == pytest.approx(0.6)  # forward drive
    assert cfg.strafe_speed == pytest.approx(0.0)  # correction axis stays 0


def test_strafe_follow_line_single_is_lateral_correction() -> None:
    step = StrafeFollowLineSingle(FakeSensor(), distance_cm=40, speed=0.6, side=LineSide.RIGHT)
    cfg = step.config
    assert cfg.lateral_correction is True
    assert cfg.heading_speed == pytest.approx(0.6)
    assert cfg.strafe_speed == pytest.approx(0.0)
    assert cfg.side is LineSide.RIGHT


@pytest.mark.parametrize(
    ("speed", "expected_sign"),
    [(0.5, -1.0), (-0.5, 1.0), (0.0, -1.0)],
)
def test_lateral_follow_line_forward_correction_and_sign(
    speed: float, expected_sign: float
) -> None:
    step = LateralFollowLine(FakeSensor(), FakeSensor(), distance_cm=40, speed=speed)
    cfg = step.config
    assert cfg.forward_correction is True
    assert cfg.lateral_correction is False
    assert cfg.strafe_speed == pytest.approx(speed)  # lateral is the primary axis
    assert cfg.heading_speed == pytest.approx(0.0)
    assert cfg.correction_sign == pytest.approx(expected_sign)


@pytest.mark.parametrize(
    ("speed", "expected_sign"),
    [(0.5, -1.0), (-0.5, 1.0)],
)
def test_lateral_follow_line_single_forward_correction_and_sign(
    speed: float, expected_sign: float
) -> None:
    step = LateralFollowLineSingle(FakeSensor(), distance_cm=40, speed=speed)
    cfg = step.config
    assert cfg.forward_correction is True
    assert cfg.strafe_speed == pytest.approx(speed)
    assert cfg.correction_sign == pytest.approx(expected_sign)


def test_directional_follow_line_single_defaults() -> None:
    step = DirectionalFollowLineSingle(
        FakeSensor(), distance_cm=50, heading_speed=0.2, strafe_speed=0.3, side=LineSide.RIGHT
    )
    cfg = step.config
    assert cfg.heading_speed == pytest.approx(0.2)
    assert cfg.strafe_speed == pytest.approx(0.3)
    assert cfg.side is LineSide.RIGHT
    assert cfg.lateral_correction is False
    assert cfg.forward_correction is False


# --- signatures -----------------------------------------------------------


def test_signature_includes_distance_and_until() -> None:
    sig = FollowLine(
        FakeSensor(), FakeSensor(), distance_cm=80, until=FakeCondition()
    )._generate_signature()
    assert "80.0cm" in sig
    assert "until" in sig
    assert sig.startswith("FollowLine(")


def test_signature_distance_only() -> None:
    sig = FollowLine(FakeSensor(), FakeSensor(), distance_cm=33.0)._generate_signature()
    assert "33.0cm" in sig
    assert "until" not in sig


def test_signature_until_only_has_no_distance() -> None:
    sig = FollowLine(FakeSensor(), FakeSensor(), until=FakeCondition())._generate_signature()
    assert "cm" not in sig
    assert "until" in sig


def test_single_signature_reports_side_and_speed() -> None:
    sig = FollowLineSingle(
        FakeSensor(), distance_cm=20, side=LineSide.RIGHT, speed=0.25
    )._generate_signature()
    assert "side=right" in sig
    assert "speed=0.25" in sig


def test_lateral_signature_reports_speed() -> None:
    sig = LateralFollowLine(
        FakeSensor(), FakeSensor(), distance_cm=15, speed=0.4
    )._generate_signature()
    assert sig.startswith("LateralFollowLine(")
    assert "speed=0.40" in sig


# ===========================================================================
# 3. Auto-generated *_dsl builders and factory functions
# ===========================================================================


def test_follow_line_factory_resolves_to_step() -> None:
    left, right = FakeSensor(), FakeSensor()
    step = follow_line(left, right, distance_cm=80, speed=0.3, kp=0.9, ki=0.1, kd=0.2).resolve()
    assert isinstance(step, FollowLine)
    assert step.config.left_sensor is left
    assert step.config.right_sensor is right
    assert step.config.speed_scale == pytest.approx(0.3)
    assert (step.config.kp, step.config.ki, step.config.kd) == pytest.approx((0.9, 0.1, 0.2))


def test_follow_line_factory_fluent_setters() -> None:
    left, right = FakeSensor(), FakeSensor()
    step = (
        follow_line()
        .left_sensor(left)
        .right_sensor(right)
        .distance_cm(60)
        .speed(0.45)
        .kp(0.8)
        .resolve()
    )
    assert isinstance(step, FollowLine)
    assert step.config.distance_cm == pytest.approx(60)
    assert step.config.speed_scale == pytest.approx(0.45)
    assert step.config.kp == pytest.approx(0.8)


def test_factory_defaults_match_step_defaults() -> None:
    step = follow_line(FakeSensor(), FakeSensor(), distance_cm=10).resolve()
    assert step.config.speed_scale == pytest.approx(0.5)
    assert (step.config.kp, step.config.ki, step.config.kd) == pytest.approx((0.4, 0.0, 0.1))


def test_factory_until_only_resolves() -> None:
    cond = FakeCondition()
    step = follow_line(FakeSensor(), FakeSensor(), speed=0.3).until(cond).resolve()
    assert isinstance(step, FollowLine)
    assert step._until is cond
    assert step.config.distance_cm is None


def test_factory_resolve_is_idempotent() -> None:
    builder = follow_line(FakeSensor(), FakeSensor(), distance_cm=10)
    first = builder.resolve()
    assert builder.resolve() is first


def test_factory_propagates_validation_error_on_resolve() -> None:
    with pytest.raises(ValueError, match="either 'distance_cm' or 'until'"):
        follow_line(FakeSensor(), FakeSensor()).resolve()


def test_factory_missing_sensor_raises_on_resolve() -> None:
    # Sensors are left _UNSET -> the underlying constructor is missing args.
    with pytest.raises(TypeError):
        follow_line(distance_cm=10).resolve()


@pytest.mark.parametrize(
    ("factory", "step_cls", "kwargs"),
    [
        (follow_line_single, FollowLineSingle, {"sensor": FakeSensor(), "distance_cm": 30}),
        (
            directional_follow_line,
            DirectionalFollowLine,
            {"left_sensor": FakeSensor(), "right_sensor": FakeSensor(), "distance_cm": 30},
        ),
        (
            strafe_follow_line,
            StrafeFollowLine,
            {"left_sensor": FakeSensor(), "right_sensor": FakeSensor(), "distance_cm": 30},
        ),
        (
            strafe_follow_line_single,
            StrafeFollowLineSingle,
            {"sensor": FakeSensor(), "distance_cm": 30},
        ),
        (
            lateral_follow_line,
            LateralFollowLine,
            {"left_sensor": FakeSensor(), "right_sensor": FakeSensor(), "distance_cm": 30},
        ),
        (
            lateral_follow_line_single,
            LateralFollowLineSingle,
            {"sensor": FakeSensor(), "distance_cm": 30},
        ),
        (
            directional_follow_line_single,
            DirectionalFollowLineSingle,
            {"sensor": FakeSensor(), "distance_cm": 30},
        ),
    ],
)
def test_every_factory_resolves_to_its_step(factory, step_cls, kwargs) -> None:
    step = factory(**kwargs).resolve()
    assert isinstance(step, step_cls)
    assert step.config.distance_cm == pytest.approx(30)


def test_strafe_factory_side_setter() -> None:
    step = (
        strafe_follow_line_single()
        .sensor(FakeSensor())
        .distance_cm(20)
        .side(LineSide.RIGHT)
        .resolve()
    )
    assert step.config.side is LineSide.RIGHT
    assert step.config.lateral_correction is True


def test_lateral_factory_preserves_sign_for_negative_speed() -> None:
    step = lateral_follow_line(FakeSensor(), FakeSensor(), distance_cm=20, speed=-0.5).resolve()
    assert step.config.correction_sign == pytest.approx(1.0)
    assert step.config.forward_correction is True


# ===========================================================================
# 4. ConfigurableLineFollowBuilder (the fluent "hero" builder)
# ===========================================================================


def test_configurable_builder_defaults() -> None:
    b = ConfigurableLineFollowBuilder()
    assert b._kp == pytest.approx(0.4)
    assert b._ki == pytest.approx(0.0)
    assert b._kd == pytest.approx(0.1)
    assert b._heading_hold is True
    assert b._correction_sign == pytest.approx(1.0)
    assert b._correction is FollowCorrection.ANGULAR


def test_configurable_builder_methods_return_self() -> None:
    b = line_follow()
    assert b.single(FakeSensor()) is b
    assert b.move(forward=0.5) is b
    assert b.correct_angular() is b
    assert b.pid(0.5) is b
    assert b.distance_cm(10) is b


def test_configurable_angular_single() -> None:
    step = (
        line_follow()
        .single(FakeSensor(), side=LineSide.RIGHT)
        .move(forward=0.6)
        .correct_angular()
        .distance_cm(30)
        .resolve()
    )
    assert isinstance(step, DirectionalSingleLineFollow)
    cfg = step.config
    assert cfg.side is LineSide.RIGHT
    assert cfg.heading_speed == pytest.approx(0.6)
    assert cfg.lateral_correction is False
    assert cfg.forward_correction is False
    assert cfg.heading_hold is True


def test_configurable_angular_dual() -> None:
    left, right = FakeSensor(), FakeSensor()
    step = (
        line_follow()
        .dual(left, right)
        .move(forward=0.5, strafe=0.0)
        .correct_angular()
        .distance_cm(30)
        .resolve()
    )
    assert isinstance(step, DirectionalLineFollow)
    assert step.config.left_sensor is left
    assert step.config.right_sensor is right


def test_configurable_forward_and_strafe_speed_setters() -> None:
    step = (
        line_follow()
        .dual(FakeSensor(), FakeSensor())
        .forward_speed(0.3)
        .strafe_speed(0.4)
        .correct_angular()
        .distance_cm(25)
        .resolve()
    )
    assert step.config.heading_speed == pytest.approx(0.3)
    assert step.config.strafe_speed == pytest.approx(0.4)


def test_configurable_lateral_hold_heading_flag() -> None:
    step = (
        line_follow()
        .single(FakeSensor())
        .move(forward=0.5)
        .correct_lateral(hold_heading=False)
        .distance_cm(20)
        .resolve()
    )
    assert step.config.lateral_correction is True
    assert step.config.heading_hold is False


def test_configurable_forward_correction_sign_composition() -> None:
    # correct_forward multiplies the user sign by the strafe-direction normal.
    step = (
        line_follow()
        .dual(FakeSensor(), FakeSensor())
        .move(strafe=0.5)  # strafing right -> normal sign -1
        .correct_forward()
        .distance_cm(20)
        .resolve()
    )
    assert step.config.forward_correction is True
    assert step.config.correction_sign == pytest.approx(-1.0)


def test_configurable_forward_correction_negative_strafe() -> None:
    step = (
        line_follow()
        .dual(FakeSensor(), FakeSensor())
        .move(strafe=-0.5)  # strafing left -> normal sign +1
        .correct_forward()
        .distance_cm(20)
        .resolve()
    )
    assert step.config.correction_sign == pytest.approx(1.0)


def test_configurable_custom_correction_sign_multiplies_forward_normal() -> None:
    step = (
        line_follow()
        .dual(FakeSensor(), FakeSensor())
        .move(strafe=0.5)
        .correct_forward()
        .correction_sign(2.0)  # user multiplier
        .distance_cm(20)
        .resolve()
    )
    # 2.0 (user) * -1.0 (right-strafe normal) = -2.0
    assert step.config.correction_sign == pytest.approx(-2.0)


def test_configurable_custom_correction_sign_passthrough_for_angular() -> None:
    step = (
        line_follow()
        .dual(FakeSensor(), FakeSensor())
        .move(forward=0.5)
        .correct_angular()
        .correction_sign(-3.0)
        .distance_cm(20)
        .resolve()
    )
    # Angular correction does not apply the lateral-normal factor.
    assert step.config.correction_sign == pytest.approx(-3.0)


def test_configurable_pid_setter() -> None:
    step = (
        line_follow()
        .single(FakeSensor())
        .move(forward=0.5)
        .correct_angular()
        .pid(1.5, 0.2, 0.7)
        .distance_cm(20)
        .resolve()
    )
    assert (step.config.kp, step.config.ki, step.config.kd) == pytest.approx((1.5, 0.2, 0.7))


def test_configurable_until_without_distance() -> None:
    cond = FakeCondition()
    step = (
        line_follow().single(FakeSensor()).move(forward=0.5).correct_angular().until(cond).resolve()
    )
    assert step._until is cond
    assert step.config.distance_cm is None


def test_configurable_single_resets_dual_and_vice_versa() -> None:
    # Calling single() after dual() must clear the pair so the build picks
    # the single-sensor step (and not raise "exactly one tracking mode").
    step = (
        line_follow()
        .dual(FakeSensor(), FakeSensor())
        .single(FakeSensor())
        .move(forward=0.5)
        .correct_angular()
        .distance_cm(20)
        .resolve()
    )
    assert isinstance(step, DirectionalSingleLineFollow)

    step2 = (
        line_follow()
        .single(FakeSensor())
        .dual(FakeSensor(), FakeSensor())
        .move(forward=0.5)
        .correct_angular()
        .distance_cm(20)
        .resolve()
    )
    assert isinstance(step2, DirectionalLineFollow)


# --- builder validation guards --------------------------------------------


def test_configurable_requires_distance_or_until() -> None:
    with pytest.raises(ValueError, match=r"distance_cm.*until"):
        line_follow().single(FakeSensor()).move(forward=0.5).correct_angular().resolve()


def test_configurable_requires_a_tracking_mode() -> None:
    with pytest.raises(ValueError, match="exactly one tracking mode"):
        line_follow().move(forward=0.5).correct_angular().distance_cm(20).resolve()


def test_configurable_lateral_rejects_nonzero_strafe() -> None:
    with pytest.raises(ValueError, match="base strafe must stay 0"):
        (
            line_follow()
            .single(FakeSensor())
            .move(forward=0.5, strafe=0.2)
            .correct_lateral()
            .distance_cm(20)
            .resolve()
        )


def test_configurable_forward_rejects_nonzero_forward() -> None:
    with pytest.raises(ValueError, match="base forward must stay 0"):
        (
            line_follow()
            .dual(FakeSensor(), FakeSensor())
            .move(forward=0.5, strafe=0.5)
            .correct_forward()
            .distance_cm(20)
            .resolve()
        )


def test_configurable_angular_rejects_zero_base_motion() -> None:
    with pytest.raises(ValueError, match="non-zero base motion"):
        (
            line_follow()
            .single(FakeSensor())
            .move(forward=0.0, strafe=0.0)
            .correct_angular()
            .distance_cm(20)
            .resolve()
        )


def test_configurable_default_correction_is_angular_needs_motion() -> None:
    # Without an explicit correct_*(), the default ANGULAR mode still demands
    # non-zero base motion.
    with pytest.raises(ValueError, match="non-zero base motion"):
        line_follow().single(FakeSensor()).distance_cm(20).resolve()


# ===========================================================================
# 5. Runtime steering logic — on_update() with injected fakes
# ===========================================================================


def _arm_profiled(step, pid: FakePid, motion: FakeLinearMotion) -> None:
    step._pid = pid
    step._motion = motion


def _arm_directional(step, motion: FakeDirectionalMotion) -> None:
    step._motion = motion


def test_two_sensor_error_is_left_minus_right() -> None:
    step = FollowLine(FakeSensor(black=0.8), FakeSensor(black=0.2), distance_cm=50)
    pid, motion = FakePid(output=0.33), FakeLinearMotion()
    _arm_profiled(step, pid, motion)

    finished = step.on_update(_ROBOT, 0.01)

    assert pid.last_error == pytest.approx(0.6)  # 0.8 - 0.2
    assert motion.omega_overrides == [pytest.approx(0.33)]  # PID output -> omega
    assert motion.updates == [pytest.approx(0.01)]
    assert finished is False


def test_two_sensor_error_symmetric_when_centered() -> None:
    step = FollowLine(FakeSensor(black=0.5), FakeSensor(black=0.5), distance_cm=50)
    pid, motion = FakePid(), FakeLinearMotion()
    _arm_profiled(step, pid, motion)
    step.on_update(_ROBOT, 0.02)
    assert pid.last_error == pytest.approx(0.0)


def test_two_sensor_finished_propagates() -> None:
    step = FollowLine(FakeSensor(), FakeSensor(), distance_cm=50)
    _arm_profiled(step, FakePid(), FakeLinearMotion(finished=True))
    assert step.on_update(_ROBOT, 0.01) is True


@pytest.mark.parametrize(
    ("side", "reading", "expected_error"),
    [
        (LineSide.LEFT, 0.7, 0.2),  # reading - 0.5
        (LineSide.LEFT, 0.3, -0.2),
        (LineSide.RIGHT, 0.7, -0.2),  # sign flipped
        (LineSide.RIGHT, 0.3, 0.2),
        (LineSide.LEFT, 0.5, 0.0),  # exactly on the edge
        (LineSide.RIGHT, 0.5, 0.0),
    ],
)
def test_single_sensor_edge_error(side: LineSide, reading: float, expected_error: float) -> None:
    step = FollowLineSingle(FakeSensor(black=reading), distance_cm=50, side=side)
    pid, motion = FakePid(), FakeLinearMotion()
    _arm_profiled(step, pid, motion)
    step.on_update(_ROBOT, 0.01)
    assert pid.last_error == pytest.approx(expected_error)


def test_directional_two_sensor_sets_sensor_error() -> None:
    step = DirectionalFollowLine(
        FakeSensor(black=0.9), FakeSensor(black=0.1), distance_cm=50, strafe_speed=0.5
    )
    motion = FakeDirectionalMotion()
    _arm_directional(step, motion)
    finished = step.on_update(_ROBOT, 0.01)
    assert motion.sensor_errors == [pytest.approx(0.8)]
    assert motion.updates == [pytest.approx(0.01)]
    assert finished is False


def test_directional_finished_propagates() -> None:
    step = DirectionalFollowLine(FakeSensor(), FakeSensor(), distance_cm=50, strafe_speed=0.5)
    _arm_directional(step, FakeDirectionalMotion(finished=True))
    assert step.on_update(_ROBOT, 0.01) is True


def test_directional_handles_empty_telemetry() -> None:
    # correction defaults to 0.0 when telemetry is empty — must not raise.
    step = DirectionalFollowLine(FakeSensor(), FakeSensor(), distance_cm=50, strafe_speed=0.5)
    _arm_directional(step, FakeDirectionalMotion(telemetry=[]))
    step.on_update(_ROBOT, 0.01)  # no exception


def test_directional_reads_last_telemetry_correction() -> None:
    step = DirectionalFollowLine(FakeSensor(), FakeSensor(), distance_cm=50, strafe_speed=0.5)
    motion = FakeDirectionalMotion(telemetry=[_TelemetrySample(0.1), _TelemetrySample(0.9)])
    _arm_directional(step, motion)
    step.on_update(_ROBOT, 0.01)  # uses telemetry[-1].correction == 0.9, no exception


@pytest.mark.parametrize(
    ("side", "reading", "expected_error"),
    [
        (LineSide.LEFT, 0.75, 0.25),
        (LineSide.RIGHT, 0.75, -0.25),
    ],
)
def test_directional_single_edge_error(
    side: LineSide, reading: float, expected_error: float
) -> None:
    step = DirectionalFollowLineSingle(
        FakeSensor(black=reading), distance_cm=50, strafe_speed=0.4, side=side
    )
    motion = FakeDirectionalMotion()
    _arm_directional(step, motion)
    step.on_update(_ROBOT, 0.01)
    assert motion.sensor_errors == [pytest.approx(expected_error)]


# --- the `until` short-circuit --------------------------------------------


def test_until_short_circuits_profiled_before_touching_motion() -> None:
    cond = FakeCondition(triggers=True)
    step = FollowLine(FakeSensor(black=0.8), FakeSensor(black=0.2), distance_cm=50, until=cond)
    pid, motion = FakePid(), FakeLinearMotion()
    _arm_profiled(step, pid, motion)

    finished = step.on_update(_ROBOT, 0.01)

    assert finished is True
    assert cond.check_count == 1
    assert pid.calls == []  # never reached the PID
    assert motion.omega_overrides == []
    assert motion.updates == []


def test_until_not_triggered_continues_to_steer() -> None:
    cond = FakeCondition(triggers=False)
    step = FollowLine(FakeSensor(black=0.8), FakeSensor(black=0.2), distance_cm=50, until=cond)
    pid, motion = FakePid(), FakeLinearMotion()
    _arm_profiled(step, pid, motion)

    finished = step.on_update(_ROBOT, 0.01)

    assert finished is False
    assert cond.check_count == 1
    assert pid.last_error == pytest.approx(0.6)  # steering still ran


def test_until_short_circuits_directional() -> None:
    cond = FakeCondition(triggers=True)
    step = DirectionalFollowLine(
        FakeSensor(), FakeSensor(), distance_cm=50, strafe_speed=0.5, until=cond
    )
    motion = FakeDirectionalMotion()
    _arm_directional(step, motion)

    assert step.on_update(_ROBOT, 0.01) is True
    assert motion.sensor_errors == []  # never fed the controller
    assert motion.updates == []


# ===========================================================================
# 6. to_simulation_step() delta geometry
# ===========================================================================


def test_simulation_delta_profiled_forward_only() -> None:
    step = FollowLine(FakeSensor(), FakeSensor(), distance_cm=50)
    delta = step.to_simulation_step().delta
    assert delta.forward == pytest.approx(0.5)  # 50 cm -> 0.5 m
    assert delta.strafe == pytest.approx(0.0)
    assert delta.angular == pytest.approx(0.0)


def test_simulation_delta_profiled_until_only_uses_fallback_distance() -> None:
    step = FollowLine(FakeSensor(), FakeSensor(), until=FakeCondition())
    delta = step.to_simulation_step().delta
    # No distance -> 0.3 m fallback.
    assert delta.forward == pytest.approx(0.3)


def test_simulation_delta_directional_splits_by_speed_components() -> None:
    step = DirectionalFollowLine(
        FakeSensor(), FakeSensor(), distance_cm=50, heading_speed=0.3, strafe_speed=0.4
    )
    delta = step.to_simulation_step().delta
    mag = math.hypot(0.3, 0.4)  # 0.5
    assert delta.forward == pytest.approx(0.5 * 0.3 / mag)  # 0.5 m * 0.6
    assert delta.strafe == pytest.approx(0.5 * 0.4 / mag)  # 0.5 m * 0.8


def test_simulation_delta_directional_zero_speed_defaults_to_forward() -> None:
    # When both speed components are 0, geometry falls back to pure forward.
    step = DirectionalFollowLine(FakeSensor(), FakeSensor(), until=FakeCondition())
    delta = step.to_simulation_step().delta
    assert delta.forward == pytest.approx(0.3)  # 0.3 m fallback * fwd_frac 1.0
    assert delta.strafe == pytest.approx(0.0)
