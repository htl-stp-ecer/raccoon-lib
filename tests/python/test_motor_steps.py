"""Spec-driven tests for ``raccoon.step.motor.steps``.

The motor steps are thin wrappers around an ``IMotor`` hardware handle. Every
step is constructed with pure-Python validation (percent range, velocity > 0,
timeout > 0, ``.port`` duck-typing) and its ``_execute_step()`` coroutine
issues a single firmware command. No real C++ motor is needed: we inject a
``FakeMotor`` double that records the calls it receives, so each test asserts
the *semantic* command (which method, which argument) rather than re-running
the implementation.

Expected values are derived independently from the docstrings:

* percent power is clamped to the closed range ``[-100, 100]`` (a ValueError
  outside it; the endpoints are valid).
* ``SetMotorDps`` converts deg/s -> rad/s (``* pi / 180``) then rad/s -> BEMF
  ticks (``/ (ticks_to_rad * 200 Hz)``) with ``round`` to the nearest int.
* ``MoveMotorTo`` / ``MoveMotorRelative`` block until ``get_position()`` is
  within ``tolerance`` ticks of the target and bail on the optional timeout.
  They never stop the motor afterwards (it keeps holding the position).
  ``MoveMotorRelative`` resolves ``current + delta`` itself and commands an
  absolute ``move_to_position``.
* stop modes map OFF -> ``off()``, PASSIVE_BRAKE -> ``set_speed(0)``,
  ACTIVE_BRAKE -> ``brake()``.

We deliberately avoid pinning exact exception-message text or
``_generate_signature()`` formatting (documented non-goals); we assert the
exception TYPE plus a short stable token, the numeric values carried, and the
``motor:<port>`` resource contract.
"""

from __future__ import annotations

import asyncio
import importlib
import math

import pytest

# The conftest pre-imports the package tree before pytest-cov attaches its
# tracer, so the module's import-time def/docstring lines never get counted.
# Reload the module here, under coverage, so those definition statements
# re-execute while traced (same approach as test_wall_align_core.py).
import raccoon.step.motor.steps as motor_steps

motor_steps = importlib.reload(motor_steps)

MotorBrake = motor_steps.MotorBrake
MotorOff = motor_steps.MotorOff
MotorPassiveBrake = motor_steps.MotorPassiveBrake
MoveMotorRelative = motor_steps.MoveMotorRelative
MoveMotorRelativeDegrees = motor_steps.MoveMotorRelativeDegrees
MoveMotorTo = motor_steps.MoveMotorTo
MoveMotorToDegrees = motor_steps.MoveMotorToDegrees
SetMotorDps = motor_steps.SetMotorDps
SetMotorPower = motor_steps.SetMotorPower
SetMotorVelocity = motor_steps.SetMotorVelocity
StopMode = motor_steps.StopMode
StopMotor = motor_steps.StopMotor

# The robot argument is never touched by any motor step's _execute_step.
_ROBOT = object()

_BEMF_SAMPLE_RATE = 200.0  # Hz, per the module docstring (5 ms firmware period)


# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------


class _Calib:
    def __init__(self, ticks_to_rad: float) -> None:
        self.ticks_to_rad = ticks_to_rad


class FakeMotor:
    """Records every firmware command issued to it.

    ``positions`` is a list of readings consumed one-per-``get_position()``
    call (the last value repeats) so the blocking position loops can be driven
    to completion deterministically.
    """

    def __init__(
        self,
        port: int = 0,
        *,
        positions=(0,),
        ticks_to_rad: float = 0.01,
    ) -> None:
        self.port = port
        self.calls: list[tuple] = []
        self.get_position_calls = 0
        self._ticks_to_rad = ticks_to_rad
        self._positions = list(positions)

    # --- commands recorded -------------------------------------------------
    def set_speed(self, value) -> None:
        self.calls.append(("set_speed", value))

    def set_velocity(self, value) -> None:
        self.calls.append(("set_velocity", value))

    def off(self) -> None:
        self.calls.append(("off",))

    def brake(self) -> None:
        self.calls.append(("brake",))

    def move_to_position(self, velocity, position) -> None:
        self.calls.append(("move_to_position", velocity, position))

    def get_position(self) -> int:
        idx = min(self.get_position_calls, len(self._positions) - 1)
        self.get_position_calls += 1
        return self._positions[idx]

    def get_calibration(self) -> _Calib:
        return _Calib(self._ticks_to_rad)


class _NoPort:
    """A non-motor object: deliberately lacks a ``.port`` attribute."""


def _run(coro):
    return asyncio.run(coro)


# ===========================================================================
# StopMode enum contract
# ===========================================================================


def test_stop_mode_values_are_stable_strings() -> None:
    # The .value strings are a public contract (used in signatures/telemetry).
    assert StopMode.OFF.value == "off"
    assert StopMode.PASSIVE_BRAKE.value == "passive_brake"
    assert StopMode.ACTIVE_BRAKE.value == "active_brake"


def test_stop_mode_members_distinct() -> None:
    assert len({StopMode.OFF, StopMode.PASSIVE_BRAKE, StopMode.ACTIVE_BRAKE}) == 3


# ===========================================================================
# SetMotorPower — percent range validation + int coercion + command
# ===========================================================================


@pytest.mark.parametrize("percent", [-100, 0, 100, -100.0, 100.0, 50, -7])
def test_set_motor_power_accepts_in_range(percent) -> None:
    step = SetMotorPower(FakeMotor(), percent)
    assert step._percent == int(percent)


@pytest.mark.parametrize("percent", [101, -101, 200, -200, 1000])
def test_set_motor_power_rejects_out_of_range(percent) -> None:
    with pytest.raises(ValueError, match=r"-100"):
        SetMotorPower(FakeMotor(), percent)


def test_set_motor_power_boundary_just_over_limit() -> None:
    # 100 ok, 101 not — verifies the comparison is inclusive of 100 exactly.
    SetMotorPower(FakeMotor(), 100)  # no raise
    SetMotorPower(FakeMotor(), -100)  # no raise
    with pytest.raises(ValueError):
        SetMotorPower(FakeMotor(), 101)
    with pytest.raises(ValueError):
        SetMotorPower(FakeMotor(), -101)


def test_set_motor_power_truncates_float_toward_zero() -> None:
    # int() truncates toward zero; 100.9 -> 100 (still valid), keep that contract.
    assert SetMotorPower(FakeMotor(), 100.9)._percent == 100
    assert SetMotorPower(FakeMotor(), -100.9)._percent == -100


def test_set_motor_power_float_truncation_can_save_out_of_range() -> None:
    # 100.4 -> int 100 is in range; the check happens on the truncated int.
    assert SetMotorPower(FakeMotor(), 100.4)._percent == 100


def test_set_motor_power_requires_port() -> None:
    with pytest.raises(TypeError, match=r"port"):
        SetMotorPower(_NoPort(), 50)


def test_set_motor_power_resource_name() -> None:
    step = SetMotorPower(FakeMotor(port=3), 50)
    assert step.required_resources() == frozenset({"motor:3"})


def test_set_motor_power_executes_set_speed() -> None:
    motor = FakeMotor(port=1)
    step = SetMotorPower(motor, 50)
    _run(step._execute_step(_ROBOT))
    assert motor.calls == [("set_speed", 50)]


def test_set_motor_power_negative_command_passes_through() -> None:
    motor = FakeMotor()
    _run(SetMotorPower(motor, -100)._execute_step(_ROBOT))
    assert motor.calls == [("set_speed", -100)]


# ===========================================================================
# SetMotorVelocity — int coercion, NO range clamp, command
# ===========================================================================


def test_set_motor_velocity_coerces_to_int() -> None:
    step = SetMotorVelocity(FakeMotor(), 800.7)
    assert step._velocity == 800  # int() truncation


@pytest.mark.parametrize("velocity", [-5000, 0, 5000])
def test_set_motor_velocity_has_no_range_limit(velocity) -> None:
    # Unlike percent, velocity is in raw BEMF units with no documented bound.
    step = SetMotorVelocity(FakeMotor(), velocity)
    assert step._velocity == velocity


def test_set_motor_velocity_requires_port() -> None:
    with pytest.raises(TypeError, match=r"port"):
        SetMotorVelocity(_NoPort(), 100)


def test_set_motor_velocity_resource_name() -> None:
    assert SetMotorVelocity(FakeMotor(port=2), 100).required_resources() == frozenset({"motor:2"})


def test_set_motor_velocity_executes_set_velocity() -> None:
    motor = FakeMotor()
    _run(SetMotorVelocity(motor, 800)._execute_step(_ROBOT))
    assert motor.calls == [("set_velocity", 800)]


# ===========================================================================
# SetMotorDps — deg/s -> BEMF conversion math (computed independently)
# ===========================================================================


def _expected_bemf(dps: float, ticks_to_rad: float) -> int:
    rad_per_sec = dps * math.pi / 180.0
    return int(round(rad_per_sec / (ticks_to_rad * _BEMF_SAMPLE_RATE)))


@pytest.mark.parametrize(
    ("dps", "ticks_to_rad"),
    [
        (180.0, 0.01),
        (90.0, 0.005),
        (-180.0, 0.01),
        (0.0, 0.01),
        (360.0, 0.02),
        (45.0, 0.0123),
        # Large-magnitude BEMF targets: a tiny ticks_to_rad makes the rounded
        # result big enough (~873) that a 0.5% error in the 200 Hz sample rate
        # OR the 180 deg->rad divisor shifts the rounded integer. This is what
        # forces the conversion constants to be exactly 200.0 and 180.0.
        (1000.0, 0.0001),
        (-1000.0, 0.0001),
    ],
)
def test_set_motor_dps_conversion(dps, ticks_to_rad) -> None:
    motor = FakeMotor(ticks_to_rad=ticks_to_rad)
    _run(SetMotorDps(motor, dps)._execute_step(_ROBOT))
    expected = _expected_bemf(dps, ticks_to_rad)
    assert motor.calls == [("set_velocity", expected)]


def test_set_motor_dps_large_target_absolute_value() -> None:
    # Fully hand-computed, independent of the source: 1000 deg/s = 17.4533 rad/s;
    # divide by (ticks_to_rad * 200 Hz) = 0.02 -> 872.66 -> round -> 873.
    # Pins both the deg->rad divisor (180) and the BEMF sample rate (200).
    motor = FakeMotor(ticks_to_rad=0.0001)
    _run(SetMotorDps(motor, 1000.0)._execute_step(_ROBOT))
    assert motor.calls == [("set_velocity", 873)]


def test_set_motor_dps_zero_is_zero() -> None:
    motor = FakeMotor(ticks_to_rad=0.01)
    _run(SetMotorDps(motor, 0.0)._execute_step(_ROBOT))
    assert motor.calls == [("set_velocity", 0)]


def test_set_motor_dps_negative_drives_reverse() -> None:
    motor = FakeMotor(ticks_to_rad=0.01)
    _run(SetMotorDps(motor, -180.0)._execute_step(_ROBOT))
    ((_, value),) = motor.calls
    assert value < 0
    assert value == -_expected_bemf(180.0, 0.01)


def test_set_motor_dps_rounds_to_nearest() -> None:
    # Pick dps/ticks so the unrounded value is clearly fractional and the
    # spec's round() (banker's rounding) result is checkable.
    ticks_to_rad = 0.01
    dps = 30.0  # rad/s = 30*pi/180 = 0.5235987..., /(0.01*200)=0.2617... -> round 0
    assert _expected_bemf(dps, ticks_to_rad) == 0
    motor = FakeMotor(ticks_to_rad=ticks_to_rad)
    _run(SetMotorDps(motor, dps)._execute_step(_ROBOT))
    assert motor.calls == [("set_velocity", 0)]


def test_set_motor_dps_coerces_dps_to_float() -> None:
    step = SetMotorDps(FakeMotor(), 90)
    assert isinstance(step._dps, float)
    assert step._dps == pytest.approx(90.0)


def test_set_motor_dps_requires_port() -> None:
    with pytest.raises(TypeError, match=r"port"):
        SetMotorDps(_NoPort(), 90.0)


def test_set_motor_dps_resource_name() -> None:
    assert SetMotorDps(FakeMotor(port=5), 90.0).required_resources() == frozenset({"motor:5"})


# ===========================================================================
# MoveMotorTo — validation + blocking-until-done + timeout
# ===========================================================================


def test_move_to_requires_port() -> None:
    with pytest.raises(TypeError, match=r"port"):
        MoveMotorTo(_NoPort(), position=100)


@pytest.mark.parametrize("velocity", [0, -1, -1000])
def test_move_to_rejects_nonpositive_velocity(velocity) -> None:
    with pytest.raises(ValueError, match=r"velocity"):
        MoveMotorTo(FakeMotor(), position=100, velocity=velocity)


def test_move_to_accepts_positive_velocity_boundary() -> None:
    # 1 is the smallest valid velocity (> 0, not >= 0).
    step = MoveMotorTo(FakeMotor(), position=100, velocity=1)
    assert step._velocity == 1


def test_move_to_default_velocity_is_1000() -> None:
    assert MoveMotorTo(FakeMotor(), position=100)._velocity == 1000


@pytest.mark.parametrize("timeout", [0, -1, -0.5, 0.0])
def test_move_to_rejects_nonpositive_timeout(timeout) -> None:
    with pytest.raises(ValueError, match=r"timeout"):
        MoveMotorTo(FakeMotor(), position=100, timeout=timeout)


def test_move_to_rejects_non_numeric_timeout() -> None:
    with pytest.raises(ValueError, match=r"timeout"):
        MoveMotorTo(FakeMotor(), position=100, timeout="soon")


def test_move_to_none_timeout_allowed() -> None:
    step = MoveMotorTo(FakeMotor(), position=100, timeout=None)
    assert step._timeout is None


def test_move_to_coerces_position_and_velocity_to_int() -> None:
    step = MoveMotorTo(FakeMotor(), position=100.9, velocity=500.9)
    assert step._position == 100
    assert step._velocity == 500


def test_move_to_resource_name() -> None:
    assert MoveMotorTo(FakeMotor(port=2), position=10).required_resources() == frozenset(
        {"motor:2"}
    )


def test_move_to_rejects_negative_tolerance() -> None:
    with pytest.raises(ValueError, match=r"tolerance"):
        MoveMotorTo(FakeMotor(), position=100, tolerance=-1)


def test_move_to_default_tolerance_is_3() -> None:
    assert MoveMotorTo(FakeMotor(), position=100)._tolerance == 3


def test_move_to_commands_position_then_returns_when_reached() -> None:
    # Already at the target on the first poll -> loop body never sleeps.
    motor = FakeMotor(port=2, positions=[500])
    _run(MoveMotorTo(motor, position=500, velocity=800)._execute_step(_ROBOT))
    assert motor.calls[0] == ("move_to_position", 800, 500)


def test_move_to_polls_until_position_reached() -> None:
    motor = FakeMotor(positions=[0, 200, 400, 500])
    _run(MoveMotorTo(motor, position=500, velocity=800)._execute_step(_ROBOT))
    # command issued exactly once, regardless of poll count
    assert motor.calls.count(("move_to_position", 800, 500)) == 1
    assert motor.get_position_calls == 4


@pytest.mark.parametrize("reading", [497, 503])
def test_move_to_tolerance_boundary_counts_as_reached(reading) -> None:
    # |500 - reading| == 3 == default tolerance -> reached on the first poll.
    motor = FakeMotor(positions=[reading])
    _run(MoveMotorTo(motor, position=500)._execute_step(_ROBOT))
    assert motor.get_position_calls == 1


@pytest.mark.parametrize("reading", [496, 504])
def test_move_to_just_outside_tolerance_keeps_waiting(reading) -> None:
    motor = FakeMotor(positions=[reading, 500])
    _run(MoveMotorTo(motor, position=500)._execute_step(_ROBOT))
    assert motor.get_position_calls == 2


def test_move_to_custom_tolerance() -> None:
    motor = FakeMotor(positions=[480])
    _run(MoveMotorTo(motor, position=500, tolerance=20)._execute_step(_ROBOT))
    assert motor.get_position_calls == 1


def test_move_to_leaves_motor_in_position_mode_after_reaching() -> None:
    # The motor must keep holding: no stop/brake/off/speed command afterwards.
    motor = FakeMotor(positions=[0, 500])
    _run(MoveMotorTo(motor, position=500, velocity=800)._execute_step(_ROBOT))
    assert [c[0] for c in motor.calls] == ["move_to_position"]


def test_move_to_timeout_breaks_out_without_stopping_motor() -> None:
    # Never reached; a tiny timeout must break the loop and NOT issue a stop.
    motor = FakeMotor(positions=[0])
    step = MoveMotorTo(motor, position=500, velocity=800, timeout=0.02)
    _run(step._execute_step(_ROBOT))
    assert [c[0] for c in motor.calls] == ["move_to_position"]


def test_move_to_does_not_time_out_when_completing_under_generous_timeout(monkeypatch) -> None:
    # Elapsed wall-time = (loop.time() - start), tiny vs a 60 s timeout, so the
    # position is reached BEFORE the deadline and no timeout warning is emitted.
    # If the elapsed expression were (loop.time() + start) it would instantly
    # exceed the timeout and warn on the first not-reached poll.
    motor = FakeMotor(positions=[0, 100, 500])
    step = MoveMotorTo(motor, position=500, velocity=800, timeout=60.0)
    warnings: list[str] = []
    monkeypatch.setattr(step, "warn", warnings.append)
    _run(step._execute_step(_ROBOT))
    assert warnings == []


def test_move_to_timeout_warns_with_current_position(monkeypatch) -> None:
    motor = FakeMotor(positions=[42])
    step = MoveMotorTo(motor, position=500, velocity=800, timeout=0.02)
    warnings: list[str] = []
    monkeypatch.setattr(step, "warn", warnings.append)
    _run(step._execute_step(_ROBOT))
    assert len(warnings) == 1
    assert "target=500" in warnings[0] and "current=42" in warnings[0]


def test_move_to_polls_at_a_fast_cadence(monkeypatch) -> None:
    # The poll loop sleeps a short interval (~10 ms) between position checks.
    # Patch asyncio.sleep to a no-op and record the requested delays, then assert
    # each requested sleep is well under a second. A mutated 1.01 s interval
    # would blow this bound (and, unpatched, make the suite hang).
    recorded: list[float] = []

    async def fake_sleep(delay: float) -> None:
        recorded.append(delay)

    monkeypatch.setattr(motor_steps.asyncio, "sleep", fake_sleep)
    motor = FakeMotor(positions=[0, 3, 6, 10])
    _run(MoveMotorTo(motor, position=10, velocity=5, tolerance=0)._execute_step(_ROBOT))
    assert recorded  # the loop did sleep at least once
    assert all(d < 0.5 for d in recorded)


# ===========================================================================
# MoveMotorRelative — validation + blocking-until-done + timeout
# ===========================================================================


def test_move_relative_requires_port() -> None:
    with pytest.raises(TypeError, match=r"port"):
        MoveMotorRelative(_NoPort(), delta=100)


@pytest.mark.parametrize("velocity", [0, -1, -50])
def test_move_relative_rejects_nonpositive_velocity(velocity) -> None:
    with pytest.raises(ValueError, match=r"velocity"):
        MoveMotorRelative(FakeMotor(), delta=100, velocity=velocity)


def test_move_relative_accepts_positive_velocity_boundary() -> None:
    # velocity == 1 is the smallest accepted value: the guard is `> 0`, NOT
    # `> 1`. This pins the exact comparison constant.
    step = MoveMotorRelative(FakeMotor(), delta=100, velocity=1)
    assert step._velocity == 1


def test_move_relative_default_velocity_is_1000() -> None:
    assert MoveMotorRelative(FakeMotor(), delta=100)._velocity == 1000


@pytest.mark.parametrize("timeout", [0, -1, 0.0])
def test_move_relative_rejects_nonpositive_timeout(timeout) -> None:
    with pytest.raises(ValueError, match=r"timeout"):
        MoveMotorRelative(FakeMotor(), delta=100, timeout=timeout)


def test_move_relative_rejects_non_numeric_timeout() -> None:
    with pytest.raises(ValueError, match=r"timeout"):
        MoveMotorRelative(FakeMotor(), delta=100, timeout=object())


def test_move_relative_coerces_delta_to_int() -> None:
    step = MoveMotorRelative(FakeMotor(), delta=200.7, velocity=600)
    assert step._delta == 200


def test_move_relative_negative_delta_allowed() -> None:
    # delta is signed (forward/reverse); only velocity is constrained > 0.
    step = MoveMotorRelative(FakeMotor(), delta=-300, velocity=600)
    assert step._delta == -300


def test_move_relative_resource_name() -> None:
    assert MoveMotorRelative(FakeMotor(port=4), delta=10).required_resources() == frozenset(
        {"motor:4"}
    )


def test_move_relative_rejects_negative_tolerance() -> None:
    with pytest.raises(ValueError, match=r"tolerance"):
        MoveMotorRelative(FakeMotor(), delta=100, tolerance=-1)


def test_move_relative_commands_absolute_goal_current_plus_delta() -> None:
    # Start at 1000; delta 200 -> absolute goal 1200 commanded via move_to_position.
    motor = FakeMotor(positions=[1000, 1200])
    _run(MoveMotorRelative(motor, delta=200, velocity=600)._execute_step(_ROBOT))
    assert motor.calls == [("move_to_position", 600, 1200)]


def test_move_relative_negative_delta_goal() -> None:
    motor = FakeMotor(positions=[1000, 700])
    _run(MoveMotorRelative(motor, delta=-300, velocity=600)._execute_step(_ROBOT))
    assert motor.calls == [("move_to_position", 600, 700)]


def test_move_relative_polls_until_position_reached() -> None:
    motor = FakeMotor(positions=[0, 50, 150, 200])
    _run(MoveMotorRelative(motor, delta=200, velocity=600)._execute_step(_ROBOT))
    assert motor.calls.count(("move_to_position", 600, 200)) == 1
    assert motor.get_position_calls == 4  # 1 start read + 3 polls


def test_move_relative_tolerance_applies_to_goal() -> None:
    motor = FakeMotor(positions=[0, 197])
    _run(MoveMotorRelative(motor, delta=200)._execute_step(_ROBOT))
    assert motor.get_position_calls == 2


def test_move_relative_leaves_motor_in_position_mode_after_reaching() -> None:
    motor = FakeMotor(positions=[0, 200])
    _run(MoveMotorRelative(motor, delta=200, velocity=600)._execute_step(_ROBOT))
    assert [c[0] for c in motor.calls] == ["move_to_position"]


def test_move_relative_timeout_breaks_without_stopping() -> None:
    motor = FakeMotor(positions=[0])
    step = MoveMotorRelative(motor, delta=200, velocity=600, timeout=0.02)
    _run(step._execute_step(_ROBOT))
    assert [c[0] for c in motor.calls] == ["move_to_position"]


def test_move_relative_does_not_time_out_when_completing_under_generous_timeout(
    monkeypatch,
) -> None:
    # Same elapsed-time sign check as MoveMotorTo.
    motor = FakeMotor(positions=[0, 100, 200])
    step = MoveMotorRelative(motor, delta=200, velocity=600, timeout=60.0)
    warnings: list[str] = []
    monkeypatch.setattr(step, "warn", warnings.append)
    _run(step._execute_step(_ROBOT))
    assert warnings == []


def test_move_relative_timeout_warns_with_current_position(monkeypatch) -> None:
    motor = FakeMotor(positions=[7])
    step = MoveMotorRelative(motor, delta=200, velocity=600, timeout=0.02)
    warnings: list[str] = []
    monkeypatch.setattr(step, "warn", warnings.append)
    _run(step._execute_step(_ROBOT))
    assert len(warnings) == 1
    assert "target=207" in warnings[0] and "current=7" in warnings[0]


def test_move_relative_polls_at_a_fast_cadence(monkeypatch) -> None:
    recorded: list[float] = []

    async def fake_sleep(delay: float) -> None:
        recorded.append(delay)

    monkeypatch.setattr(motor_steps.asyncio, "sleep", fake_sleep)
    motor = FakeMotor(positions=[0, 0, 2, 5])
    _run(MoveMotorRelative(motor, delta=5, velocity=5, tolerance=0)._execute_step(_ROBOT))
    assert recorded
    assert all(d < 0.5 for d in recorded)


# ===========================================================================
# MoveMotorToDegrees / MoveMotorRelativeDegrees — same as the tick steps,
# but the target is an angle converted via the calibration's ticks_to_rad
# ===========================================================================

# 1 tick == 1 degree: makes the expected tick values equal the degree values.
_ONE_DEG_PER_TICK = math.pi / 180.0


def _deg_motor(**kwargs) -> FakeMotor:
    return FakeMotor(ticks_to_rad=_ONE_DEG_PER_TICK, **kwargs)


def test_move_to_degrees_requires_port() -> None:
    with pytest.raises(TypeError, match=r"port"):
        MoveMotorToDegrees(_NoPort(), degrees=90)


@pytest.mark.parametrize("velocity", [0, -1, -1000])
def test_move_to_degrees_rejects_nonpositive_velocity(velocity) -> None:
    with pytest.raises(ValueError, match=r"velocity"):
        MoveMotorToDegrees(FakeMotor(), degrees=90, velocity=velocity)


@pytest.mark.parametrize("timeout", [0, -1, 0.0])
def test_move_to_degrees_rejects_nonpositive_timeout(timeout) -> None:
    with pytest.raises(ValueError, match=r"timeout"):
        MoveMotorToDegrees(FakeMotor(), degrees=90, timeout=timeout)


def test_move_to_degrees_rejects_non_numeric_timeout() -> None:
    with pytest.raises(ValueError, match=r"timeout"):
        MoveMotorToDegrees(FakeMotor(), degrees=90, timeout="soon")


def test_move_to_degrees_rejects_negative_tolerance() -> None:
    with pytest.raises(ValueError, match=r"tolerance"):
        MoveMotorToDegrees(FakeMotor(), degrees=90, tolerance=-1)


def test_move_to_degrees_defaults_match_tick_step() -> None:
    step = MoveMotorToDegrees(FakeMotor(), degrees=90)
    assert step._velocity == 1000
    assert step._timeout is None
    assert step._tolerance == 3


def test_move_to_degrees_resource_name() -> None:
    step = MoveMotorToDegrees(FakeMotor(port=3), degrees=10)
    assert step.required_resources() == frozenset({"motor:3"})


def test_move_to_degrees_commands_absolute_ticks_with_unchanged_velocity() -> None:
    motor = _deg_motor(positions=[90])
    _run(MoveMotorToDegrees(motor, degrees=90, velocity=800)._execute_step(_ROBOT))
    assert motor.calls == [("move_to_position", 800, 90)]


def test_move_to_degrees_uses_calibration_ticks_to_rad() -> None:
    # ticks_to_rad = 0.01 rad -> 180 deg = pi rad = 314.159.. ticks -> 314.
    motor = FakeMotor(ticks_to_rad=0.01, positions=[314])
    _run(MoveMotorToDegrees(motor, degrees=180)._execute_step(_ROBOT))
    assert motor.calls[0][2] == round(math.pi / 0.01)


def test_move_to_degrees_negative_angle() -> None:
    motor = _deg_motor(positions=[-45])
    _run(MoveMotorToDegrees(motor, degrees=-45)._execute_step(_ROBOT))
    assert motor.calls[0][2] == -45


def test_move_to_degrees_polls_until_reached_and_keeps_holding() -> None:
    motor = _deg_motor(positions=[0, 40, 90])
    _run(MoveMotorToDegrees(motor, degrees=90)._execute_step(_ROBOT))
    assert [c[0] for c in motor.calls] == ["move_to_position"]
    assert motor.get_position_calls == 3


def test_move_to_degrees_tolerance_in_ticks() -> None:
    motor = _deg_motor(positions=[87])  # |90-87| == 3 == default
    _run(MoveMotorToDegrees(motor, degrees=90)._execute_step(_ROBOT))
    assert motor.get_position_calls == 1
    motor = _deg_motor(positions=[86, 90])  # 4 > 3 -> keeps waiting
    _run(MoveMotorToDegrees(motor, degrees=90)._execute_step(_ROBOT))
    assert motor.get_position_calls == 2
    motor = _deg_motor(positions=[80])
    _run(MoveMotorToDegrees(motor, degrees=90, tolerance=10)._execute_step(_ROBOT))
    assert motor.get_position_calls == 1


def test_move_to_degrees_timeout_warns_without_stopping(monkeypatch) -> None:
    motor = _deg_motor(positions=[0])
    step = MoveMotorToDegrees(motor, degrees=90, timeout=0.02)
    warnings: list[str] = []
    monkeypatch.setattr(step, "warn", warnings.append)
    _run(step._execute_step(_ROBOT))
    assert len(warnings) == 1
    assert "target=90" in warnings[0]
    assert [c[0] for c in motor.calls] == ["move_to_position"]


def test_move_to_degrees_without_calibration_raises() -> None:
    motor = FakeMotor(ticks_to_rad=0.0)
    with pytest.raises(ValueError, match=r"ticks_to_rad"):
        _run(MoveMotorToDegrees(motor, degrees=90)._execute_step(_ROBOT))
    assert motor.calls == []


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_move_to_degrees_rejects_non_finite_angle(bad) -> None:
    motor = _deg_motor()
    with pytest.raises(ValueError, match=r"finite"):
        _run(MoveMotorToDegrees(motor, degrees=bad)._execute_step(_ROBOT))
    assert motor.calls == []


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_move_to_degrees_rejects_non_finite_calibration(bad) -> None:
    motor = FakeMotor(ticks_to_rad=bad)
    with pytest.raises(ValueError, match=r"ticks_to_rad"):
        _run(MoveMotorToDegrees(motor, degrees=90)._execute_step(_ROBOT))
    assert motor.calls == []


def test_move_relative_degrees_rejects_non_finite_angle() -> None:
    motor = _deg_motor(positions=[0])
    with pytest.raises(ValueError, match=r"finite"):
        _run(MoveMotorRelativeDegrees(motor, delta_degrees=float("nan"))._execute_step(_ROBOT))
    assert motor.calls == []


def test_move_relative_degrees_requires_port() -> None:
    with pytest.raises(TypeError, match=r"port"):
        MoveMotorRelativeDegrees(_NoPort(), delta_degrees=90)


@pytest.mark.parametrize("velocity", [0, -5])
def test_move_relative_degrees_rejects_nonpositive_velocity(velocity) -> None:
    with pytest.raises(ValueError, match=r"velocity"):
        MoveMotorRelativeDegrees(FakeMotor(), delta_degrees=90, velocity=velocity)


@pytest.mark.parametrize("timeout", [0, -1])
def test_move_relative_degrees_rejects_nonpositive_timeout(timeout) -> None:
    with pytest.raises(ValueError, match=r"timeout"):
        MoveMotorRelativeDegrees(FakeMotor(), delta_degrees=90, timeout=timeout)


def test_move_relative_degrees_rejects_negative_tolerance() -> None:
    with pytest.raises(ValueError, match=r"tolerance"):
        MoveMotorRelativeDegrees(FakeMotor(), delta_degrees=90, tolerance=-1)


def test_move_relative_degrees_defaults_match_tick_step() -> None:
    step = MoveMotorRelativeDegrees(FakeMotor(), delta_degrees=90)
    assert (step._velocity, step._timeout, step._tolerance) == (1000, None, 3)


def test_move_relative_degrees_commands_current_plus_delta() -> None:
    # Start at 1000 ticks, +90 deg at 1 tick/deg -> absolute goal 1090.
    motor = _deg_motor(positions=[1000, 1090])
    step = MoveMotorRelativeDegrees(motor, delta_degrees=90, velocity=600)
    _run(step._execute_step(_ROBOT))
    assert motor.calls == [("move_to_position", 600, 1090)]


def test_move_relative_degrees_negative_delta() -> None:
    motor = _deg_motor(positions=[500, 410])
    _run(MoveMotorRelativeDegrees(motor, delta_degrees=-90)._execute_step(_ROBOT))
    assert motor.calls[0][2] == 410


def test_move_relative_degrees_leaves_motor_holding() -> None:
    motor = _deg_motor(positions=[0, 0, 90])
    _run(MoveMotorRelativeDegrees(motor, delta_degrees=90)._execute_step(_ROBOT))
    assert [c[0] for c in motor.calls] == ["move_to_position"]


def test_move_relative_degrees_timeout_warns_without_stopping(monkeypatch) -> None:
    motor = _deg_motor(positions=[0])
    step = MoveMotorRelativeDegrees(motor, delta_degrees=90, timeout=0.02)
    warnings: list[str] = []
    monkeypatch.setattr(step, "warn", warnings.append)
    _run(step._execute_step(_ROBOT))
    assert len(warnings) == 1
    assert [c[0] for c in motor.calls] == ["move_to_position"]


def test_degree_steps_signature_mentions_port() -> None:
    assert "port=2" in MoveMotorToDegrees(FakeMotor(port=2), degrees=5)._generate_signature()
    sig = MoveMotorRelativeDegrees(FakeMotor(port=2), delta_degrees=5)._generate_signature()
    assert "port=2" in sig


# ===========================================================================
# StopMotor + the three convenience subclasses — mode dispatch
# ===========================================================================


def test_stop_motor_default_mode_is_active_brake() -> None:
    step = StopMotor(FakeMotor())
    assert step._mode is StopMode.ACTIVE_BRAKE


def test_stop_motor_requires_port() -> None:
    with pytest.raises(TypeError, match=r"port"):
        StopMotor(_NoPort())


def test_stop_motor_resource_name() -> None:
    assert StopMotor(FakeMotor(port=6)).required_resources() == frozenset({"motor:6"})


@pytest.mark.parametrize(
    ("mode", "expected_call"),
    [
        (StopMode.OFF, ("off",)),
        (StopMode.PASSIVE_BRAKE, ("set_speed", 0)),
        (StopMode.ACTIVE_BRAKE, ("brake",)),
    ],
)
def test_stop_motor_dispatches_by_mode(mode, expected_call) -> None:
    motor = FakeMotor()
    _run(StopMotor(motor, mode)._execute_step(_ROBOT))
    assert motor.calls == [expected_call]


def test_motor_off_uses_off_command() -> None:
    motor = FakeMotor()
    step = MotorOff(motor)
    assert step._mode is StopMode.OFF
    _run(step._execute_step(_ROBOT))
    assert motor.calls == [("off",)]


def test_motor_passive_brake_commands_zero_speed() -> None:
    motor = FakeMotor()
    step = MotorPassiveBrake(motor)
    assert step._mode is StopMode.PASSIVE_BRAKE
    _run(step._execute_step(_ROBOT))
    assert motor.calls == [("set_speed", 0)]


def test_motor_brake_uses_brake_command() -> None:
    motor = FakeMotor()
    step = MotorBrake(motor)
    assert step._mode is StopMode.ACTIVE_BRAKE
    _run(step._execute_step(_ROBOT))
    assert motor.calls == [("brake",)]


def test_stop_subclasses_require_port() -> None:
    for cls in (MotorOff, MotorPassiveBrake, MotorBrake):
        with pytest.raises(TypeError, match=r"port"):
            cls(_NoPort())


# ===========================================================================
# DSL registration contract — decorator presence, tags, and hidden flag.
#
# These are STRUCTURED contracts (not prose): the @dsl_step / @dsl decorators
# stamp marker attributes the codegen + docs catalog consume. Removing a
# decorator or mutating its tags/hidden flag is a real behaviour change, so we
# assert the resulting class attributes rather than any message text.
# Tags drive documentation grouping (CLAUDE.md), so the exact tag tuple is a
# contract worth pinning.
# ===========================================================================


_DSL_STEP_CLASSES = [
    SetMotorPower,
    SetMotorVelocity,
    SetMotorDps,
    MoveMotorTo,
    MoveMotorRelative,
    MotorOff,
    MotorPassiveBrake,
    MotorBrake,
]


@pytest.mark.parametrize("cls", _DSL_STEP_CLASSES, ids=lambda c: c.__name__)
def test_dsl_step_decorator_present(cls) -> None:
    # @dsl_step stamps __dsl_step__ and a DslMeta in the class's OWN __dict__.
    # If the decorator were removed, the marker would be absent on the class
    # itself (and not inherited, since the StopMotor base is @dsl, not @dsl_step).
    assert cls.__dict__.get("__dsl_step__") is True
    assert "__dsl__" in cls.__dict__


@pytest.mark.parametrize("cls", _DSL_STEP_CLASSES, ids=lambda c: c.__name__)
def test_dsl_step_tags_are_motor_actuator(cls) -> None:
    # Exact tag tuple is the documentation-grouping contract: primary "motor",
    # secondary "actuator", in that order.
    assert cls.__dict__.get("__dsl_step_tags__") == ("motor", "actuator")
    assert cls.__dict__.get("__dsl_tags__") == ("motor", "actuator")
    assert cls.__dsl__.tags == ("motor", "actuator")


@pytest.mark.parametrize("cls", _DSL_STEP_CLASSES, ids=lambda c: c.__name__)
def test_dsl_step_is_hidden(cls) -> None:
    # The raw class is hidden; users go through the generated factory.
    assert cls.__dict__.get("__dsl_hidden__") is True
    assert cls.__dsl__.hidden is True


def test_stop_motor_is_dsl_hidden() -> None:
    # StopMotor uses the manual @dsl(hidden=True) pattern: it must carry its own
    # DslMeta with hidden=True. A hidden=False mutation flips this contract.
    assert "__dsl__" in StopMotor.__dict__
    assert StopMotor.__dict__["__dsl__"].hidden is True
    assert StopMotor.__dict__.get("__dsl_hidden__") is True
    # StopMotor is the manual base, not a codegen @dsl_step.
    assert "__dsl_step__" not in StopMotor.__dict__


# ===========================================================================
# Signatures — assert stable semantic tokens only (not exact formatting)
# ===========================================================================


def test_signatures_carry_port_and_value_tokens() -> None:
    assert "port=3" in SetMotorPower(FakeMotor(port=3), 50)._generate_signature()
    assert "50" in SetMotorPower(FakeMotor(port=3), 50)._generate_signature()
    assert "port=1" in SetMotorVelocity(FakeMotor(port=1), 800)._generate_signature()
    assert "800" in SetMotorVelocity(FakeMotor(port=1), 800)._generate_signature()
    dps_sig = SetMotorDps(FakeMotor(port=7), 90.0)._generate_signature()
    assert "port=7" in dps_sig
    assert "90" in dps_sig
    assert "port=2" in MoveMotorTo(FakeMotor(port=2), position=10)._generate_signature()
    assert "port=4" in MoveMotorRelative(FakeMotor(port=4), delta=5)._generate_signature()
    assert StopMode.OFF.value in StopMotor(FakeMotor(), StopMode.OFF)._generate_signature()
