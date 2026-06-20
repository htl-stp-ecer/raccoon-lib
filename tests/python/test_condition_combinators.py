"""Tests for the composable StopCondition combinators and primitives.

These exercise the pure-Python control logic in
``raccoon.step.condition`` without any C++ drivetrain:

* operator combinators ``|`` (_AnyOf), ``&`` (_AllOf), ``>``/``+`` (_Then),
  including the two-phase state machine, short-circuit semantics and the
  chained-comparison ``__bool__`` guard.
* the sensor / odometry / time primitives: ``on_black``/``on_white``
  threshold logic + validation, ``after_seconds`` elapsed logic,
  ``after_cm`` cm->m conversion + relative/absolute mode,
  ``after_forward_cm``/``after_lateral_cm`` axis projection,
  ``after_degrees`` wrap, ``on_digital``, ``on_analog_above/below``,
  ``stall_detected``, ``custom`` and ``over_line``.

All robot interactions go through lightweight fakes; ``time.monotonic`` is
monkeypatched where deterministic timing matters.
"""

from __future__ import annotations

import math

import pytest

from raccoon.step import condition as cond
from raccoon.step.condition import (
    StopCondition,
    after_cm,
    after_degrees,
    after_forward_cm,
    after_lateral_cm,
    after_seconds,
    custom,
    on_analog_above,
    on_analog_below,
    on_black,
    on_digital,
    on_white,
    over_line,
    stall_detected,
)

ROBOT = object()  # sentinel: most primitives never touch the robot


# --------------------------------------------------------------------------- #
# Test doubles                                                                 #
# --------------------------------------------------------------------------- #
class FakeSensor:
    """IR sensor stand-in with controllable black/white probabilities."""

    def __init__(self, black: float = 0.0, white: float = 0.0) -> None:
        self.black = black
        self.white = white

    def probabilityOfBlack(self) -> float:
        return self.black

    def probabilityOfWhite(self) -> float:
        return self.white


class FakeDigital:
    def __init__(self, value: bool = False) -> None:
        self.value = value

    def read(self) -> bool:
        return self.value


class FakeAnalog:
    def __init__(self, value: int = 0) -> None:
        self.value = value

    def read(self) -> int:
        return self.value


class FakeMotor:
    def __init__(self, position: int = 0) -> None:
        self.position = position

    def get_position(self) -> int:
        return self.position


class FakeCondition(StopCondition):
    """Records start()/check() calls; check() returns a scripted sequence."""

    def __init__(self, results=None) -> None:
        self._results = list(results) if results is not None else [False]
        self.start_count = 0
        self.check_count = 0

    def start(self, robot) -> None:
        self.start_count += 1

    def check(self, robot) -> bool:
        self.check_count += 1
        if len(self._results) > 1:
            return self._results.pop(0)
        return self._results[0]


class _Pose:
    def __init__(self, x=0.0, y=0.0, heading=0.0):
        self.position = (x, y)
        self.heading = heading


class _Dist:
    def __init__(self, forward=0.0, lateral=0.0):
        self.forward = forward
        self.lateral = lateral


class FakeOdometry:
    def __init__(self):
        self._path = 0.0
        self._pose = _Pose()
        self._dist = _Dist()

    def get_path_length(self):
        return self._path

    def get_pose(self):
        return self._pose

    def get_distance_from_origin(self):
        return self._dist


class FakeRobot:
    def __init__(self):
        self.odometry = FakeOdometry()


# --------------------------------------------------------------------------- #
# Base StopCondition contract                                                  #
# --------------------------------------------------------------------------- #
def test_base_check_not_implemented():
    with pytest.raises(NotImplementedError):
        StopCondition().check(ROBOT)


def test_base_start_is_noop():
    # Default start does nothing and returns None.
    assert StopCondition().start(ROBOT) is None


def test_bool_guard_raises_with_helpful_message():
    # Behavioural: the guard must reject bool() with a TypeError that names the
    # offending type. Exact prose is intentionally not pinned.
    with pytest.raises(TypeError, match=r"StopCondition"):
        bool(StopCondition())


def test_chained_comparison_triggers_bool_guard():
    # a > b > c => (a > b) and (b > c) => bool((a>b)) -> raises.
    a, b, c = FakeCondition(), FakeCondition(), FakeCondition()
    with pytest.raises(TypeError):
        _ = a > b > c


# --------------------------------------------------------------------------- #
# _AnyOf  (|)                                                                  #
# --------------------------------------------------------------------------- #
def test_anyof_starts_all_children():
    a, b = FakeCondition([False]), FakeCondition([False])
    combo = a | b
    combo.start(ROBOT)
    assert a.start_count == 1
    assert b.start_count == 1


def test_anyof_true_when_any_true():
    assert (FakeCondition([False]) | FakeCondition([True])).check(ROBOT) is True
    assert (FakeCondition([True]) | FakeCondition([False])).check(ROBOT) is True


def test_anyof_false_when_all_false():
    assert (FakeCondition([False]) | FakeCondition([False])).check(ROBOT) is False


def test_anyof_short_circuits_on_first_true():
    a = FakeCondition([True])
    b = FakeCondition([False])
    combo = a | b
    assert combo.check(ROBOT) is True
    assert a.check_count == 1
    # any() short-circuits: second is never evaluated.
    assert b.check_count == 0


def test_anyof_rejects_non_condition():
    # Position index (1) and rejected type (int) are behavioural; prose is not.
    with pytest.raises(TypeError, match=r"position 1") as exc:
        cond._AnyOf(FakeCondition(), 5)
    assert "int" in str(exc.value)


# --------------------------------------------------------------------------- #
# _AllOf  (&)                                                                  #
# --------------------------------------------------------------------------- #
def test_allof_starts_all_children():
    a, b = FakeCondition([False]), FakeCondition([False])
    (a & b).start(ROBOT)
    assert a.start_count == 1 and b.start_count == 1


def test_allof_true_only_when_all_true():
    assert (FakeCondition([True]) & FakeCondition([True])).check(ROBOT) is True
    assert (FakeCondition([True]) & FakeCondition([False])).check(ROBOT) is False
    assert (FakeCondition([False]) & FakeCondition([True])).check(ROBOT) is False


def test_allof_short_circuits_on_first_false():
    a = FakeCondition([False])
    b = FakeCondition([True])
    combo = a & b
    assert combo.check(ROBOT) is False
    assert a.check_count == 1
    assert b.check_count == 0  # all() short-circuits.


def test_allof_rejects_non_condition():
    # Position index (0) and rejected type (float) are behavioural; prose is not.
    with pytest.raises(TypeError, match=r"position 0") as exc:
        cond._AllOf(0.5, FakeCondition())
    assert "float" in str(exc.value)


# --------------------------------------------------------------------------- #
# _Then  (> and +)                                                            #
# --------------------------------------------------------------------------- #
def test_then_second_not_started_until_first_fires():
    first = FakeCondition([False, True])
    second = FakeCondition([True])
    combo = first + second
    combo.start(ROBOT)
    assert first.start_count == 1
    assert second.start_count == 0  # second not started yet


def test_then_two_phase_state_machine():
    first = FakeCondition([False, True])  # fires on 2nd check
    second = FakeCondition([False, True])  # fires on 2nd check after start
    combo = first + second
    combo.start(ROBOT)

    # Phase 1: first not done -> always returns False.
    assert combo.check(ROBOT) is False  # first.check -> False
    assert second.start_count == 0

    # first fires now; _Then records done, starts second, still returns False.
    assert combo.check(ROBOT) is False
    assert combo._first_done is True
    assert second.start_count == 1

    # Phase 2: now delegating to second.
    assert combo.check(ROBOT) is False  # second.check -> False
    assert combo.check(ROBOT) is True  # second.check -> True


def test_then_first_done_initialised_false():
    """A freshly constructed _Then starts in phase 1 (first not done).

    Checking before start() must evaluate *first* (which returns False here),
    NOT delegate to second. Kills the `_first_done = True/None` init mutants.
    """
    first = FakeCondition([False])
    second = FakeCondition([True])  # would return True if wrongly delegated to
    combo = first + second
    assert combo._first_done is False
    # No start(): check() in phase 1 evaluates first (False), returns False.
    assert combo.check(ROBOT) is False
    assert first.check_count == 1
    assert second.check_count == 0  # second never consulted in phase 1


def test_then_start_resets_first_done():
    first = FakeCondition([True])
    second = FakeCondition([False])
    combo = first + second
    combo.start(ROBOT)
    combo.check(ROBOT)  # first fires -> _first_done True
    assert combo._first_done is True
    combo.start(ROBOT)  # restart
    assert combo._first_done is False
    assert first.start_count == 2


def test_then_gt_alias_same_behaviour():
    combo = FakeCondition([True]) > FakeCondition([True])
    assert isinstance(combo, cond._Then)


def test_then_three_stage_chain_order():
    a = FakeCondition([True])
    b = FakeCondition([True])
    c = FakeCondition([True])
    # a + b + c == _Then(_Then(a, b), c)
    combo = a + b + c
    combo.start(ROBOT)
    # First check: outer first is _Then(a,b); a fires, b starts but inner
    # returns False (just transitioned). outer first not done -> False.
    assert combo.check(ROBOT) is False
    assert a.check_count == 1 and b.start_count == 1 and b.check_count == 0
    # Second check: inner _Then(a,b) delegates to b which fires -> inner True,
    # outer marks first_done, starts c, returns False.
    assert combo.check(ROBOT) is False
    assert b.check_count == 1 and c.start_count == 1
    # Third check: outer delegates to c -> True.
    assert combo.check(ROBOT) is True
    assert c.check_count == 1


def test_then_four_stage_chain_eventually_true():
    a, b, c, d = (FakeCondition([True]) for _ in range(4))
    combo = a + b + c + d
    combo.start(ROBOT)
    # Drive enough cycles; should become True exactly once all four fired.
    results = [combo.check(ROBOT) for _ in range(6)]
    assert results[-1] is True
    assert any(r is False for r in results[:-1])


def test_then_rejects_non_condition_first():
    # Rejected type (int) is behavioural; the surrounding prose is not.
    with pytest.raises(TypeError, match=r"StopCondition") as exc:
        cond._Then(5, FakeCondition())
    assert "int" in str(exc.value)


def test_then_rejects_non_condition_second():
    with pytest.raises(TypeError, match=r"StopCondition") as exc:
        cond._Then(FakeCondition(), "x")
    assert "str" in str(exc.value)


# --------------------------------------------------------------------------- #
# on_black                                                                     #
# --------------------------------------------------------------------------- #
def test_on_black_default_threshold():
    s = FakeSensor(black=0.7)
    assert on_black(s).check(ROBOT) is True
    s.black = 0.69
    assert on_black(s).check(ROBOT) is False


def test_on_black_boundary_is_inclusive():
    # >= threshold
    s = FakeSensor(black=0.5)
    assert on_black(s, threshold=0.5).check(ROBOT) is True


def test_on_black_custom_threshold():
    s = FakeSensor(black=0.4)
    assert on_black(s, threshold=0.3).check(ROBOT) is True
    assert on_black(s, threshold=0.41).check(ROBOT) is False


def test_on_black_threshold_bounds_valid():
    on_black(FakeSensor(), threshold=0.0)
    on_black(FakeSensor(), threshold=1.0)


@pytest.mark.parametrize("bad", [-0.01, 1.01, 2.0])
def test_on_black_threshold_out_of_range_raises(bad):
    with pytest.raises(ValueError, match="threshold must be"):
        on_black(FakeSensor(), threshold=bad)


def test_on_black_rejects_non_sensor():
    # Behavioural: names the missing capability and the rejected type.
    with pytest.raises(TypeError, match=r"probabilityOfBlack") as exc:
        on_black(object())
    assert "object" in str(exc.value)


def test_on_black_threshold_message_echoes_value():
    # The rejected value (1.5) is behavioural and must be echoed; prose is not.
    with pytest.raises(ValueError, match=r"threshold must be") as exc:
        on_black(FakeSensor(), threshold=1.5)
    assert "1.5" in str(exc.value)


# --------------------------------------------------------------------------- #
# on_white                                                                     #
# --------------------------------------------------------------------------- #
def test_on_white_default_and_boundary():
    s = FakeSensor(white=0.7)
    assert on_white(s).check(ROBOT) is True
    s.white = 0.7
    assert on_white(s, threshold=0.7).check(ROBOT) is True
    s.white = 0.69
    assert on_white(s).check(ROBOT) is False


def test_on_white_threshold_out_of_range_raises():
    # The rejected value (-1.0) is behavioural; prose is not.
    with pytest.raises(ValueError, match=r"threshold must be") as exc:
        on_white(FakeSensor(), threshold=-1.0)
    assert "-1.0" in str(exc.value)


def test_on_white_threshold_bounds_inclusive():
    # Exactly 0.0 and exactly 1.0 are accepted (kills < / <= boundary mutants).
    on_white(FakeSensor(), threshold=0.0)
    on_white(FakeSensor(), threshold=1.0)


def test_on_white_threshold_above_one_rejected():
    # 1.5 must raise: kills the <= 2.0 upper-bound mutant.
    with pytest.raises(ValueError, match="threshold must be"):
        on_white(FakeSensor(), threshold=1.5)


def test_on_white_rejects_object_without_method():
    with pytest.raises(TypeError, match=r"probabilityOfWhite") as exc:
        on_white(object())
    assert "object" in str(exc.value)


# --------------------------------------------------------------------------- #
# after_seconds                                                                #
# --------------------------------------------------------------------------- #
def test_after_seconds_elapsed_logic(monkeypatch):
    clock = {"t": 100.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    c = after_seconds(5.0)
    c.start(ROBOT)  # deadline = 105
    assert c.check(ROBOT) is False
    clock["t"] = 104.999
    assert c.check(ROBOT) is False
    clock["t"] = 105.0  # exactly deadline -> >= True
    assert c.check(ROBOT) is True
    clock["t"] = 200.0
    assert c.check(ROBOT) is True


def test_after_seconds_deadline_initialised_zero(monkeypatch):
    """Before start(), _deadline is 0, so check() is True once the clock is
    past 0. Kills the `_deadline = 1` / `_deadline = None` init mutants:
    with deadline 1 the check would be False at t=0.5; with None it raises.
    """
    monkeypatch.setattr(cond.time, "monotonic", lambda: 0.5)
    c = after_seconds(5.0)  # never started
    assert c.check(ROBOT) is True  # 0.5 >= 0 (real deadline)


def test_after_seconds_zero_immediately_true(monkeypatch):
    clock = {"t": 50.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    c = after_seconds(0)
    c.start(ROBOT)
    assert c.check(ROBOT) is True


def test_after_seconds_negative_raises():
    # Rejected value (-1) is behavioural; prose is not.
    with pytest.raises(ValueError, match=r"seconds must be") as exc:
        after_seconds(-1)
    assert "-1" in str(exc.value)


def test_after_seconds_non_number_raises():
    # Rejected type (str) is behavioural; prose is not.
    with pytest.raises(TypeError, match=r"seconds must be") as exc:
        after_seconds("5")
    assert "str" in str(exc.value)


# --------------------------------------------------------------------------- #
# after_cm                                                                     #
# --------------------------------------------------------------------------- #
def test_after_cm_converts_cm_to_m():
    c = after_cm(50)
    assert c._target_m == pytest.approx(0.5)


def test_after_cm_relative_mode_baseline():
    r = FakeRobot()
    r.odometry._path = 2.0
    c = after_cm(100)  # 1.0 m target
    c.start(r)
    assert c._baseline_m == pytest.approx(2.0)
    # traveled 0.99 m -> not yet
    r.odometry._path = 2.99
    assert c.check(r) is False
    # traveled exactly 1.0 m -> True
    r.odometry._path = 3.0
    assert c.check(r) is True


def test_after_cm_check_false_before_start():
    r = FakeRobot()
    # Path is already well past the target; check() must still be False because
    # the condition has not started. Kills the `_started = True` init mutant.
    r.odometry._path = 999.0
    c = after_cm(10)
    assert c.check(r) is False  # not started


def test_after_cm_cm_one_is_valid():
    # cm == 1 is allowed (> 0). Kills the `cm <= 1` boundary mutant.
    c = after_cm(1)
    assert c._target_m == pytest.approx(0.01)


def test_after_cm_absolute_mode_ignores_prior_distance():
    r = FakeRobot()
    r.odometry._path = 5.0
    c = after_cm(100, absolute=True)
    c.start(r)
    assert c._baseline_m == 0.0
    # absolute: measures from origin; 0.99 m not enough... but path is 5.0
    # so it is already >= 1.0 immediately.
    assert c.check(r) is True


def test_after_cm_zero_raises():
    # Rejected value (0) is behavioural; prose is not.
    with pytest.raises(ValueError, match=r"cm must be") as exc:
        after_cm(0)
    assert "0" in str(exc.value)


def test_after_cm_negative_raises():
    with pytest.raises(ValueError, match="cm must be > 0"):
        after_cm(-10)


def test_after_cm_non_number_raises():
    # Rejected type (str) is behavioural; prose is not.
    with pytest.raises(TypeError, match=r"cm must be") as exc:
        after_cm("10")
    assert "str" in str(exc.value)


# --------------------------------------------------------------------------- #
# after_forward_cm / after_lateral_cm (axis displacement)                     #
# --------------------------------------------------------------------------- #
def test_after_forward_cm_projection_along_heading():
    r = FakeRobot()
    # heading 0 -> forward axis is +x.
    r.odometry._pose = _Pose(x=0.0, y=0.0, heading=0.0)
    c = after_forward_cm(10)  # 0.1 m
    c.start(r)
    r.odometry._pose = _Pose(x=0.05, y=1.0, heading=0.0)  # y irrelevant for fwd
    assert c.check(r) is False
    r.odometry._pose = _Pose(x=0.1, y=5.0, heading=0.0)
    assert c.check(r) is True


def test_after_forward_cm_heading_90deg():
    r = FakeRobot()
    r.odometry._pose = _Pose(x=0.0, y=0.0, heading=math.pi / 2)
    c = after_forward_cm(10)
    c.start(r)
    # forward axis now +y; moving x does nothing.
    r.odometry._pose = _Pose(x=1.0, y=0.05, heading=math.pi / 2)
    assert c.check(r) is False
    r.odometry._pose = _Pose(x=1.0, y=0.1, heading=math.pi / 2)
    assert c.check(r) is True


def test_after_forward_cm_negative_target():
    r = FakeRobot()
    r.odometry._pose = _Pose(0.0, 0.0, 0.0)
    c = after_forward_cm(-10)  # target -0.1 m
    c.start(r)
    r.odometry._pose = _Pose(-0.05, 0.0, 0.0)
    assert c.check(r) is False  # delta -0.05 > -0.1
    r.odometry._pose = _Pose(-0.1, 0.0, 0.0)
    assert c.check(r) is True  # delta -0.1 <= -0.1


def test_after_forward_cm_target_m_conversion():
    # cm/100 conversion: 100 cm -> exactly 1.0 m (kills cm/101 mutant).
    assert after_forward_cm(100)._target_m == pytest.approx(1.0)


def test_after_forward_cm_nonzero_origin_x_and_y():
    """Origin captured at a nonzero pose; displacement is relative to it.

    Heading 0 -> forward = dx. Origin at (1.0, 2.0). This pins:
    * origin_x from position[0] not position[1] (kills 144),
    * dx = pos.x - origin_x not + (kills the dx sign mutant 155).
    """
    r = FakeRobot()
    r.odometry._pose = _Pose(x=1.0, y=2.0, heading=0.0)
    c = after_forward_cm(10)  # 0.1 m
    c.start(r)
    # Moved +0.05 m forward (x 1.0 -> 1.05): not enough.
    r.odometry._pose = _Pose(x=1.05, y=2.0, heading=0.0)
    assert c.check(r) is False
    # Moved +0.1 m forward (x 1.0 -> 1.1): triggers.
    r.odometry._pose = _Pose(x=1.1, y=2.0, heading=0.0)
    assert c.check(r) is True


def test_after_lateral_cm_nonzero_origin_y():
    """Heading 0 -> lateral = dy. Origin y nonzero pins the dy subtraction
    (kills the dy sign mutant 158)."""
    r = FakeRobot()
    r.odometry._pose = _Pose(x=0.0, y=3.0, heading=0.0)
    c = after_lateral_cm(10)  # 0.1 m
    c.start(r)
    r.odometry._pose = _Pose(x=0.0, y=3.05, heading=0.0)
    assert c.check(r) is False
    r.odometry._pose = _Pose(x=0.0, y=3.1, heading=0.0)
    assert c.check(r) is True


def test_after_lateral_cm_projection_at_heading():
    """Lateral projection is -dx*sin(h) + dy*cos(h).

    At heading 90deg: lateral = -dx*1 + dy*0 = -dx. Drive +x and the lateral
    displacement is negative; with a negative target it triggers. This pins
    both the -dx*sin sign (mutant 170) and the dy*cos multiply-vs-divide
    (mutant 173 would make dy/cos = 0.1/0 -> inf and change the result).
    """
    r = FakeRobot()
    r.odometry._pose = _Pose(x=0.0, y=0.0, heading=math.pi / 2)
    c = after_lateral_cm(-10)  # target -0.1 m
    c.start(r)
    # Move +x by 0.05 -> lateral = -0.05, not yet <= -0.1.
    r.odometry._pose = _Pose(x=0.05, y=0.0, heading=math.pi / 2)
    assert c.check(r) is False
    # Move +x by 0.1 -> lateral = -0.1 <= -0.1 -> True.
    r.odometry._pose = _Pose(x=0.1, y=0.0, heading=math.pi / 2)
    assert c.check(r) is True


def test_after_lateral_cm_dy_cos_multiply():
    """At heading 0, lateral = dy*cos(0) = dy. If cos were a divisor instead
    of a multiplier (mutant 173) the result would still be dy here (cos=1), so
    use a heading where cos != 1: at 60deg, lateral of a pure +y move is
    dy*cos(60deg) = 0.5*dy, half the path. Reaching 0.1 m lateral needs
    dy = 0.2 m, not 0.1 m."""
    r = FakeRobot()
    h = math.radians(60.0)
    r.odometry._pose = _Pose(x=0.0, y=0.0, heading=h)
    c = after_lateral_cm(10)  # 0.1 m lateral
    c.start(r)
    # dy = 0.1 -> lateral = 0.1*cos(60) = 0.05 m, not enough.
    r.odometry._pose = _Pose(x=0.0, y=0.1, heading=h)
    assert c.check(r) is False
    # dy = 0.2 -> lateral = 0.2*cos(60) = 0.1 m -> triggers.
    r.odometry._pose = _Pose(x=0.0, y=0.2, heading=h)
    assert c.check(r) is True


def test_after_lateral_cm_projection():
    r = FakeRobot()
    r.odometry._pose = _Pose(0.0, 0.0, 0.0)
    c = after_lateral_cm(10)  # lateral = -dx*sin + dy*cos; heading0 -> dy
    c.start(r)
    r.odometry._pose = _Pose(5.0, 0.05, 0.0)
    assert c.check(r) is False
    r.odometry._pose = _Pose(5.0, 0.1, 0.0)
    assert c.check(r) is True


def test_axis_check_false_before_start():
    r = FakeRobot()
    # Pose already far past the target along +x; check() must still be False
    # because start() was never called. Kills the `_started = True` init mutant.
    r.odometry._pose = _Pose(x=99.0, y=0.0, heading=0.0)
    assert after_forward_cm(10).check(r) is False


def test_axis_absolute_forward_reads_distance_from_origin():
    r = FakeRobot()
    r.odometry._dist = _Dist(forward=0.05, lateral=0.0)
    c = after_forward_cm(10, absolute=True)
    c.start(r)
    assert c.check(r) is False
    r.odometry._dist = _Dist(forward=0.1, lateral=0.0)
    assert c.check(r) is True


def test_axis_absolute_lateral_reads_distance_from_origin():
    r = FakeRobot()
    r.odometry._dist = _Dist(forward=0.0, lateral=0.2)
    c = after_lateral_cm(10, absolute=True)
    c.start(r)
    assert c.check(r) is True


def test_axis_zero_cm_raises():
    # Rejected value (0) is behavioural; prose is not.
    with pytest.raises(ValueError, match=r"cm must be") as exc:
        after_forward_cm(0)
    assert "0" in str(exc.value)


def test_axis_non_number_cm_raises():
    # Rejected type (str) is behavioural; prose is not.
    with pytest.raises(TypeError, match=r"cm must be") as exc:
        after_forward_cm("5")
    assert "str" in str(exc.value)


def test_axis_bad_heading_type_raises():
    # Rejected type (str) is behavioural; prose is not.
    with pytest.raises(TypeError, match=r"heading must be") as exc:
        after_forward_cm(10, heading="north")
    assert "str" in str(exc.value)


def test_axis_heading_and_absolute_mutually_exclusive():
    # Behavioural: both option names appear; the connecting prose is not pinned.
    with pytest.raises(ValueError, match=r"heading=") as exc:
        after_forward_cm(10, heading=0, absolute=True)
    assert "absolute" in str(exc.value)


def test_axis_base_project_not_implemented():
    base = cond._AxisDisplacementCondition(10)
    with pytest.raises(NotImplementedError):
        base._project(1.0, 2.0)


def test_axis_base_absolute_component_not_implemented():
    base = cond._AxisDisplacementCondition(10, absolute=True)
    with pytest.raises(NotImplementedError):
        base._absolute_component(FakeRobot())


def test_axis_targeted_heading_uses_service():
    """When heading= is given, the frozen reference comes from
    HeadingReferenceService.target_absolute_rad, not the pose heading."""

    class FakeService:
        def target_absolute_rad(self, deg):
            # deg=90 -> pi/2; forward axis becomes +y.
            return math.radians(deg)

    class RobotWithService(FakeRobot):
        def get_service(self, _cls):
            return FakeService()

    r = RobotWithService()
    r.odometry._pose = _Pose(0.0, 0.0, heading=0.0)  # robot crooked at 0
    c = after_forward_cm(10, heading=90)
    c.start(r)
    # forward axis pinned to 90deg (+y), not the pose's 0.
    # Pure +x movement must NOT trigger (axis is +y): if heading= were ignored
    # and the pose heading (0) used, +x would project as forward and fire.
    r.odometry._pose = _Pose(x=5.0, y=0.0, heading=0.0)
    assert c.check(r) is False
    # Movement along +y reaches the target.
    r.odometry._pose = _Pose(x=5.0, y=0.1, heading=0.0)
    assert c.check(r) is True


# --------------------------------------------------------------------------- #
# after_degrees                                                                #
# --------------------------------------------------------------------------- #
def test_after_degrees_basic_rotation():
    r = FakeRobot()
    r.odometry._pose = _Pose(heading=0.0)
    c = after_degrees(90)
    c.start(r)
    r.odometry._pose = _Pose(heading=math.radians(89))
    assert c.check(r) is False
    r.odometry._pose = _Pose(heading=math.radians(90))
    assert c.check(r) is True


def test_after_degrees_wrap_shortest_arc():
    r = FakeRobot()
    # start near +pi, current near -pi: raw delta ~2pi but wrapped is small.
    r.odometry._pose = _Pose(heading=math.pi - 0.01)
    c = after_degrees(90)
    c.start(r)
    r.odometry._pose = _Pose(heading=-math.pi + 0.01)
    # raw delta = ~2pi-0.02 > pi -> wrapped delta = ~0.02 rad, far below 90deg.
    assert c.check(r) is False


def test_after_degrees_wrap_uses_two_pi_minus_delta():
    """A raw delta just over pi wraps to a value still above the target.

    raw delta = pi + 0.1 (> pi) -> wrapped = 2*pi - (pi+0.1) = pi - 0.1 rad
    (~174deg), which is >= the 90deg target -> True. If the wrap formula were
    ``2 / math.pi - delta`` (mutant) the wrapped value would be strongly
    negative and the check would return False.
    """
    r = FakeRobot()
    r.odometry._pose = _Pose(heading=0.0)
    c = after_degrees(90)
    c.start(r)
    r.odometry._pose = _Pose(heading=math.pi + 0.1)  # raw delta > pi
    assert c.check(r) is True


def test_after_degrees_subtracts_start_heading():
    """Delta is |current - start|. Start at a nonzero heading so the sign of
    the subtraction matters (kills current + start mutant)."""
    r = FakeRobot()
    r.odometry._pose = _Pose(heading=math.radians(30.0))
    c = after_degrees(45)
    c.start(r)
    # current 30 -> 70deg: delta = |70 - 30| = 40deg < 45 -> False.
    # If it were current + start = 100deg -> >= 45 -> True (wrong).
    r.odometry._pose = _Pose(heading=math.radians(70.0))
    assert c.check(r) is False
    # current -> 76deg: delta = 46deg >= 45 -> True.
    r.odometry._pose = _Pose(heading=math.radians(76.0))
    assert c.check(r) is True


def test_after_degrees_check_before_start_uses_zero_baseline():
    """Init baseline is 0.0; checking before start() measures from 0.

    With current heading 0 the delta is 0 -> below target -> False. A mutated
    init baseline (1.0 rad, or None) would change/raise this.
    """
    r = FakeRobot()
    r.odometry._pose = _Pose(heading=0.0)
    c = after_degrees(45)
    assert c.check(r) is False


def test_after_degrees_one_degree_valid():
    # degrees == 1 is allowed (> 0). Kills the `degrees <= 1` boundary mutant.
    c = after_degrees(1)
    assert c._target_rad == pytest.approx(math.radians(1))


def test_after_degrees_uses_abs_of_target():
    c = after_degrees(45)
    assert c._target_rad == pytest.approx(math.radians(45))


def test_after_degrees_zero_raises():
    # Rejected value (0) is behavioural; prose is not.
    with pytest.raises(ValueError, match=r"degrees must be") as exc:
        after_degrees(0)
    assert "0" in str(exc.value)


def test_after_degrees_negative_raises():
    with pytest.raises(ValueError, match="degrees must be > 0"):
        after_degrees(-30)


def test_after_degrees_non_number_raises():
    # Rejected type (str) is behavioural; prose is not.
    with pytest.raises(TypeError, match=r"degrees must be") as exc:
        after_degrees("90")
    assert "str" in str(exc.value)


def test_after_degrees_missing_odometry_raises():
    class NoOdom:
        odometry = None

    c = after_degrees(90)
    # Behavioural: names the missing dependency (odometry); prose is not pinned.
    with pytest.raises(RuntimeError, match=r"odometry"):
        c.start(NoOdom())


# --------------------------------------------------------------------------- #
# after_degrees — accumulating total-rotation regression (the bug fix)         #
#                                                                              #
# Before the fix, after_degrees computed |current - start| wrapped to the      #
# SHORTEST arc, which saturates at 180deg — so after_degrees(>180) could       #
# NEVER fire. The fix accumulates the per-tick heading step (each wrapped to   #
# (-pi, pi], abs()'d) across check() calls, so targets > 180deg work and the   #
# total is direction-agnostic / wrap-safe / unsigned.                          #
# --------------------------------------------------------------------------- #
def _drive_heading(c, r, headings):
    """Feed a sequence of absolute headings (degrees) via repeated check().

    Returns the list of check() results, one per heading.
    """
    results = []
    for deg in headings:
        r.odometry._pose = _Pose(heading=math.radians(deg))
        results.append(c.check(r))
    return results


class TestAfterDegreesAccumulates:
    """Regression coverage for the shortest-arc -> accumulating-rotation fix."""

    def test_270_turn_in_10deg_steps_fires_only_at_target(self):
        """A 270deg turn fed as +10deg increments must NOT fire below 270deg
        and MUST fire at/after 270deg.

        This is the core bug regression. Under the OLD shortest-arc code the
        measured delta was |current - start| wrapped to (-pi, pi], which can
        never exceed 180deg, so after_degrees(270) NEVER fired -> every result
        would be False and the at-270 assertion below would fail.
        """
        r = FakeRobot()
        r.odometry._pose = _Pose(heading=0.0)
        c = after_degrees(270)
        c.start(r)

        # 0 -> 260 in +10deg steps: 26 steps, accumulated tops out at 260deg.
        below = _drive_heading(c, r, list(range(10, 261, 10)))
        assert below == [False] * len(below)
        assert c._accumulated_rad == pytest.approx(math.radians(260))

        # One more step carries past 270deg (271) -> fires. (Stepping to
        # exactly 270 lands a hair under the target due to float summation of
        # 27 increments; a real turn always overshoots the goal slightly.)
        r.odometry._pose = _Pose(heading=math.radians(271))
        assert c.check(r) is True
        assert c._accumulated_rad == pytest.approx(math.radians(271))

    def test_just_below_270_stays_false(self):
        """At 269deg accumulated (one degree short) the 270 target is False."""
        r = FakeRobot()
        r.odometry._pose = _Pose(heading=0.0)
        c = after_degrees(270)
        c.start(r)
        # Step straight to 269deg in one tick: |269| wrapped = -91deg? No —
        # 269deg wraps to 269-360 = -91deg, abs = 91deg. So feed it in chunks
        # that stay within a half-turn each to accumulate a clean 269deg.
        results = _drive_heading(c, r, [90, 180, 269])
        assert results == [False, False, False]
        assert c._accumulated_rad == pytest.approx(math.radians(269))

    def test_negative_direction_accumulates_same(self):
        """A turn in the negative direction accumulates identically; 200deg of
        negative rotation fires after_degrees(200)."""
        r = FakeRobot()
        r.odometry._pose = _Pose(heading=0.0)
        c = after_degrees(200)
        c.start(r)
        # 0 -> -190 in -10deg steps: never reaches 200deg.
        below = _drive_heading(c, r, [-d for d in range(10, 191, 10)])
        assert below == [False] * len(below)
        assert c._accumulated_rad == pytest.approx(math.radians(190))
        # One more step carries past 200deg (-201) -> fires. (float summation
        # of the increments lands a hair under at exactly -200.)
        r.odometry._pose = _Pose(heading=math.radians(-201))
        assert c.check(r) is True
        assert c._accumulated_rad == pytest.approx(math.radians(201))

    def test_wraparound_does_not_inflate_accumulation(self):
        """Crossing the +pi/-pi discontinuity as one physical +20deg step must
        accumulate +20deg, not 340deg.

        Heading goes 170deg -> -170deg. The raw difference is -340deg, but the
        physical rotation is +20deg. The per-tick wrap to (-pi, pi] keeps the
        accumulator honest.
        """
        r = FakeRobot()
        r.odometry._pose = _Pose(heading=math.radians(170))
        c = after_degrees(90)  # target well above the true 20deg step
        c.start(r)
        # 170 -> -170 is a +20deg physical step across the wrap.
        r.odometry._pose = _Pose(heading=math.radians(-170))
        assert c.check(r) is False
        assert c._accumulated_rad == pytest.approx(math.radians(20))
        # Another +20deg (-170 -> -150) -> 40deg total, still below 90.
        r.odometry._pose = _Pose(heading=math.radians(-150))
        assert c.check(r) is False
        assert c._accumulated_rad == pytest.approx(math.radians(40))

    def test_back_and_forth_accumulates_unsigned_total(self):
        """+100deg then -100deg = 200deg of total (unsigned) rotation, even
        though net displacement is 0. after_degrees(150) fires."""
        r = FakeRobot()
        r.odometry._pose = _Pose(heading=0.0)
        c = after_degrees(150)
        c.start(r)
        # Swing out to +100deg (below 150).
        r.odometry._pose = _Pose(heading=math.radians(100))
        assert c.check(r) is False
        assert c._accumulated_rad == pytest.approx(math.radians(100))
        # Swing back to 0deg: that is another 100deg of rotation -> 200 total.
        # Net displacement is 0, but total unsigned rotation is 200deg >= 150.
        r.odometry._pose = _Pose(heading=0.0)
        assert c.check(r) is True
        assert c._accumulated_rad == pytest.approx(math.radians(200))

    def test_small_single_step_still_works(self):
        """The simple 90deg single-step case still behaves correctly."""
        r = FakeRobot()
        r.odometry._pose = _Pose(heading=0.0)
        c = after_degrees(90)
        c.start(r)
        r.odometry._pose = _Pose(heading=math.radians(89.5))
        assert c.check(r) is False
        r.odometry._pose = _Pose(heading=math.radians(90))
        assert c.check(r) is True

    def test_start_resets_accumulator_and_last_heading(self):
        """start() zeroes the accumulator and re-anchors _last_heading, so a
        reused condition does not carry stale rotation across runs."""
        r = FakeRobot()
        r.odometry._pose = _Pose(heading=0.0)
        c = after_degrees(90)
        c.start(r)
        _drive_heading(c, r, [40, 80])  # accumulate 80deg
        assert c._accumulated_rad == pytest.approx(math.radians(80))
        # Re-start from a fresh heading: accumulator resets, anchor moves.
        r.odometry._pose = _Pose(heading=math.radians(200))
        c.start(r)
        assert c._accumulated_rad == 0.0
        assert c._last_heading == pytest.approx(math.radians(200))
        # A small step from the new anchor must not include the old 80deg.
        r.odometry._pose = _Pose(heading=math.radians(210))
        assert c.check(r) is False
        assert c._accumulated_rad == pytest.approx(math.radians(10))


# --------------------------------------------------------------------------- #
# on_digital                                                                   #
# --------------------------------------------------------------------------- #
def test_on_digital_pressed_default():
    s = FakeDigital(value=True)
    assert on_digital(s).check(ROBOT) is True
    s.value = False
    assert on_digital(s).check(ROBOT) is False


def test_on_digital_released_mode():
    s = FakeDigital(value=False)
    assert on_digital(s, pressed=False).check(ROBOT) is True
    s.value = True
    assert on_digital(s, pressed=False).check(ROBOT) is False


def test_on_digital_rejects_non_sensor():
    with pytest.raises(TypeError, match=r"read\(\)") as exc:
        on_digital(object())
    assert "object" in str(exc.value)


# --------------------------------------------------------------------------- #
# on_analog_above / below                                                      #
# --------------------------------------------------------------------------- #
def test_on_analog_above_strict():
    s = FakeAnalog(value=100)
    assert on_analog_above(s, 99).check(ROBOT) is True
    assert on_analog_above(s, 100).check(ROBOT) is False  # strict >
    assert on_analog_above(s, 101).check(ROBOT) is False


def test_on_analog_below_strict():
    s = FakeAnalog(value=100)
    assert on_analog_below(s, 101).check(ROBOT) is True
    assert on_analog_below(s, 100).check(ROBOT) is False  # strict <


def test_on_analog_above_rejects_non_sensor():
    with pytest.raises(TypeError, match=r"read\(\)") as exc:
        on_analog_above(object(), 5)
    assert "object" in str(exc.value)


def test_on_analog_above_rejects_non_number_threshold():
    with pytest.raises(TypeError, match=r"threshold must be") as exc:
        on_analog_above(FakeAnalog(), "5")
    assert "str" in str(exc.value)


def test_on_analog_below_rejects_non_sensor():
    with pytest.raises(TypeError, match=r"read\(\)") as exc:
        on_analog_below(object(), 5)
    assert "object" in str(exc.value)


def test_on_analog_below_rejects_non_number_threshold():
    with pytest.raises(TypeError, match=r"threshold must be") as exc:
        on_analog_below(FakeAnalog(), "5")
    assert "str" in str(exc.value)


# --------------------------------------------------------------------------- #
# stall_detected                                                               #
# --------------------------------------------------------------------------- #
def test_stall_detected_fires_after_duration(monkeypatch):
    clock = {"t": 0.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    m = FakeMotor(position=0)
    c = stall_detected(m, threshold_tps=10, duration=0.25)
    c.start(r := ROBOT)
    # dt too small -> ignored.
    clock["t"] = 0.01
    assert c.check(r) is False
    # dt >= 0.02, no movement -> stall_start set, but duration not met.
    clock["t"] = 0.05
    assert c.check(r) is False
    assert c._stall_start == pytest.approx(0.05)
    # still stalled but duration NOT yet met -> stall_start kept, returns False.
    clock["t"] = 0.2
    assert c.check(r) is False
    assert c._stall_start == pytest.approx(0.05)
    # still stalled past duration -> True.
    clock["t"] = 0.31
    assert c.check(r) is True


def test_stall_detected_resets_when_moving(monkeypatch):
    clock = {"t": 0.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    m = FakeMotor(position=0)
    c = stall_detected(m, threshold_tps=10, duration=0.25)
    c.start(ROBOT)
    clock["t"] = 0.1
    m.position = 0  # speed 0 < 10 -> stall_start set
    assert c.check(ROBOT) is False
    assert c._stall_start is not None
    # now it moves fast -> resets stall_start.
    clock["t"] = 0.2
    m.position = 1000  # speed huge
    assert c.check(ROBOT) is False
    assert c._stall_start is None


def test_stall_detected_dt_boundary_exactly_002(monkeypatch):
    """dt == 0.02 is NOT ignored (guard is dt < 0.02, strict).

    At exactly 0.02 the check proceeds and, with no movement, records a stall
    start. The `dt <= 0.02` mutant would early-return and never set it.
    """
    clock = {"t": 0.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    m = FakeMotor(position=0)
    c = stall_detected(m, threshold_tps=10, duration=0.25)
    c.start(ROBOT)
    clock["t"] = 0.02  # dt exactly 0.02
    assert c.check(ROBOT) is False
    assert c._stall_start == pytest.approx(0.02)  # proved it proceeded


def test_stall_detected_speed_uses_difference_over_dt(monkeypatch):
    """speed = |pos - last_position| / dt.

    Start at a nonzero position so the subtraction sign matters (kills the
    `pos + last_position` mutant), and choose numbers where dividing vs
    multiplying by dt flips the stall decision (kills the `* dt` mutant).

    Setup: last_position = 100, dt = 0.1 s. Move to pos = 102 -> moved 2 ticks.
    Real speed = |102-100|/0.1 = 20 tps >= threshold(10) -> NOT stalled.
    `pos + last` mutant: |102+100|/0.1 = 2020 tps -> also not stalled (same
    branch) — so we separately check a no-stall vs stall flip below.
    `* dt` mutant: |102-100|*0.1 = 0.2 tps < 10 -> stalled (wrong branch).
    """
    clock = {"t": 0.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    m = FakeMotor(position=100)
    c = stall_detected(m, threshold_tps=10, duration=0.25)
    c.start(ROBOT)  # last_position = 100, last_time = 0
    clock["t"] = 0.1
    m.position = 102  # moved 2 ticks in 0.1 s -> 20 tps, above threshold
    assert c.check(ROBOT) is False
    # Not stalled: speed (20) >= threshold (10), so stall_start stays None.
    assert c._stall_start is None


def test_stall_detected_pos_subtraction_sign(monkeypatch):
    """A small real movement keeps speed below threshold (stalled), but the
    `pos + last_position` mutant would compute a huge speed and reset.

    last_position = 1000, move to 1001 in 0.1 s -> real speed = 10... use
    threshold 20 so |1001-1000|/0.1 = 10 < 20 -> stalled. The `+` mutant gives
    |1001+1000|/0.1 = 20010 >= 20 -> not stalled (stall_start reset to None).
    """
    clock = {"t": 0.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    m = FakeMotor(position=1000)
    c = stall_detected(m, threshold_tps=20, duration=0.25)
    c.start(ROBOT)
    clock["t"] = 0.1
    m.position = 1001  # real speed 10 tps < 20 -> stalled
    assert c.check(ROBOT) is False
    assert c._stall_start == pytest.approx(0.1)  # stall recorded


def test_stall_detected_speed_threshold_is_strict(monkeypatch):
    """speed exactly == threshold is NOT a stall (guard is speed < threshold).

    Move 1 tick in 0.1 s with threshold 10 -> speed = 10 == threshold.
    Real: 10 < 10 is False -> not stalled (stall_start stays None).
    `speed <= threshold` mutant: would treat it as a stall.
    """
    clock = {"t": 0.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    m = FakeMotor(position=0)
    c = stall_detected(m, threshold_tps=10, duration=0.25)
    c.start(ROBOT)
    clock["t"] = 0.1
    m.position = 1  # speed = 1/0.1 = 10 tps == threshold
    assert c.check(ROBOT) is False
    assert c._stall_start is None


def test_stall_detected_duration_boundary_inclusive(monkeypatch):
    """Elapsed stall time exactly == duration triggers (guard is >=).

    Real: now - stall_start == duration -> >= -> True.
    `> duration` mutant: would return False at the exact boundary.
    """
    clock = {"t": 0.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    m = FakeMotor(position=0)
    c = stall_detected(m, threshold_tps=10, duration=0.25)
    c.start(ROBOT)
    clock["t"] = 0.05
    assert c.check(ROBOT) is False  # stall_start = 0.05
    assert c._stall_start == pytest.approx(0.05)
    clock["t"] = 0.30  # elapsed exactly 0.25 == duration (float-exact)
    assert c.check(ROBOT) is True


def test_stall_detected_last_time_updates_between_checks(monkeypatch):
    """dt is measured from the *previous* check, not start.

    After a first check at t=0.1 (last_time -> 0.1), a second check at t=0.3
    has dt = 0.3 - 0.1 = 0.2. The `now + last_time` mutant would compute
    0.3 + 0.1 = 0.4; we exploit this with a movement that makes the speed
    decision flip on the divisor.

    Move 4 ticks between the two checks. Real speed = 4 / 0.2 = 20 tps.
    `now + last_time` mutant divisor = 0.4 -> 4 / 0.4 = 10 tps. With threshold
    15: real 20 >= 15 (not stalled), mutant 10 < 15 (stalled). Assert not
    stalled.
    """
    clock = {"t": 0.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    m = FakeMotor(position=0)
    c = stall_detected(m, threshold_tps=15, duration=0.25)
    c.start(ROBOT)  # last_time = 0
    clock["t"] = 0.1
    m.position = 1000  # big move -> not stalled, last_time -> 0.1
    assert c.check(ROBOT) is False
    assert c._stall_start is None
    clock["t"] = 0.3
    m.position = 1004  # moved 4 ticks since last check (dt = 0.2)
    assert c.check(ROBOT) is False
    assert c._stall_start is None  # speed 20 tps >= 15 -> not stalled


def test_stall_detected_default_threshold_is_10(monkeypatch):
    """Default threshold_tps is 10 (not 11): a speed of 10.5 tps is NOT a
    stall under the default."""
    clock = {"t": 0.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    m = FakeMotor(position=0)
    c = stall_detected(m)  # defaults: threshold_tps=10, duration=0.25
    assert c._threshold_tps == 10
    c.start(ROBOT)
    clock["t"] = 0.1
    m.position = 105  # 105 ticks / 0.1 s = 1050 tps... too big; use small move
    # Reset: choose a move giving 10.5 tps: 1.05 ticks not integer; instead
    # verify the stored default directly above and a speed-just-above-10 case.
    m.position = 0
    c2 = stall_detected(m)
    c2.start(ROBOT)
    clock["t"] = 0.2
    m.position = 2  # 2 ticks / 0.1 s (dt from 0.1->0.2) = 20 tps >= 10
    assert c2.check(ROBOT) is False
    assert c2._stall_start is None  # not stalled at default threshold 10


def test_stall_detected_default_duration_is_025(monkeypatch):
    """Default duration is 0.25 s (not 1.25): a stall lasting 0.3 s fires."""
    clock = {"t": 0.0}
    monkeypatch.setattr(cond.time, "monotonic", lambda: clock["t"])
    m = FakeMotor(position=0)
    c = stall_detected(m)  # default duration 0.25
    assert c._duration == pytest.approx(0.25)
    c.start(ROBOT)
    clock["t"] = 0.1
    assert c.check(ROBOT) is False  # stall_start = 0.1
    clock["t"] = 0.41  # elapsed 0.31 s >= 0.25 default (but < 1.25 mutant)
    assert c.check(ROBOT) is True


def test_stall_detected_threshold_one_is_valid():
    # threshold_tps == 1 is allowed (> 0). Kills the `threshold_tps <= 1` mutant.
    c = stall_detected(FakeMotor(), threshold_tps=1)
    assert c._threshold_tps == 1


def test_stall_detected_init_state_before_start(monkeypatch):
    """Verify the constructor's initial tracking state via a check() before
    start(): _last_position=0, _last_time=0.0, _stall_start=None.

    With the clock at 0.1, dt = 0.1 - 0 = 0.1 (>= 0.02) so the check proceeds;
    a motor at position 1 gives speed = |1 - 0| / 0.1 = 10 tps, above the
    threshold of 5 -> not stalled, and the check advances _last_time to 0.1.

    This pins each init constant:
      * _last_position = 1 (259) -> speed 0 -> stalled (stall_start set)
      * _last_position = None (260) -> abs(1 - None) raises
      * _last_time = 1.0 (261) -> dt = -0.9 < 0.02 -> early return, _last_time
        never advances to 0.1
      * _last_time = None (262) -> 0.1 - None raises
    """
    monkeypatch.setattr(cond.time, "monotonic", lambda: 0.1)
    m = FakeMotor(position=1)
    c = stall_detected(m, threshold_tps=5, duration=0.25)
    # No start(): exercise the constructor's initial state.
    assert c.check(ROBOT) is False
    assert c._stall_start is None  # not stalled (speed 10 >= 5)
    assert c._last_time == pytest.approx(0.1)  # check proceeded and advanced
    assert c._last_position == 1


def test_stall_detected_stall_start_initialised_none(monkeypatch):
    """A stalled first check (before start) sets _stall_start from None.

    With no movement the speed is 0 < threshold, taking the
    `if self._stall_start is None:` branch and recording the stall time. This
    pins the init value of _stall_start to None: a mutated init of "" would
    fail the `is None` test and instead evaluate `now - ""`, raising TypeError.
    """
    monkeypatch.setattr(cond.time, "monotonic", lambda: 0.1)
    m = FakeMotor(position=0)
    c = stall_detected(m, threshold_tps=10, duration=0.25)
    # No start(): _stall_start starts as None; speed 0 < 10 -> stall recorded.
    assert c.check(ROBOT) is False
    assert c._stall_start == pytest.approx(0.1)


def test_stall_detected_bad_motor():
    with pytest.raises(TypeError, match=r"get_position") as exc:
        stall_detected(object())
    assert "object" in str(exc.value)


def test_stall_detected_bad_threshold():
    # Rejected value (0) is behavioural; prose is not.
    with pytest.raises(ValueError, match=r"threshold_tps must be") as exc:
        stall_detected(FakeMotor(), threshold_tps=0)
    assert "0" in str(exc.value)


def test_stall_detected_bad_duration():
    # Rejected value (0) is behavioural; prose is not.
    with pytest.raises(ValueError, match=r"duration must be") as exc:
        stall_detected(FakeMotor(), duration=0)
    assert "0" in str(exc.value)


# --------------------------------------------------------------------------- #
# custom                                                                       #
# --------------------------------------------------------------------------- #
def test_custom_delegates_to_callable():
    calls = []

    def fn(robot):
        calls.append(robot)
        return robot == "stop"

    c = custom(fn)
    assert c.check("go") is False
    assert c.check("stop") is True
    assert calls == ["go", "stop"]


def test_custom_rejects_non_callable():
    # Rejected type (int) is behavioural; prose is not.
    with pytest.raises(TypeError, match=r"callable") as exc:
        custom(42)
    assert "int" in str(exc.value)


# --------------------------------------------------------------------------- #
# over_line  (factory composing on_black + on_white)                          #
# --------------------------------------------------------------------------- #
def test_over_line_is_then_of_black_white():
    s = FakeSensor()
    c = over_line(s)
    assert isinstance(c, cond._Then)
    assert isinstance(c._first, on_black)
    assert isinstance(c._second, on_white)


def test_over_line_black_then_white_sequence():
    s = FakeSensor(black=0.0, white=0.0)
    c = over_line(s)
    c.start(ROBOT)
    # neither black nor white yet
    assert c.check(ROBOT) is False
    # sees black -> first fires, second (white) starts but not satisfied
    s.black = 0.9
    assert c.check(ROBOT) is False
    assert c._first_done is True
    # white not yet
    s.black = 0.0
    s.white = 0.0
    assert c.check(ROBOT) is False
    # now white -> stop
    s.white = 0.9
    assert c.check(ROBOT) is True


def test_over_line_custom_thresholds():
    s = FakeSensor()
    c = over_line(s, black_threshold=0.3, white_threshold=0.4)
    assert c._first._threshold == pytest.approx(0.3)
    assert c._second._threshold == pytest.approx(0.4)
