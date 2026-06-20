"""Spec-driven tests for leaf-step CONSTRUCTION in ``raccoon.step.motion.drive``.

Scope: the pure-Python construction/validation logic of the directional drive
steps — ``DriveForward`` / ``DriveBackward`` / ``StrafeLeft`` / ``StrafeRight``
and the shared ``_ConditionalDrive.__init__``. Motion *execution* (``on_start``
builds a real C++ ``LinearMotion`` and pulls services off the robot) is out of
scope and never exercised here.

Every expected value below is derived independently from the docstrings,
parameter names and units, NOT by running the code:

* distance: centimetres -> metres is ``cm / 100.0`` (so 25 cm -> 0.25 m).
* the per-direction ``_sign`` multiplies the distance; from the class
  semantics (drive forward vs backward, strafe left vs right) the conventional
  signs are Forward=+1, Backward=-1, StrafeLeft=-1, StrafeRight=+1. These are
  cross-checked against the axis each step uses (Forward axis vs Lateral axis).
* speed must lie in the half-open interval ``(0.0, 1.0]``.
* when no distance is given a large sentinel distance (100 m, per the docstring)
  is used, signed by ``_sign``.

These pure quantities surface through two construction-time paths that need no
hardware:

* the stored attributes set in ``__init__`` (``_cm``, ``_sign``, ``_axis``,
  ``_speed``, ``_until``, ``_heading_deg``);
* ``lower_to_segments()``, which recomputes the signed metre distance / sentinel
  and is therefore the load-bearing assertion of the cm->m + sign contract.
"""

from __future__ import annotations

import importlib
import math

import pytest

# conftest pre-imports raccoon.step.motion (and thus drive) before pytest-cov
# attaches its tracer, so the module's import-time class/def/docstring lines are
# never counted. Reload under coverage so those definition statements
# re-execute while traced, then bind every name from the reloaded module so
# identities stay consistent within this file.
import raccoon.step.motion.drive as drive
from raccoon.motion import LinearAxis
from raccoon.step.condition import StopCondition

drive = importlib.reload(drive)

DriveBackward = drive.DriveBackward
DriveForward = drive.DriveForward
StrafeLeft = drive.StrafeLeft
StrafeRight = drive.StrafeRight
_ConditionalDrive = drive._ConditionalDrive
_drive_forward_uncalibrated = drive._drive_forward_uncalibrated

# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------


class FakeCondition(StopCondition):
    """Records start()/check() calls; never actually stops."""

    def __init__(self) -> None:
        self.started = False
        self.checks = 0

    def start(self, robot) -> None:
        self.started = True

    def check(self, robot) -> bool:
        self.checks += 1
        return False


# The four concrete directional steps and their *independently derived* expected
# (axis, sign). Forward axis: forward = +1, backward = -1. Lateral axis: left is
# the negative lateral direction, right the positive -> left = -1, right = +1.
ALL_STEPS = [
    (DriveForward, LinearAxis.Forward, 1.0),
    (DriveBackward, LinearAxis.Forward, -1.0),
    (StrafeLeft, LinearAxis.Lateral, -1.0),
    (StrafeRight, LinearAxis.Lateral, 1.0),
]


def _seg(step):
    """Return the single Segment a directional step lowers to."""
    segs = step.lower_to_segments()
    assert len(segs) == 1
    return segs[0]


# ---------------------------------------------------------------------------
# Direction convention: axis + sign per concrete step
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(("cls", "axis", "sign"), ALL_STEPS)
def test_class_axis_and_sign_constants(cls, axis, sign):
    # Convention is carried as class attributes; assert against hand-derived
    # values, not against whatever the source happens to contain.
    assert cls._axis == axis
    assert cls._sign == sign


@pytest.mark.parametrize(("cls", "axis", "sign"), ALL_STEPS)
def test_axis_and_sign_propagate_to_instance(cls, axis, sign):
    step = cls(cm=10)
    assert step._axis == axis
    assert step._sign == sign


def test_forward_and_backward_signs_are_opposite():
    assert DriveForward._sign == -DriveBackward._sign


def test_strafe_left_and_right_signs_are_opposite():
    assert StrafeLeft._sign == -StrafeRight._sign


def test_forward_axis_steps_use_forward_axis_only():
    assert DriveForward._axis == LinearAxis.Forward
    assert DriveBackward._axis == LinearAxis.Forward


def test_strafe_steps_use_lateral_axis_only():
    assert StrafeLeft._axis == LinearAxis.Lateral
    assert StrafeRight._axis == LinearAxis.Lateral


# ---------------------------------------------------------------------------
# cm -> m conversion + sign injection (via lower_to_segments)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(("cls", "axis", "sign"), ALL_STEPS)
def test_distance_cm_to_m_with_sign(cls, axis, sign):
    # 25 cm -> 0.25 m, multiplied by the direction sign. Derived by hand.
    step = cls(cm=25)
    seg = _seg(step)
    assert seg.distance_m == pytest.approx(sign * 0.25)
    assert seg.axis == axis
    assert seg.sign == sign
    assert seg.has_known_endpoint is True
    assert seg.kind == "linear"


@pytest.mark.parametrize("cm", [1, 50, 100, 250, 7.5, 0.5])
def test_forward_distance_conversion_arithmetic(cm):
    # Forward sign is +1; expected metres is exactly cm/100.
    step = DriveForward(cm=cm)
    seg = _seg(step)
    assert seg.distance_m == pytest.approx(cm / 100.0)


def test_backward_distance_is_negative():
    seg = _seg(DriveBackward(cm=40))
    # 40 cm -> 0.40 m, reversed -> -0.40 m.
    assert seg.distance_m == pytest.approx(-0.40)


def test_strafe_left_distance_is_negative():
    seg = _seg(StrafeLeft(cm=30))
    assert seg.distance_m == pytest.approx(-0.30)


def test_strafe_right_distance_is_positive():
    seg = _seg(StrafeRight(cm=30))
    assert seg.distance_m == pytest.approx(0.30)


def test_distance_one_cm_is_one_hundredth_metre():
    # Smallest meaningful positive distance: exactly 1 cm == 0.01 m.
    seg = _seg(DriveForward(cm=1))
    assert seg.distance_m == pytest.approx(0.01)


def test_large_distance_conversion():
    seg = _seg(DriveForward(cm=10_000))
    assert seg.distance_m == pytest.approx(100.0)


# ---------------------------------------------------------------------------
# Sentinel-distance injection (no cm, only until)
# ---------------------------------------------------------------------------


def test_sentinel_distance_value_is_100m():
    # Documented sentinel is 100 m.
    assert _ConditionalDrive._SENTINEL_DISTANCE_M == 100.0


@pytest.mark.parametrize(("cls", "axis", "sign"), ALL_STEPS)
def test_until_only_has_no_known_endpoint_segment(cls, axis, sign):
    cond = FakeCondition()
    step = cls(until=cond)
    seg = _seg(step)
    # No distance target: segment carries None distance and is open-ended.
    assert seg.distance_m is None
    assert seg.has_known_endpoint is False
    assert seg.condition is cond
    assert seg.axis == axis
    assert seg.sign == sign


def test_until_only_stores_no_cm():
    step = DriveForward(until=FakeCondition())
    assert step._cm is None


# ---------------------------------------------------------------------------
# Stored attribute wiring
# ---------------------------------------------------------------------------


def test_defaults_speed_until_heading():
    step = DriveForward(cm=10)
    assert step._speed == 1.0
    assert step._until is None
    assert step._heading_deg is None
    assert step._cm == 10
    assert step._skip_calibration is False
    assert step._motion is None


def test_speed_is_stored_verbatim():
    step = DriveForward(cm=10, speed=0.6)
    assert step._speed == 0.6


def test_heading_is_stored_in_degrees_unconverted():
    # heading is documented in DEGREES from the heading reference; the value is
    # stored verbatim, no deg->rad conversion at construction time.
    step = DriveForward(cm=10, heading=90)
    assert step._heading_deg == 90


def test_until_condition_is_stored():
    cond = FakeCondition()
    step = DriveForward(cm=10, until=cond)
    assert step._until is cond
    # Construction must NOT start the condition.
    assert cond.started is False


def test_heading_propagates_to_segment():
    seg = _seg(DriveForward(cm=10, heading=45))
    assert seg.heading_deg == 45


def test_speed_propagates_to_segment():
    seg = _seg(DriveForward(cm=10, speed=0.3))
    assert seg.speed_scale == pytest.approx(0.3)


# ---------------------------------------------------------------------------
# Required-argument guard: need cm OR until
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("cls", [DriveForward, DriveBackward, StrafeLeft, StrafeRight])
def test_requires_cm_or_until(cls):
    with pytest.raises(ValueError, match=r"cm|until"):
        cls()


def test_cm_only_is_allowed():
    # No exception expected.
    DriveForward(cm=10)


def test_until_only_is_allowed():
    DriveForward(until=FakeCondition())


def test_both_cm_and_until_is_allowed():
    cond = FakeCondition()
    step = DriveForward(cm=10, until=cond)
    assert step._cm == 10
    assert step._until is cond


# ---------------------------------------------------------------------------
# cm validation: type + positivity
# ---------------------------------------------------------------------------


def test_cm_must_be_number_not_string():
    with pytest.raises(TypeError, match=r"cm"):
        DriveForward(cm="25")


def test_cm_must_be_number_not_list():
    with pytest.raises(TypeError, match=r"cm"):
        DriveForward(cm=[25])


def test_cm_zero_is_rejected():
    # Boundary: 0 is NOT > 0, so it must be a ValueError (not TypeError).
    with pytest.raises(ValueError, match=r"cm"):
        DriveForward(cm=0)


def test_cm_negative_is_rejected():
    with pytest.raises(ValueError, match=r"cm"):
        DriveForward(cm=-5)


def test_cm_smallest_positive_is_accepted():
    # Just above the boundary.
    step = DriveForward(cm=0.0001)
    assert step._cm == 0.0001


def test_cm_bool_true_is_accepted_as_number():
    # bool is a subclass of int; True == 1 > 0. Documents the (perhaps
    # surprising) consequence of the int|float isinstance check.
    step = DriveForward(cm=True)
    assert step._cm is True
    # True -> 1 cm -> 0.01 m through the conversion.
    assert _seg(step).distance_m == pytest.approx(0.01)


def test_cm_bool_false_is_rejected_as_nonpositive():
    # bool False == 0, which fails the > 0 check -> ValueError.
    with pytest.raises(ValueError, match=r"cm"):
        DriveForward(cm=False)


# ---------------------------------------------------------------------------
# speed validation: (0.0, 1.0]
# ---------------------------------------------------------------------------


def test_speed_upper_bound_one_is_accepted():
    # 1.0 is the inclusive upper bound.
    step = DriveForward(cm=10, speed=1.0)
    assert step._speed == 1.0


def test_speed_just_above_one_is_rejected():
    with pytest.raises(ValueError, match=r"speed"):
        DriveForward(cm=10, speed=1.0001)


def test_speed_zero_is_rejected():
    # 0.0 is excluded (open lower bound).
    with pytest.raises(ValueError, match=r"speed"):
        DriveForward(cm=10, speed=0.0)


def test_speed_negative_is_rejected():
    with pytest.raises(ValueError, match=r"speed"):
        DriveForward(cm=10, speed=-0.5)


def test_speed_smallest_positive_is_accepted():
    step = DriveForward(cm=10, speed=1e-6)
    assert step._speed == 1e-6


def test_speed_large_is_rejected():
    with pytest.raises(ValueError, match=r"speed"):
        DriveForward(cm=10, speed=5.0)


def test_speed_must_be_number_not_string():
    with pytest.raises(TypeError, match=r"speed"):
        DriveForward(cm=10, speed="fast")


def test_speed_type_check_precedes_range_check():
    # A non-number speed must raise TypeError, never the ValueError range msg.
    with pytest.raises(TypeError):
        DriveForward(cm=10, speed=None)


# ---------------------------------------------------------------------------
# until validation: must be a StopCondition
# ---------------------------------------------------------------------------


def test_until_wrong_type_rejected():
    with pytest.raises(TypeError, match=r"until"):
        DriveForward(cm=10, until="stop")


def test_until_object_not_stopcondition_rejected():
    with pytest.raises(TypeError, match=r"until"):
        DriveForward(cm=10, until=object())


def test_until_subclass_instance_accepted():
    cond = FakeCondition()  # subclass of StopCondition
    step = DriveForward(cm=10, until=cond)
    assert step._until is cond


# ---------------------------------------------------------------------------
# heading validation: must be a number (degrees)
# ---------------------------------------------------------------------------


def test_heading_must_be_number():
    with pytest.raises(TypeError, match=r"heading"):
        DriveForward(cm=10, heading="north")


def test_heading_zero_is_accepted():
    # 0 deg is a legitimate absolute heading, distinct from None.
    step = DriveForward(cm=10, heading=0)
    assert step._heading_deg == 0


def test_heading_negative_is_accepted():
    step = DriveForward(cm=10, heading=-90)
    assert step._heading_deg == -90


def test_heading_180_and_360_accepted():
    assert DriveForward(cm=10, heading=180)._heading_deg == 180
    assert DriveForward(cm=10, heading=360)._heading_deg == 360


def test_heading_none_keeps_relative_mode():
    step = DriveForward(cm=10)
    assert step._heading_deg is None


# ---------------------------------------------------------------------------
# Validation ordering: the required-arg guard runs before per-arg checks
# ---------------------------------------------------------------------------


def test_missing_args_takes_priority_over_bad_speed():
    # cm None + until None triggers the "requires cm or until" ValueError before
    # the speed range is ever evaluated.
    with pytest.raises(ValueError, match=r"cm|until"):
        DriveForward(speed=5.0)


# ---------------------------------------------------------------------------
# Signature is a plain string (no prose pinning — just type + key tokens)
# ---------------------------------------------------------------------------


def test_signature_distance_mode_is_string_mentioning_cm():
    sig = DriveForward(cm=25)._generate_signature()
    assert isinstance(sig, str)
    assert "cm" in sig


def test_signature_until_mode_is_string():
    sig = DriveForward(until=FakeCondition())._generate_signature()
    assert isinstance(sig, str)
    assert "until" in sig


def test_signature_includes_class_name():
    # The four classes must be distinguishable in logs.
    assert "DriveForward" in DriveForward(cm=10)._generate_signature()
    assert "StrafeRight" in StrafeRight(cm=10)._generate_signature()


# ---------------------------------------------------------------------------
# Uncalibrated helper factory
# ---------------------------------------------------------------------------


def test_uncalibrated_helper_returns_driveforward_skipping_calibration():
    step = _drive_forward_uncalibrated(15)
    assert isinstance(step, DriveForward)
    assert step._skip_calibration is True
    assert step._cm == 15
    assert step._speed == 1.0


def test_uncalibrated_helper_passes_speed():
    step = _drive_forward_uncalibrated(15, speed=0.4)
    assert step._speed == 0.4


def test_normal_construction_does_not_skip_calibration():
    assert DriveForward(cm=10)._skip_calibration is False


# ---------------------------------------------------------------------------
# Cross-step distinctness sanity (signs genuinely differ)
# ---------------------------------------------------------------------------


def test_all_four_steps_produce_distinct_signed_distances():
    d = {cls.__name__: _seg(cls(cm=20)).distance_m for cls, _, _ in ALL_STEPS}
    assert d["DriveForward"] == pytest.approx(0.20)
    assert d["DriveBackward"] == pytest.approx(-0.20)
    assert d["StrafeLeft"] == pytest.approx(-0.20)
    assert d["StrafeRight"] == pytest.approx(0.20)
    # Forward != Backward, Left != Right.
    assert not math.isclose(d["DriveForward"], d["DriveBackward"])
    assert not math.isclose(d["StrafeLeft"], d["StrafeRight"])


# ===========================================================================
# on_start / on_update execution — the C++ LinearMotion is monkeypatched and
# the lazily-imported calibration / heading services are replaced with doubles,
# so we can assert the LinearMotionConfig that on_start() builds and the exact
# control flow of on_update() without any hardware or event loop.
# ===========================================================================


class FakeMotion:
    """Stand-in for the C++ LinearMotion built inside on_start()."""

    def __init__(self, finished_seq=(False,)) -> None:
        self._finished = list(finished_seq)
        self.started = 0
        self.update_dts: list[float] = []
        self._idx = 0

    def start(self) -> None:
        self.started += 1

    def update(self, dt: float) -> None:
        self.update_dts.append(dt)

    def is_finished(self) -> bool:
        val = self._finished[min(self._idx, len(self._finished) - 1)]
        self._idx += 1
        return val


class FakeStoppingCondition(StopCondition):
    """Condition with a scripted check() sequence; records start()/check()."""

    def __init__(self, check_seq=(False,)) -> None:
        self._seq = list(check_seq)
        self.started = 0
        self.checks = 0

    def start(self, robot) -> None:
        self.started += 1

    def check(self, robot) -> bool:
        val = self._seq[min(self.checks, len(self._seq) - 1)]
        self.checks += 1
        return val


class FakeTrimService:
    """Records scale_distance_m() calls; multiplies by a per-axis scale."""

    def __init__(self, forward=1.0, lateral=1.0) -> None:
        self._scale = {"forward": forward, "lateral": lateral}
        self.calls: list[tuple[str, float]] = []

    def scale_distance_m(self, axis_name, distance_m):
        self.calls.append((axis_name, distance_m))
        return distance_m * self._scale[axis_name]


class FakeHeadingRef:
    """Records target_absolute_rad(); echoes a transform distinct from radians()."""

    def __init__(self) -> None:
        self.calls: list[float] = []

    def target_absolute_rad(self, deg):
        self.calls.append(deg)
        # +1000 makes the heading-service path unmistakably distinct from the
        # plain-radians odometry-fallback path.
        return math.radians(deg) + 1000.0


class _FakePose:
    def __init__(self, heading) -> None:
        self.heading = heading


class FakeOdometry:
    def __init__(self, heading=0.0) -> None:
        self._heading = heading

    def get_pose(self):
        return _FakePose(self._heading)


class FakeRobot:
    def __init__(self, heading=0.0, trim=None) -> None:
        self.drive = object()
        self.odometry = FakeOdometry(heading)
        self.motion_pid_config = object()
        self._trim = trim or FakeTrimService()
        self._heading_ref = FakeHeadingRef()

    def get_service(self, svc_type):
        from raccoon.step.motion._motion_trim import MotionTrimService

        if svc_type is MotionTrimService:
            return self._trim
        return self._heading_ref


def _install_doubles(monkeypatch, captured):
    """Patch LinearMotion + the lazy calibration check; capture the config."""

    def fake_linear_motion(drive, odometry, pid, config):
        captured["config"] = config
        captured["ctor_args"] = (drive, odometry, pid)
        m = FakeMotion()
        captured["motion"] = m
        return m

    monkeypatch.setattr(drive, "LinearMotion", fake_linear_motion)

    cal = {"n": 0}

    def fake_check():
        cal["n"] += 1

    import raccoon.step.calibration as calmod

    monkeypatch.setattr(calmod, "check_distance_calibration", fake_check, raising=False)
    captured["cal"] = cal


# ---------------------------------------------------------------------------
# on_start: distance config (cm->m, sign, sentinel, trim, has_target, speed)
# ---------------------------------------------------------------------------


def test_on_start_forward_distance_to_metres(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = DriveForward(cm=25, speed=0.8)
    step._skip_calibration = True
    step.on_start(FakeRobot())
    cfg = cap["config"]
    assert cfg.axis == LinearAxis.Forward
    assert cfg.distance_m == pytest.approx(0.25)  # 25 cm * +1 / 100
    assert cfg.has_distance_target is True
    assert cfg.speed_scale == pytest.approx(0.8)
    assert cap["motion"].started == 1
    assert step._motion is cap["motion"]


def test_on_start_backward_distance_is_negative(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = DriveBackward(cm=40)
    step._skip_calibration = True
    step.on_start(FakeRobot())
    assert cap["config"].distance_m == pytest.approx(-0.40)
    assert cap["config"].axis == LinearAxis.Forward


def test_on_start_strafe_left_lateral_negative(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = StrafeLeft(cm=15)
    step._skip_calibration = True
    step.on_start(FakeRobot())
    assert cap["config"].axis == LinearAxis.Lateral
    assert cap["config"].distance_m == pytest.approx(-0.15)


def test_on_start_strafe_right_lateral_positive(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = StrafeRight(cm=15)
    step._skip_calibration = True
    step.on_start(FakeRobot())
    assert cap["config"].axis == LinearAxis.Lateral
    assert cap["config"].distance_m == pytest.approx(0.15)


def test_on_start_distance_uses_multiplication_not_division(monkeypatch):
    # Kills the `_sign * _cm` -> `_sign / _cm` mutant: with cm=50 the correct
    # value is +0.50 m; division would give 1/50/100 = 0.0002.
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = DriveForward(cm=50)
    step._skip_calibration = True
    step.on_start(FakeRobot())
    assert cap["config"].distance_m == pytest.approx(0.50)


def test_on_start_until_only_uses_sentinel(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    cond = FakeStoppingCondition()
    step = DriveForward(until=cond, speed=0.5)
    step.on_start(FakeRobot())
    assert cap["config"].distance_m == pytest.approx(100.0)  # sentinel * +1
    assert cap["config"].has_distance_target is False
    assert cap["config"].speed_scale == pytest.approx(0.5)
    assert cond.started == 1  # the until-condition was armed


def test_on_start_backward_until_sentinel_negative(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = DriveBackward(until=FakeStoppingCondition())
    step.on_start(FakeRobot())
    assert cap["config"].distance_m == pytest.approx(-100.0)  # sentinel * -1
    assert cap["config"].has_distance_target is False


def test_on_start_speed_scale_propagated(monkeypatch):
    # Kills `config.speed_scale = None`: the configured value must be the speed.
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = DriveForward(cm=10, speed=0.33)
    step._skip_calibration = True
    step.on_start(FakeRobot())
    assert cap["config"].speed_scale == pytest.approx(0.33)


def test_on_start_no_until_does_not_arm_condition(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = DriveForward(cm=10)
    step._skip_calibration = True
    # Should not raise (no condition to start) and motion still built.
    step.on_start(FakeRobot())
    assert cap["motion"].started == 1


# ---------------------------------------------------------------------------
# on_start: distance trim (forward/lateral axis name + scaling)
# ---------------------------------------------------------------------------


def test_on_start_forward_trim_applied(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    trim = FakeTrimService(forward=2.0)
    step = DriveForward(cm=30)
    step._skip_calibration = True
    step.on_start(FakeRobot(trim=trim))
    # raw +0.30 m, axis name "forward", scaled by 2.0 -> 0.60 m.
    assert trim.calls == [("forward", pytest.approx(0.30))]
    assert cap["config"].distance_m == pytest.approx(0.60)


def test_on_start_lateral_trim_axis_name(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    trim = FakeTrimService(lateral=0.5)
    step = StrafeRight(cm=20)
    step._skip_calibration = True
    step.on_start(FakeRobot(trim=trim))
    # Lateral axis must map to the "lateral" trim key, not "forward".
    assert trim.calls[0][0] == "lateral"
    assert cap["config"].distance_m == pytest.approx(0.10)  # +0.20 * 0.5


def test_on_start_no_trim_in_until_mode(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    trim = FakeTrimService(forward=5.0)
    step = DriveForward(until=FakeStoppingCondition())
    step.on_start(FakeRobot(trim=trim))
    assert trim.calls == []  # trim only applies when there is a real distance
    assert cap["config"].distance_m == pytest.approx(100.0)


def test_on_start_motion_built_with_robot_subsystems(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    robot = FakeRobot()
    step = DriveForward(cm=10)
    step._skip_calibration = True
    step.on_start(robot)
    d, o, p = cap["ctor_args"]
    assert d is robot.drive
    assert o is robot.odometry
    assert p is robot.motion_pid_config


# ---------------------------------------------------------------------------
# on_start: heading resolution (absolute service vs odometry fallback)
# ---------------------------------------------------------------------------


def test_on_start_heading_none_uses_odometry(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    robot = FakeRobot(heading=0.7)
    step = DriveForward(cm=10)
    step._skip_calibration = True
    step.on_start(robot)
    # No pinned heading: target = current odometry heading, in radians.
    assert cap["config"].target_heading_rad == pytest.approx(0.7)
    assert robot._heading_ref.calls == []  # service NOT consulted


def test_on_start_heading_given_uses_reference_service(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    robot = FakeRobot(heading=0.7)
    step = DriveForward(cm=10, heading=90)
    step._skip_calibration = True
    step.on_start(robot)
    assert robot._heading_ref.calls == [90]
    # FakeHeadingRef: radians(90)+1000 — proves the service value flowed in,
    # not the odometry heading (0.7).
    assert cap["config"].target_heading_rad == pytest.approx(math.radians(90) + 1000.0)


def test_on_start_heading_zero_takes_service_path(monkeypatch):
    # heading=0 is NOT None -> must use the service, not the odometry fallback.
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    robot = FakeRobot(heading=0.5)
    step = DriveForward(cm=10, heading=0)
    step._skip_calibration = True
    step.on_start(robot)
    assert robot._heading_ref.calls == [0]
    assert cap["config"].target_heading_rad == pytest.approx(1000.0)


# ---------------------------------------------------------------------------
# on_start: calibration gate (distance mode checks unless skipped / until mode)
# ---------------------------------------------------------------------------


def test_on_start_distance_mode_checks_calibration(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = DriveForward(cm=10)  # _skip_calibration defaults False
    step.on_start(FakeRobot())
    assert cap["cal"]["n"] == 1


def test_on_start_skip_flag_suppresses_calibration(monkeypatch):
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = DriveForward(cm=10)
    step._skip_calibration = True
    step.on_start(FakeRobot())
    assert cap["cal"]["n"] == 0


def test_on_start_until_mode_skips_calibration(monkeypatch):
    # No distance target -> no distance calibration required.
    cap: dict = {}
    _install_doubles(monkeypatch, cap)
    step = DriveForward(until=FakeStoppingCondition())
    step.on_start(FakeRobot())
    assert cap["cal"]["n"] == 0


# ---------------------------------------------------------------------------
# on_update: until short-circuit, motion update, completion flag
# ---------------------------------------------------------------------------


def test_on_update_no_until_returns_motion_unfinished():
    step = DriveForward(cm=10)
    step._motion = FakeMotion(finished_seq=[False])
    assert step.on_update(object(), 0.05) is False
    assert step._motion.update_dts == [0.05]


def test_on_update_no_until_returns_motion_finished():
    step = DriveForward(cm=10)
    step._motion = FakeMotion(finished_seq=[True])
    assert step.on_update(object(), 0.05) is True


def test_on_update_until_true_short_circuits_and_returns_true():
    # Kills the `return True` -> `return False` mutant AND the `and` -> `or`
    # mutant: when the condition fires we stop and DO NOT step the motion.
    cond = FakeStoppingCondition(check_seq=[True])
    step = DriveForward(until=cond)
    step._motion = FakeMotion(finished_seq=[False])
    assert step.on_update(object(), 0.05) is True
    assert cond.checks == 1
    assert step._motion.update_dts == []  # motion not advanced this tick


def test_on_update_until_false_falls_through_to_motion():
    # Kills the `and` -> `or` mutant: with until present but check() False,
    # the motion MUST still be stepped (or-mutant would early-return True).
    cond = FakeStoppingCondition(check_seq=[False])
    step = DriveForward(until=cond)
    step._motion = FakeMotion(finished_seq=[False])
    assert step.on_update(object(), 0.07) is False
    assert cond.checks == 1
    assert step._motion.update_dts == [0.07]


def test_on_update_until_false_then_motion_finishes():
    cond = FakeStoppingCondition(check_seq=[False])
    step = DriveForward(until=cond)
    step._motion = FakeMotion(finished_seq=[True])
    assert step.on_update(object(), 0.07) is True
    assert step._motion.update_dts == [0.07]


def test_on_update_passes_dt_through_unchanged():
    step = DriveForward(cm=10)
    step._motion = FakeMotion(finished_seq=[False, False])
    step.on_update(object(), 0.02)
    step.on_update(object(), 0.13)
    assert step._motion.update_dts == [0.02, 0.13]


# ---------------------------------------------------------------------------
# Signature: heading token shown only when set (guards the `is not None` test)
# ---------------------------------------------------------------------------


def test_signature_shows_heading_when_set():
    sig = DriveForward(cm=10, heading=90)._generate_signature()
    assert "90" in sig


def test_signature_shows_heading_zero():
    # heading=0 is not None -> the heading clause must still render.
    sig = DriveForward(cm=10, heading=0)._generate_signature()
    assert "0.0" in sig


def test_signature_carries_speed_value():
    sig = DriveForward(cm=25, speed=0.8)._generate_signature()
    assert "0.80" in sig


# ---------------------------------------------------------------------------
# Base-class default attributes are the documented direction convention
# ---------------------------------------------------------------------------


def test_base_axis_and_sign_defaults():
    # The base _ConditionalDrive ships Forward / +1 so an undirected subclass
    # would drive forward, not nowhere.
    assert _ConditionalDrive._axis == LinearAxis.Forward
    assert _ConditionalDrive._sign == 1.0


def test_base_init_default_speed_is_one():
    # Construct straight through the base __init__ (bypassing subclass defaults)
    # so the base `speed=1.0` default is the value under test.
    step = _ConditionalDrive(cm=10)
    assert step._speed == 1.0


# ---------------------------------------------------------------------------
# DSL metadata: tags per public subclass (kills decorator-removal + tag mutants)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("cls", "tags"),
    [
        (DriveForward, ("motion", "drive")),
        (DriveBackward, ("motion", "drive")),
        (StrafeLeft, ("motion", "strafe")),
        (StrafeRight, ("motion", "strafe")),
    ],
)
def test_dsl_tags_per_subclass(cls, tags):
    assert cls.__dsl__.tags == tags
    assert cls.__dsl_step_tags__ == tags
    assert cls.__dsl_step__ is True
