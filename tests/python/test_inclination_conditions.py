"""Tests for the DMP-quaternion inclination stop conditions.

Covers ``on_incline`` / ``on_level`` / ``over_ramp`` — the ramp-detection
conditions that read the IMU's fused orientation quaternion and compute how far
the chassis is tilted off a flat reference. No C++ drivetrain or real IMU is
needed: a ``FakeIMU`` is injected into each condition (by pre-setting ``_imu``)
and a flat reference quaternion is injected (by pre-setting ``_q_ref``) before
``start()``, so the pure-Python tilt math and threshold/hysteresis logic are
exercised in isolation.
"""

from __future__ import annotations

import math

import pytest

from raccoon.step.condition import on_incline, on_level, over_ramp

ROBOT = object()  # sentinel: never touched once _q_ref is pre-injected
FLAT_REF = (1.0, 0.0, 0.0, 0.0)  # identity quaternion = flat reference


class FakeIMU:
    """IMU stand-in whose orientation is a DMP-style fused quaternion.

    ``get_quaternion()`` returns ``(w, x, y, z)`` like the real binding.
    """

    def __init__(self, quat: tuple[float, float, float, float] = FLAT_REF) -> None:
        self.quat = quat

    def set_tilt(self, degrees: float, axis: str = "y") -> None:
        """Orient as if the chassis were pitched (y) or rolled (x) by ``degrees``."""
        h = math.radians(degrees) / 2.0
        c, s = math.cos(h), math.sin(h)
        if axis == "x":
            self.quat = (c, s, 0.0, 0.0)
        else:
            self.quat = (c, 0.0, s, 0.0)

    def get_quaternion(self):
        return self.quat


def _armed(condition, imu: FakeIMU, ref: tuple[float, float, float, float] = FLAT_REF):
    """Inject the fake IMU + flat reference and run start(), returning the condition."""
    condition._imu = imu
    condition._q_ref = ref
    condition.start(ROBOT)
    return condition


# --------------------------------------------------------------------------- #
# on_incline                                                                   #
# --------------------------------------------------------------------------- #
def test_on_incline_stays_quiet_while_flat():
    imu = FakeIMU()
    c = _armed(on_incline(10, smoothing=1.0), imu)
    assert c.check(ROBOT) is False


def test_on_incline_fires_past_threshold():
    imu = FakeIMU()
    c = _armed(on_incline(10, smoothing=1.0), imu)
    imu.set_tilt(20)  # 20deg ramp exceeds the 10deg threshold
    assert c.check(ROBOT) is True


def test_on_incline_holds_below_threshold():
    imu = FakeIMU()
    c = _armed(on_incline(10, smoothing=1.0), imu)
    imu.set_tilt(6)  # gentle slope, under threshold
    assert c.check(ROBOT) is False


def test_on_incline_is_sign_agnostic_down_ramp():
    # Tilting the other way (nose down) must read the same magnitude.
    imu = FakeIMU()
    c = _armed(on_incline(10, smoothing=1.0), imu)
    imu.set_tilt(-20)
    assert c.check(ROBOT) is True


def test_on_incline_detects_roll():
    # Sideways (roll) tilt also counts: the relative tilt is axis-agnostic.
    imu = FakeIMU()
    c = _armed(on_incline(10, smoothing=1.0), imu)
    imu.set_tilt(15, axis="x")
    assert c.check(ROBOT) is True


def test_on_incline_is_yaw_invariant():
    # A pure yaw (heading) change must NOT read as tilt.
    imu = FakeIMU()
    c = _armed(on_incline(10, smoothing=1.0), imu)
    h = math.radians(80) / 2.0
    imu.quat = (math.cos(h), 0.0, 0.0, math.sin(h))  # 80deg yaw, zero tilt
    assert c.check(ROBOT) is False


def test_on_incline_relative_to_nonflat_reference():
    # Reference captured at a slightly tilted mount: tilt is measured relative
    # to it, so the same absolute orientation reads ~0.
    h = math.radians(18) / 2.0
    tilted_ref = (math.cos(h), 0.0, math.sin(h), 0.0)
    imu = FakeIMU(quat=tilted_ref)
    c = _armed(on_incline(10, smoothing=1.0), imu, ref=tilted_ref)
    assert c.check(ROBOT) is False  # same as reference -> 0deg relative tilt


# --------------------------------------------------------------------------- #
# on_level                                                                     #
# --------------------------------------------------------------------------- #
def test_on_level_fires_when_flat():
    imu = FakeIMU()
    c = _armed(on_level(4, smoothing=1.0), imu)
    assert c.check(ROBOT) is True


def test_on_level_quiet_while_tilted():
    imu = FakeIMU()
    imu.set_tilt(20)
    c = _armed(on_level(4, smoothing=1.0), imu)
    assert c.check(ROBOT) is False


def test_on_level_fires_after_returning_flat():
    imu = FakeIMU()
    imu.set_tilt(20)
    c = _armed(on_level(4, smoothing=1.0), imu)
    assert c.check(ROBOT) is False
    imu.set_tilt(2)  # crested back onto the flat
    assert c.check(ROBOT) is True


# --------------------------------------------------------------------------- #
# over_ramp (enter then level, sharing one IMU and one flat reference)         #
# --------------------------------------------------------------------------- #
def test_over_ramp_full_traversal():
    imu = FakeIMU()
    c = over_ramp(enter_deg=10, exit_deg=4, smoothing=1.0)
    # over_ramp is a chain of two fresh conditions; inject the same IMU and the
    # same flat reference into both so on_level measures against true flat, not
    # the tilted orientation present when its stage arms.
    first = c._first
    second = c._second
    first._imu = imu
    first._q_ref = FLAT_REF
    second._imu = imu
    second._q_ref = FLAT_REF
    c.start(ROBOT)

    # Flat at the bottom: neither stage fires.
    assert c.check(ROBOT) is False
    # Climbing the ramp: incline stage satisfied, but level stage armed now.
    imu.set_tilt(20)
    assert c.check(ROBOT) is False
    # Still tilted while ascending: not done yet.
    assert c.check(ROBOT) is False
    # Crested onto the flat top: level stage fires -> whole ramp crossed.
    imu.set_tilt(1)
    assert c.check(ROBOT) is True


# --------------------------------------------------------------------------- #
# smoothing / EMA behaviour                                                    #
# --------------------------------------------------------------------------- #
def test_smoothing_rejects_a_single_spike():
    imu = FakeIMU()
    c = _armed(on_incline(10, smoothing=0.2), imu)
    # One noisy sample that on its own would exceed threshold...
    imu.set_tilt(30)
    # ...is attenuated by the EMA (0.2*30deg-worth ~ well under 10deg) so it
    # does not fire on the first spike tick.
    assert c.check(ROBOT) is False


def test_smoothing_converges_on_sustained_tilt():
    imu = FakeIMU()
    c = _armed(on_incline(10, smoothing=0.3), imu)
    imu.set_tilt(20)
    fired = any(c.check(ROBOT) for _ in range(50))
    assert fired is True


# --------------------------------------------------------------------------- #
# reference / availability errors                                              #
# --------------------------------------------------------------------------- #
class _FakeHeadingService:
    def __init__(self, ref):
        self._ref = ref

    def tilt_reference_quat(self):
        return self._ref


class _FakeRobot:
    def __init__(self, ref):
        self._svc = _FakeHeadingService(ref)

    def get_service(self, _cls):
        return self._svc


def test_start_raises_without_reference():
    # No mark_heading_reference() -> service returns None -> clear error.
    c = on_incline(10)
    c._imu = FakeIMU()
    with pytest.raises(RuntimeError, match="flat tilt reference"):
        c.start(_FakeRobot(None))


def test_start_uses_reference_from_service():
    # With a reference present, start() pulls it from the service.
    c = on_incline(10, smoothing=1.0)
    c._imu = FakeIMU()
    c.start(_FakeRobot(FLAT_REF))
    assert c._q_ref == FLAT_REF
    assert c.check(_FakeRobot(FLAT_REF)) is False


# --------------------------------------------------------------------------- #
# validation                                                                   #
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("bad", [-1, 91, "10"])
def test_on_incline_rejects_bad_degrees(bad):
    with pytest.raises((ValueError, TypeError)):
        on_incline(bad)


def test_on_incline_rejects_bad_up_axis():
    with pytest.raises(ValueError):
        on_incline(10, up_axis="w")


@pytest.mark.parametrize("bad", [0.0, -0.1, 1.5])
def test_on_incline_rejects_bad_smoothing(bad):
    with pytest.raises(ValueError):
        on_incline(10, smoothing=bad)


def test_over_ramp_rejects_inverted_hysteresis():
    with pytest.raises(ValueError):
        over_ramp(enter_deg=5, exit_deg=10)


# --------------------------------------------------------------------------- #
# label / state reporting                                                      #
# --------------------------------------------------------------------------- #
def test_labels_and_state_render():
    imu = FakeIMU()
    imu.set_tilt(20)
    c = _armed(on_incline(10, smoothing=1.0), imu)
    assert "on_incline" in c._label()
    c.check(ROBOT)
    state = c._state(ROBOT)
    assert "tilt=" in state and "20" in state
