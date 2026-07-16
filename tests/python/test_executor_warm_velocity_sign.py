"""``_warm_velocity`` must re-sign the profile's entry speed by travel direction.

Reproduces (pure-Python, no C++/sim) the hardware defect from run
``run1_20260713-195444`` M050 cone return: the path compiled correctly as
``Drive(back -35) → Arc(+25°, reverse) → Drive(back -30)``, but the LAST drive —
the only leg that warm-starts off the profiled reverse arc — lunged FORWARD
(commanded ``vx=+0.237`` toward a ``-0.30`` goal) instead of backward.

Root cause: ``entry_speed_mps`` is a MAGNITUDE (``velocity_profile`` caps
everything with ``abs(speed_scale)``), so ``_warm_velocity`` seeded
``LinearMotion::startWarm`` with a POSITIVE velocity for a backward leg. The
trapezoidal profile then cruised forward before it could chase the negative
goal. The fix signs the seed with the segment's travel direction (linear:
``distance_m`` sign; arc: ``speed_scale`` sign).
"""

from __future__ import annotations

import importlib.util

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


@pytest.fixture
def _mod():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion.path import executor as ex
    from raccoon.step.motion.path.ir import Segment

    return ex, Segment, LinearAxis


# ---------------------------------------------------------------------------
# Linear legs — seed sign follows distance_m
# ---------------------------------------------------------------------------


@requires_libstp
def test_backward_linear_warm_seed_is_negative(_mod):
    """The regression: a backward drive warm-starting off a profiled seam must
    seed a NEGATIVE velocity, not the bare positive magnitude."""
    ex, Segment, LinearAxis = _mod
    seg = Segment(
        kind="linear",
        axis=LinearAxis.Forward,
        sign=-1.0,
        distance_m=-0.30,
        entry_speed_mps=0.20,  # magnitude from velocity_profile (always >= 0)
    )
    assert ex._warm_velocity(seg, fallback_mps=99.0) == pytest.approx(-0.20)


@requires_libstp
def test_forward_linear_warm_seed_is_positive(_mod):
    ex, Segment, LinearAxis = _mod
    seg = Segment(
        kind="linear",
        axis=LinearAxis.Forward,
        sign=1.0,
        distance_m=0.30,
        entry_speed_mps=0.20,
    )
    assert ex._warm_velocity(seg, fallback_mps=99.0) == pytest.approx(0.20)


@requires_libstp
def test_linear_seed_falls_back_to_sign_when_distance_zero(_mod):
    """A condition-bounded profiled leg (distance_m unset/zero) uses ``sign``."""
    ex, Segment, LinearAxis = _mod
    seg = Segment(
        kind="linear",
        axis=LinearAxis.Forward,
        sign=-1.0,
        distance_m=None,
        entry_speed_mps=0.20,
    )
    assert ex._warm_velocity(seg, fallback_mps=99.0) == pytest.approx(-0.20)


# ---------------------------------------------------------------------------
# Arc legs — seed sign follows speed_scale, magnitude divided by |radius|
# ---------------------------------------------------------------------------


@requires_libstp
def test_reverse_arc_warm_seed_is_negative_angular(_mod):
    ex, Segment, _LinearAxis = _mod
    seg = Segment(
        kind="arc",
        radius_m=0.451,
        arc_angle_rad=0.436,
        speed_scale=-1.0,  # reverse arc
        entry_speed_mps=0.20,
    )
    # angular seed = direction * v / |radius|
    assert ex._warm_velocity(seg, fallback_mps=99.0) == pytest.approx(-0.20 / 0.451)


@requires_libstp
def test_forward_arc_warm_seed_is_positive_angular(_mod):
    ex, Segment, _LinearAxis = _mod
    seg = Segment(
        kind="arc",
        radius_m=0.451,
        arc_angle_rad=0.436,
        speed_scale=1.0,
        entry_speed_mps=0.20,
    )
    assert ex._warm_velocity(seg, fallback_mps=99.0) == pytest.approx(0.20 / 0.451)


# ---------------------------------------------------------------------------
# Non-profiled / non-carrying legs still fall back to the measured velocity
# ---------------------------------------------------------------------------


@requires_libstp
def test_non_profiled_leg_uses_measured_fallback(_mod):
    ex, Segment, LinearAxis = _mod
    seg = Segment(
        kind="linear",
        axis=LinearAxis.Forward,
        sign=-1.0,
        distance_m=-0.30,
        entry_speed_mps=None,  # not profiled → carry the measured Ist-velocity
    )
    assert ex._warm_velocity(seg, fallback_mps=-0.22) == pytest.approx(-0.22)
