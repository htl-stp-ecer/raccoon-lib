"""Direction tests for the distance scale-factor / motion-trim layer.

Contract
--------
When the robot drives **too short** (measured < requested), the calibration
scale factor must make the *next* drive **longer** — never shorter. The factor
is therefore a distance-correction multiplier::

    scale_factor = requested / measured  # > 1 on undershoot, < 1 on overshoot

It is applied by :class:`MotionTrimService` as a multiplier on the commanded
distance (``scale_distance_m`` -> ``distance_m * axis_scale``, wired in
``drive.py``), and reported by :pyattr:`DistanceConfirmScreen.scale_factor` and
``calibrate_distance``'s ``avg_scale_factor``.

Regression history
------------------
These quantities were previously computed as ``measured / requested`` (the
``ticks_to_rad`` ratio). That convention is correct for the odometry gain but is
the *inverse* of a distance-trim multiplier: on an undershoot it is < 1, so
multiplying the commanded distance by it drove the robot **even shorter**. The
tests below pin the corrected ``requested / measured`` direction so the
inversion cannot come back.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
_TRIM_PATH = REPO_ROOT / "modules/libstp-motion/python/raccoon/step/motion/_motion_trim.py"


def libstp_available() -> bool:
    """True when the raccoon native module is importable."""
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not libstp_available(), reason="raccoon native module not installed"
)


# --------------------------------------------------------------------------- #
# Test doubles                                                                 #
# --------------------------------------------------------------------------- #
class _FakeStore:
    """In-memory stand-in for CalibrationStore (no filesystem)."""

    def __init__(self) -> None:
        self.data: dict[str, dict] = {}

    def load(self, section: str) -> dict:
        return dict(self.data.get(section, {}))

    def store(self, section: str, payload: dict) -> None:
        self.data[section] = dict(payload)


class _FakeRobot:
    """Robot stub exposing only the logging hooks the service touches."""

    def debug(self, *_args, **_kwargs) -> None:
        pass

    def warn(self, *_args, **_kwargs) -> None:
        pass


def _make_trim_service():
    """Load the workspace MotionTrimService with the store stubbed out.

    The installed wheel may predate the trim layer, so load the class straight
    from the workspace source file and replace ``_make_store`` so construction
    never touches the calibration store on disk.
    """
    spec = importlib.util.spec_from_file_location("raccoon_test_motion_trim", _TRIM_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    module.MotionTrimService._make_store = staticmethod(_FakeStore)  # type: ignore[attr-defined]
    return module.MotionTrimService(_FakeRobot())


# --------------------------------------------------------------------------- #
# 1. The compensation contract (pure math, no imports) — the spec.            #
# --------------------------------------------------------------------------- #
class TestTrimDirectionContract:
    """The correct factor convention for a distance-trim multiplier."""

    def test_factor_is_requested_over_measured(self):
        # drove short: commanded 30 cm, only travelled 25 cm
        requested, measured = 30.0, 25.0
        assert requested / measured > 1.0  # undershoot -> lengthen next drive

    def test_factor_round_trips_undershoot_to_target(self):
        requested, measured = 30.0, 25.0
        # Real-world transfer: commanding X actually travels X * (measured/requested).
        gain = measured / requested
        trim = requested / measured

        commanded = requested * trim
        assert commanded * gain == pytest.approx(requested)

    def test_factor_round_trips_overshoot_to_target(self):
        # drove long: commanded 30 cm, travelled 35 cm
        requested, measured = 30.0, 35.0
        gain = measured / requested
        trim = requested / measured
        assert trim < 1.0  # overshoot -> shorten the next drive

        commanded = requested * trim
        assert commanded * gain == pytest.approx(requested)

    def test_inverted_factor_would_make_undershoot_worse(self):
        # Guards the historical bug: the measured/requested convention, used as
        # a multiplier on the commanded distance, drives EVEN SHORTER.
        requested, measured = 30.0, 25.0
        gain = measured / requested
        inverted = measured / requested  # the old, wrong convention

        commanded = requested * inverted
        assert commanded * gain < measured < requested  # worse than doing nothing


# --------------------------------------------------------------------------- #
# 2. MotionTrimService application semantics (real workspace code).            #
# --------------------------------------------------------------------------- #
class TestMotionTrimServiceApplication:
    def test_default_scale_is_identity(self):
        svc = _make_trim_service()
        assert svc.scale_distance_m("forward", 0.30) == pytest.approx(0.30)
        assert svc.get_axis_scale("forward") == pytest.approx(1.0)

    def test_set_and_apply_scale_multiplies(self):
        svc = _make_trim_service()
        svc.set_axis_scale("forward", 1.2)
        assert svc.scale_distance_m("forward", 0.30) == pytest.approx(0.36)

    def test_axes_are_independent(self):
        svc = _make_trim_service()
        svc.set_axis_scale("forward", 1.2)
        svc.set_axis_scale("lateral", 0.8)
        assert svc.scale_distance_m("forward", 1.0) == pytest.approx(1.2)
        assert svc.scale_distance_m("lateral", 1.0) == pytest.approx(0.8)

    def test_rejects_non_positive_scale(self):
        svc = _make_trim_service()
        with pytest.raises(ValueError):
            svc.set_axis_scale("forward", 0.0)
        with pytest.raises(ValueError):
            svc.set_axis_scale("forward", -1.0)

    def test_rejects_unknown_axis(self):
        svc = _make_trim_service()
        with pytest.raises(ValueError):
            svc.set_axis_scale("diagonal", 1.1)

    def test_trim_factor_compensates_undershoot(self):
        """Storing requested/measured lengthens the next drive to hit target."""
        svc = _make_trim_service()
        requested_m, measured_m = 0.30, 0.25
        gain = measured_m / requested_m

        svc.set_axis_scale("forward", requested_m / measured_m)
        commanded = svc.scale_distance_m("forward", requested_m)

        assert commanded > requested_m
        assert commanded * gain == pytest.approx(requested_m)

    def test_trim_factor_compensates_overshoot(self):
        svc = _make_trim_service()
        requested_m, measured_m = 0.30, 0.35
        gain = measured_m / requested_m

        svc.set_axis_scale("lateral", requested_m / measured_m)
        commanded = svc.scale_distance_m("lateral", requested_m)

        assert commanded < requested_m
        assert commanded * gain == pytest.approx(requested_m)

    @requires_libstp
    def test_screen_scale_factor_fed_to_trim_compensates_undershoot(self):
        """End-to-end: the calibration screen's factor lengthens an undershoot.

        The factor reported by the calibration confirm screen is exactly the
        value intended for the trim axis scale, so feeding it back through the
        trim service must correct — never amplify — the original error.
        """
        from raccoon.ui.screens.distance import DistanceConfirmScreen

        svc = _make_trim_service()
        requested_cm, measured_cm = 30.0, 25.0
        gain = measured_cm / requested_cm

        screen = DistanceConfirmScreen(requested=requested_cm, measured=measured_cm)
        svc.set_axis_scale("forward", screen.scale_factor)

        commanded = svc.scale_distance_m("forward", requested_cm)
        assert commanded > requested_cm
        assert commanded * gain == pytest.approx(requested_cm)


# --------------------------------------------------------------------------- #
# 3. DistanceConfirmScreen.scale_factor convention (real installed code).      #
# --------------------------------------------------------------------------- #
class TestDistanceConfirmScaleFactorConvention:
    @requires_libstp
    def test_scale_factor_is_requested_over_measured(self):
        from raccoon.ui.screens.distance import DistanceConfirmScreen

        screen = DistanceConfirmScreen(requested=30.0, measured=25.0)
        assert screen.scale_factor == pytest.approx(30.0 / 25.0)

    @requires_libstp
    def test_scale_factor_lengthens_on_undershoot(self):
        from raccoon.ui.screens.distance import DistanceConfirmScreen

        screen = DistanceConfirmScreen(requested=30.0, measured=25.0)
        assert screen.scale_factor > 1.0, (
            "drove short (25 < 30) — the correction factor must be > 1 so the "
            "next drive is lengthened, not shortened"
        )

    @requires_libstp
    def test_scale_factor_shortens_on_overshoot(self):
        from raccoon.ui.screens.distance import DistanceConfirmScreen

        screen = DistanceConfirmScreen(requested=30.0, measured=35.0)
        assert screen.scale_factor < 1.0

    @requires_libstp
    def test_adjustment_is_positive_on_undershoot(self):
        from raccoon.ui.screens.distance import DistanceConfirmScreen

        screen = DistanceConfirmScreen(requested=30.0, measured=25.0)
        assert (
            screen.adjustment > 0.0
        ), "drove short — the reported adjustment must ask for MORE distance"

    @requires_libstp
    def test_scale_factor_handles_zero_measured(self):
        from raccoon.ui.screens.distance import DistanceConfirmScreen

        screen = DistanceConfirmScreen(requested=30.0, measured=0.0)
        assert screen.scale_factor == pytest.approx(1.0)


# --------------------------------------------------------------------------- #
# 4. calibrate_distance avg_scale_factor convention (pure-math mirror).        #
# --------------------------------------------------------------------------- #
class TestCalibrateDistanceScaleFactorConvention:
    """avg_scale_factor == old_ttr / new_ttr == requested / measured."""

    def test_avg_scale_factor_lengthens_on_undershoot(self):
        wheel_radius, delta_ticks = 0.05, 1000
        requested_m, measured_m = 0.30, 0.25

        old_ttr = (requested_m / wheel_radius) / delta_ticks
        new_ttr = (measured_m / wheel_radius) / delta_ticks

        # Mirror of the fixed formula in calibrate_distance.py.
        scale_factor = old_ttr / new_ttr

        assert scale_factor > 1.0
        assert scale_factor == pytest.approx(requested_m / measured_m)

    def test_avg_scale_factor_shortens_on_overshoot(self):
        wheel_radius, delta_ticks = 0.05, 1000
        requested_m, measured_m = 0.30, 0.35

        old_ttr = (requested_m / wheel_radius) / delta_ticks
        new_ttr = (measured_m / wheel_radius) / delta_ticks

        scale_factor = old_ttr / new_ttr

        assert scale_factor < 1.0
        assert scale_factor == pytest.approx(requested_m / measured_m)
