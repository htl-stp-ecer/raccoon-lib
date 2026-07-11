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


class _RecordingRobot:
    """Robot stub that records debug()/warn() calls for assertion."""

    def __init__(self) -> None:
        self.debug_msgs: list[str] = []
        self.warn_msgs: list[str] = []

    def debug(self, msg: str, *_a, **_k) -> None:
        self.debug_msgs.append(msg)

    def warn(self, msg: str, *_a, **_k) -> None:
        self.warn_msgs.append(msg)


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
# Real-module helpers — import the INSTALLED module (symlinked to source) so   #
# coverage attributes to the source file path. _make_store is patched on a     #
# subclass to avoid touching the real CalibrationStore / disk.                 #
# --------------------------------------------------------------------------- #
def _real_service(store=None, robot=None):
    """Construct the real MotionTrimService with an injected fake store/robot.

    Pre-seed ``store`` with section data before calling to exercise the load
    path. Returns ``(service, store, robot)``.
    """
    from raccoon.step.motion._motion_trim import MotionTrimService

    fake_store = store if store is not None else _FakeStore()
    fake_robot = robot if robot is not None else _RecordingRobot()

    class _Svc(MotionTrimService):
        @staticmethod
        def _make_store():
            return fake_store

    svc = _Svc(fake_robot)
    return svc, fake_store, fake_robot


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

    def test_bypass_scaling_returns_raw_distance(self):
        """Inside bypass_scaling(), a commanded distance is NOT trimmed.

        Calibration measurement drives run through this context so the physical
        distance they travel — and thus their odom/ground-truth ratio — is
        independent of the (possibly wrong) stored scale.
        """
        svc = _make_trim_service()
        svc.set_axis_scale("forward", 0.5)  # would halve a normal command
        assert svc.scale_distance_m("forward", 0.70) == pytest.approx(0.35)
        with svc.bypass_scaling():
            assert svc.scaling_bypassed is True
            assert svc.scale_distance_m("forward", 0.70) == pytest.approx(0.70)
        # Scaling resumes after the context exits.
        assert svc.scaling_bypassed is False
        assert svc.scale_distance_m("forward", 0.70) == pytest.approx(0.35)

    def test_bypass_scaling_is_reentrant(self):
        svc = _make_trim_service()
        svc.set_axis_scale("forward", 2.0)
        with svc.bypass_scaling():
            with svc.bypass_scaling():
                assert svc.scale_distance_m("forward", 1.0) == pytest.approx(1.0)
            # Still bypassed after the inner context exits (depth 1 > 0).
            assert svc.scaling_bypassed is True
            assert svc.scale_distance_m("forward", 1.0) == pytest.approx(1.0)
        assert svc.scaling_bypassed is False
        assert svc.scale_distance_m("forward", 1.0) == pytest.approx(2.0)

    def test_bypass_scaling_restored_on_exception(self):
        svc = _make_trim_service()
        svc.set_axis_scale("forward", 2.0)
        with pytest.raises(RuntimeError), svc.bypass_scaling():
            msg = "boom"
            raise RuntimeError(msg)
        # The finally clause must have decremented the depth back to 0.
        assert svc.scaling_bypassed is False
        assert svc.scale_distance_m("forward", 1.0) == pytest.approx(2.0)

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
# 2b. Re-calibration must COMPOSE with an already-applied scale factor.        #
# --------------------------------------------------------------------------- #
class TestTrimCalibrationComposesWithExistingScale:
    """A fresh calibration drive runs *through* the already-applied trim.

    ``drive.py`` scales every commanded distance by the current axis factor
    (``scale_distance_m`` -> ``distance_m * axis_scale``). So when the robot
    performs a calibration drive while a non-identity factor is already active,
    the physical distance it travels is ``requested * current_scale``, and the
    fresh correction ``requested / measured`` is *relative to that*, not to the
    raw command. Storing ``requested / measured`` therefore **discards** the
    existing factor; the new factor must compose:

        new_scale = current_scale * (requested / measured)
    """

    def test_recalibration_composes_with_existing_scale(self):
        svc = _make_trim_service()

        # A previous calibration already established a 1.2x forward trim.
        current_scale = 1.2
        svc.set_axis_scale("forward", current_scale)

        # The drivetrain is now mechanically perfect: commanding a physical
        # distance D actually travels D (gain == 1.0). A 0.30 m calibration
        # drive is scaled to 0.30 * 1.2 = 0.36 m physical and travels 0.36 m.
        requested_m = 0.30
        gain = 1.0
        commanded_physical = svc.scale_distance_m("forward", requested_m)
        measured_m = commanded_physical * gain  # == 0.36

        # Apply the fresh measurement back to the trim axis.
        svc.calibrate_axis("forward", requested_m, measured_m)

        # With a perfect drivetrain the existing 1.2x is exactly the error, so
        # the corrected factor must be 1.0 — NOT the naive 0.30/0.36 == 0.833,
        # which would throw away the already-applied scale.
        assert svc.get_axis_scale("forward") == pytest.approx(1.0)

        # And the round-trip holds: a subsequent 0.30 m drive lands on target.
        next_commanded = svc.scale_distance_m("forward", requested_m)
        assert next_commanded * gain == pytest.approx(requested_m)

    def test_recalibration_round_trips_with_existing_scale_and_real_gain(self):
        svc = _make_trim_service()

        # Start from an arbitrary already-applied factor and a real (lossy)
        # transfer gain. After composing the fresh correction, commanding the
        # target must travel the target.
        current_scale = 0.9
        svc.set_axis_scale("forward", current_scale)

        requested_m = 0.50
        gain = 0.8  # commanding X physically travels 0.8 * X
        commanded_physical = svc.scale_distance_m("forward", requested_m)
        measured_m = commanded_physical * gain

        svc.calibrate_axis("forward", requested_m, measured_m)

        next_commanded = svc.scale_distance_m("forward", requested_m)
        assert next_commanded * gain == pytest.approx(requested_m)


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


# --------------------------------------------------------------------------- #
# 5. Module-level constants (kill string/tuple literal mutants).               #
#    These import the REAL installed module so coverage tracks the source.     #
# --------------------------------------------------------------------------- #
class TestModuleConstants:
    def test_module_reimports_cleanly_under_coverage(self):
        # The module is imported at collection time, before coverage hooks the
        # file, so its def/decorator lines would read as "missed". Force a fresh
        # exec under the active coverage tracer so those lines are counted and
        # their mutants are exercised.
        import raccoon.step.motion._motion_trim as mod

        importlib.reload(mod)
        assert mod.TRIM_STORE_SECTION == "motion-trim"
        assert mod._TRIM_AXES == ("forward", "lateral")
        assert callable(mod.MotionTrimService._make_store)

    @requires_libstp
    def test_make_store_returns_calibration_store(self):
        # Exercise the real _make_store body (import + construct) without
        # building a full service (which would also call load() on disk).
        from raccoon.step.calibration.store import CalibrationStore
        from raccoon.step.motion._motion_trim import MotionTrimService

        store = MotionTrimService._make_store()
        assert isinstance(store, CalibrationStore)

    @requires_libstp
    def test_constructor_invokes_make_store_as_staticmethod(self, monkeypatch):
        # Kills the `@staticmethod` removal mutant: __init__ calls
        # self._make_store(). If _make_store were a plain method, the bound
        # `self` would be passed as a positional arg and the zero-arg body
        # would raise TypeError. Patch CalibrationStore so the REAL _make_store
        # body runs (no override) but never touches disk.
        import raccoon.step.calibration.store as store_mod
        from raccoon.step.motion._motion_trim import MotionTrimService

        monkeypatch.setattr(store_mod, "CalibrationStore", _FakeStore)

        # No subclass / no _make_store override here: the genuine staticmethod
        # is invoked via self._make_store() during construction.
        svc = MotionTrimService(_RecordingRobot())
        assert isinstance(svc._store, _FakeStore)
        assert svc.get_axis_scale("forward") == pytest.approx(1.0)

    def test_trim_store_section_exact_string(self):
        from raccoon.step.motion._motion_trim import TRIM_STORE_SECTION

        assert TRIM_STORE_SECTION == "motion-trim"

    def test_trim_axes_exact_tuple(self):
        from raccoon.step.motion._motion_trim import _TRIM_AXES

        # Exact value, order, type and length — kills any element/order mutant.
        assert _TRIM_AXES == ("forward", "lateral")
        assert isinstance(_TRIM_AXES, tuple)
        assert list(_TRIM_AXES) == ["forward", "lateral"]


# --------------------------------------------------------------------------- #
# 6. set_axis_scale validation + persistence on the REAL module.               #
# --------------------------------------------------------------------------- #
@requires_libstp
class TestSetAxisScaleReal:
    def test_default_scales_are_identity(self):
        svc, _store, _robot = _real_service()
        assert svc.get_axis_scale("forward") == pytest.approx(1.0)
        assert svc.get_axis_scale("lateral") == pytest.approx(1.0)

    def test_set_scale_applies_and_persists_to_store(self):
        svc, store, _robot = _real_service()
        svc.set_axis_scale("forward", 1.25)
        assert svc.get_axis_scale("forward") == pytest.approx(1.25)
        # _persist() must have written BOTH axes to the trim section.
        assert store.data["motion-trim"]["forward"] == pytest.approx(1.25)
        assert store.data["motion-trim"]["lateral"] == pytest.approx(1.0)

    def test_set_scale_is_coerced_to_float(self):
        svc, _store, _robot = _real_service()
        svc.set_axis_scale("forward", 2)  # int in
        assert isinstance(svc.get_axis_scale("forward"), float)
        assert svc.get_axis_scale("forward") == pytest.approx(2.0)

    def test_unknown_axis_message_and_does_not_persist(self):
        svc, store, _robot = _real_service()
        with pytest.raises(ValueError, match=r"diagonal") as exc:
            svc.set_axis_scale("diagonal", 1.1)
        # The offending axis name is carried; wording is prose, not pinned.
        assert "diagonal" in str(exc.value)
        # Validation happens before _persist, so nothing was written.
        assert "motion-trim" not in store.data

    def test_zero_scale_rejected_with_message(self):
        svc, _store, _robot = _real_service()
        with pytest.raises(ValueError, match=r"0\.0") as exc:
            svc.set_axis_scale("forward", 0.0)
        # The rejected value token is carried; wording is prose, not pinned.
        assert "0.0" in str(exc.value)

    def test_negative_scale_rejected(self):
        svc, store, _robot = _real_service()
        with pytest.raises(ValueError, match=r"axis scale must be > 0, got -2"):
            svc.set_axis_scale("forward", -2.0)
        assert "motion-trim" not in store.data

    def test_tiny_positive_scale_accepted(self):
        # Boundary: strictly > 0 must pass even for a minuscule value.
        svc, _store, _robot = _real_service()
        svc.set_axis_scale("lateral", 1e-9)
        assert svc.get_axis_scale("lateral") == pytest.approx(1e-9)

    def test_scale_distance_multiplies_real(self):
        svc, _store, _robot = _real_service()
        svc.set_axis_scale("forward", 1.5)
        assert svc.scale_distance_m("forward", 0.40) == pytest.approx(0.60)

    def test_get_axis_scale_unknown_axis_defaults_to_one(self):
        # get_axis_scale uses dict.get(axis, 1.0) — unknown axis is NOT an error.
        svc, _store, _robot = _real_service()
        assert svc.get_axis_scale("diagonal") == pytest.approx(1.0)


# --------------------------------------------------------------------------- #
# 7. calibrate_axis composition formula + guards on the REAL module.           #
#    new = current * (requested / measured)                                    #
# --------------------------------------------------------------------------- #
@requires_libstp
class TestCalibrateAxisReal:
    def test_compose_from_identity_undershoot(self):
        # current 1.0, drove short -> factor = 30/25 = 1.2 (> 1, lengthen).
        svc, store, _robot = _real_service()
        result = svc.calibrate_axis("forward", 0.30, 0.25)
        assert result == pytest.approx(1.2)
        assert svc.get_axis_scale("forward") == pytest.approx(1.2)
        assert store.data["motion-trim"]["forward"] == pytest.approx(1.2)

    def test_compose_from_identity_overshoot(self):
        # current 1.0, drove long -> factor = 30/35 < 1 (shorten).
        svc, _store, _robot = _real_service()
        result = svc.calibrate_axis("lateral", 0.30, 0.35)
        assert result == pytest.approx(30.0 / 35.0)
        assert result < 1.0

    def test_compose_with_existing_scale(self):
        # current 1.2; requested 0.30 measured 0.36 -> 1.2 * (0.30/0.36) = 1.0.
        svc, store, _robot = _real_service()
        svc.set_axis_scale("forward", 1.2)
        result = svc.calibrate_axis("forward", 0.30, 0.36)
        assert result == pytest.approx(1.0)
        assert svc.get_axis_scale("forward") == pytest.approx(1.0)
        assert store.data["motion-trim"]["forward"] == pytest.approx(1.0)

    def test_compose_concrete_non_round(self):
        # current 0.9; requested 0.50 measured 0.40 -> 0.9 * 1.25 = 1.125.
        svc, _store, _robot = _real_service()
        svc.set_axis_scale("forward", 0.9)
        result = svc.calibrate_axis("forward", 0.50, 0.40)
        assert result == pytest.approx(0.9 * (0.50 / 0.40))
        assert result == pytest.approx(1.125)

    def test_returns_the_stored_value(self):
        svc, _store, _robot = _real_service()
        returned = svc.calibrate_axis("forward", 0.30, 0.20)
        assert returned == pytest.approx(svc.get_axis_scale("forward"))

    def test_measured_zero_rejected(self):
        svc, store, _robot = _real_service()
        with pytest.raises(ValueError, match=r"0\.0") as exc:
            svc.calibrate_axis("forward", 0.30, 0.0)
        # The rejected value token is carried; wording is prose, not pinned.
        assert "0.0" in str(exc.value)
        assert "motion-trim" not in store.data

    def test_measured_negative_rejected(self):
        svc, _store, _robot = _real_service()
        with pytest.raises(ValueError, match=r"measured distance must be > 0, got -0.1"):
            svc.calibrate_axis("forward", 0.30, -0.1)

    def test_compose_resulting_in_non_positive_scale_rejected(self):
        # A negative requested distance yields a non-positive composed scale,
        # which set_axis_scale must reject (guards downstream multiplication).
        svc, _store, _robot = _real_service()
        with pytest.raises(ValueError, match=r"axis scale must be > 0"):
            svc.calibrate_axis("forward", -0.30, 0.25)

    def test_unknown_axis_rejected_in_calibrate(self):
        svc, _store, _robot = _real_service()
        with pytest.raises(ValueError, match=r"unknown trim axis"):
            svc.calibrate_axis("diagonal", 0.30, 0.25)


# --------------------------------------------------------------------------- #
# 8. _load_stored_scales: load path, filtering, and logging branches.          #
# --------------------------------------------------------------------------- #
@requires_libstp
class TestLoadStoredScalesReal:
    def test_loads_persisted_factors_and_debugs(self):
        store = _FakeStore()
        store.data["motion-trim"] = {"forward": 1.3, "lateral": 0.7}
        svc, _store, robot = _real_service(store=store)
        assert svc.get_axis_scale("forward") == pytest.approx(1.3)
        assert svc.get_axis_scale("lateral") == pytest.approx(0.7)
        # The "loaded" branch must log via debug, not warn.
        assert robot.debug_msgs and not robot.warn_msgs
        joined = " ".join(robot.debug_msgs)
        assert "forward=1.3000" in joined
        assert "lateral=0.7000" in joined

    def test_empty_store_warns_with_section_name(self):
        svc, _store, robot = _real_service()  # empty store
        assert svc.get_axis_scale("forward") == pytest.approx(1.0)
        assert robot.warn_msgs and not robot.debug_msgs
        assert "motion-trim" in robot.warn_msgs[0]

    def test_non_positive_stored_scale_ignored(self):
        # scale <= 0 in the store must be skipped (stays at 1.0) and, being the
        # only entry, falls through to the warn branch.
        store = _FakeStore()
        store.data["motion-trim"] = {"forward": 0.0, "lateral": -1.0}
        svc, _store, robot = _real_service(store=store)
        assert svc.get_axis_scale("forward") == pytest.approx(1.0)
        assert svc.get_axis_scale("lateral") == pytest.approx(1.0)
        assert robot.warn_msgs and not robot.debug_msgs

    def test_partial_and_bad_values_filtered(self):
        # forward is a valid float -> loaded; lateral is non-numeric -> skipped.
        store = _FakeStore()
        store.data["motion-trim"] = {"forward": 1.4, "lateral": "not-a-number"}
        svc, _store, robot = _real_service(store=store)
        assert svc.get_axis_scale("forward") == pytest.approx(1.4)
        assert svc.get_axis_scale("lateral") == pytest.approx(1.0)
        # At least one axis loaded -> debug branch.
        assert robot.debug_msgs
        assert "forward=1.4000" in " ".join(robot.debug_msgs)
        assert "lateral" not in " ".join(robot.debug_msgs)

    def test_missing_axis_key_skipped(self):
        # Only "forward" present; "lateral" missing -> KeyError caught, skipped.
        store = _FakeStore()
        store.data["motion-trim"] = {"forward": 1.1}
        svc, _store, robot = _real_service(store=store)
        assert svc.get_axis_scale("forward") == pytest.approx(1.1)
        assert svc.get_axis_scale("lateral") == pytest.approx(1.0)
        assert robot.debug_msgs

    def test_string_numeric_value_is_coerced(self):
        # float("1.5") succeeds -> the value is loaded (covers float() success).
        store = _FakeStore()
        store.data["motion-trim"] = {"forward": "1.5", "lateral": "2.0"}
        svc, _store, _robot = _real_service(store=store)
        assert svc.get_axis_scale("forward") == pytest.approx(1.5)
        assert svc.get_axis_scale("lateral") == pytest.approx(2.0)

    def test_bad_first_axis_does_not_abort_loading_of_second(self):
        # Kills the `continue` -> `break` mutant: forward is non-numeric (hits
        # the except), but lateral is a valid float and MUST still be loaded.
        # With `break` the loop would abort after forward and lateral stays 1.0.
        store = _FakeStore()
        store.data["motion-trim"] = {"forward": "not-a-number", "lateral": 0.7}
        svc, _store, robot = _real_service(store=store)
        assert svc.get_axis_scale("forward") == pytest.approx(1.0)
        assert svc.get_axis_scale("lateral") == pytest.approx(0.7)
        # lateral loaded -> debug branch fired, with only lateral reported.
        assert robot.debug_msgs and not robot.warn_msgs
        joined = " ".join(robot.debug_msgs)
        assert "lateral=0.7000" in joined
        assert "forward" not in joined

    def test_loaded_debug_message_carries_factors(self):
        # The loaded-branch debug message reports each loaded axis factor.
        # Assert the VALUE tokens are carried; the surrounding wording/format is
        # debug prose, not pinned.
        store = _FakeStore()
        store.data["motion-trim"] = {"forward": 1.3, "lateral": 0.7}
        _svc, _store, robot = _real_service(store=store)
        assert len(robot.debug_msgs) == 1
        msg = robot.debug_msgs[0]
        assert "forward=1.3000" in msg
        assert "lateral=0.7000" in msg

    def test_warn_message_names_the_section(self):
        # The empty-store warn message references the trim section name.
        # Assert the section token is carried; wording is prose, not pinned.
        _svc, _store, robot = _real_service()  # empty store -> warn branch
        assert len(robot.warn_msgs) == 1
        assert "motion-trim" in robot.warn_msgs[0]

    def test_persisted_scale_survives_into_a_second_service_instance(self):
        """The whole point of the class: store -> NEW service -> load -> apply.

        Service A writes a non-identity scale via the public API; a *second*
        MotionTrimService constructed against the SAME store must re-apply that
        persisted value through ``_load_stored_scales`` (run during B's
        ``__init__``) — not merely leave it sitting in ``store.data``. Asserting
        on B's ``get_axis_scale`` / ``scale_distance_m`` proves the persisted
        factor was read back and applied, so a regression in the load-path
        filtering would be caught here even though A's in-memory state is gone.
        """
        shared = _FakeStore()

        # Service A calibrates the forward axis (undershoot -> 30/25 = 1.2) and
        # also moves lateral, then is discarded.
        svc_a, _store_a, _robot_a = _real_service(store=shared)
        assert svc_a.calibrate_axis("forward", 0.30, 0.25) == pytest.approx(1.2)
        svc_a.set_axis_scale("lateral", 0.8)
        del svc_a  # A is gone; only the shared store carries the state.

        # Sanity: the persisted payload is what B will have to load.
        assert shared.data["motion-trim"]["forward"] == pytest.approx(1.2)
        assert shared.data["motion-trim"]["lateral"] == pytest.approx(0.8)

        # Service B is a FRESH instance over the SAME store. Its __init__ runs
        # _load_stored_scales, which must re-hydrate both axes from persistence.
        svc_b, _store_b, robot_b = _real_service(store=shared)

        # B exposes the persisted factors (load path applied them, not store.data).
        assert svc_b.get_axis_scale("forward") == pytest.approx(1.2)
        assert svc_b.get_axis_scale("lateral") == pytest.approx(0.8)
        # And actually USES them when scaling a commanded distance.
        assert svc_b.scale_distance_m("forward", 0.30) == pytest.approx(0.36)
        assert svc_b.scale_distance_m("lateral", 1.0) == pytest.approx(0.8)

        # B took the "loaded" branch (debug, not warn) and reported both axes.
        assert robot_b.debug_msgs and not robot_b.warn_msgs
        joined = " ".join(robot_b.debug_msgs)
        assert "forward=1.2000" in joined
        assert "lateral=0.8000" in joined
