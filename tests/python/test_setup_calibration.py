"""Tests for the setup-time calibration session + finalizing gate.

The setup calibration replaces the old ``calibrate_distance`` (which rewrote the
odometry ``ticks_to_rad`` baseline). Distance error is now corrected purely on
the trim layer: :class:`CalibrationGate` folds each ``collect_drive`` sample into
``MotionTrimService.calibrate_axis`` — the *composing* method, so an
already-applied scale is preserved instead of overwritten.

Per-sample the trim factor is ``odom / ground_truth`` (see
:pyattr:`DriveCalibrationSample.factor`): when the robot physically travelled
less than the internal odometry believed, the factor is ``> 1`` and the next
drive is lengthened. The gate aggregates samples by median and applies the
result via ``calibrate_axis(requested=median_factor, measured=1.0)`` — never via
``set_axis_scale`` directly.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
_SESSION_PATH = REPO_ROOT / "modules/libstp-step/python/raccoon/step/calibration/setup/session.py"
_STEPS_PATH = REPO_ROOT / "modules/libstp-step/python/raccoon/step/calibration/setup/steps.py"
_TRIM_PATH = REPO_ROOT / "modules/libstp-motion/python/raccoon/step/motion/_motion_trim.py"


def libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not libstp_available(), reason="raccoon native module not installed"
)


def _load_module(name: str, path: Path):
    # Cache by name: re-loading would mint a fresh CalibrationAxis Enum whose
    # members compare unequal to a previously loaded one, breaking dict lookups.
    cached = sys.modules.get(name)
    if cached is not None:
        return cached
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Register before exec so @dataclass can resolve string annotations via
    # sys.modules[cls.__module__] (PEP 563 / `from __future__ import annotations`).
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# --------------------------------------------------------------------------- #
# Test doubles                                                                 #
# --------------------------------------------------------------------------- #
class _FakeStore:
    def __init__(self) -> None:
        self.data: dict[str, dict] = {}

    def load(self, section: str) -> dict:
        return dict(self.data.get(section, {}))

    def store(self, section: str, payload: dict) -> None:
        self.data[section] = dict(payload)


class _FakeRobot:
    def debug(self, *_a, **_k) -> None:
        pass

    def warn(self, *_a, **_k) -> None:
        pass


def _session_module():
    # session.py only needs raccoon.hal (OdometrySource) from the installed wheel.
    return _load_module("raccoon_test_setup_session", _SESSION_PATH)


def _make_trim_service():
    module = _load_module("raccoon_test_setup_trim", _TRIM_PATH)
    module.MotionTrimService._make_store = staticmethod(_FakeStore)  # type: ignore[attr-defined]
    return module.MotionTrimService(_FakeRobot())


# --------------------------------------------------------------------------- #
# 1. DriveCalibrationSample.factor convention                                  #
# --------------------------------------------------------------------------- #
@requires_libstp
class TestDriveSampleFactor:
    def test_factor_is_odom_over_ground_truth(self):
        m = _session_module()
        sample = m.DriveCalibrationSample(
            axis=m.CalibrationAxis.FORWARD,
            odom_distance_m=0.36,
            ground_truth_distance_m=0.30,
            source="calibration_board",
        )
        # odom believed 0.36 but only 0.30 travelled -> lengthen next drive.
        assert sample.factor == pytest.approx(0.36 / 0.30)
        assert sample.factor > 1.0

    def test_factor_is_positive_despite_board_sign_flip(self):
        m = _session_module()
        # 180° board mounting offset -> ground truth carries opposite sign.
        sample = m.DriveCalibrationSample(
            axis=m.CalibrationAxis.FORWARD,
            odom_distance_m=0.50,
            ground_truth_distance_m=-0.55,
            source="calibration_board",
        )
        assert sample.factor == pytest.approx(0.50 / 0.55)
        assert sample.factor > 0.0

    def test_factor_handles_zero_ground_truth(self):
        m = _session_module()
        sample = m.DriveCalibrationSample(
            axis=m.CalibrationAxis.LATERAL,
            odom_distance_m=0.20,
            ground_truth_distance_m=0.0,
            source="manual_entry",
        )
        assert sample.factor == pytest.approx(1.0)


# --------------------------------------------------------------------------- #
# 2. Session aggregation + bookkeeping                                         #
# --------------------------------------------------------------------------- #
@requires_libstp
class TestSessionAggregation:
    def _session(self):
        return _session_module().SetupCalibrationSession(_FakeRobot())

    def test_median_axis_factor_odd(self):
        m = _session_module()
        s = self._session()
        for truth in (0.30, 0.25, 0.40):  # factors 0.36/truth, unsorted
            s.add_drive_sample(
                m.DriveCalibrationSample(m.CalibrationAxis.FORWARD, 0.36, truth, "board")
            )
        factors = sorted(0.36 / t for t in (0.30, 0.25, 0.40))
        assert s.median_axis_factor(m.CalibrationAxis.FORWARD) == pytest.approx(factors[1])

    def test_median_axis_factor_even_averages_middle_two(self):
        m = _session_module()
        s = self._session()
        for truth in (0.30, 0.36, 0.40, 0.45):
            s.add_drive_sample(
                m.DriveCalibrationSample(m.CalibrationAxis.FORWARD, 0.36, truth, "board")
            )
        factors = sorted(0.36 / t for t in (0.30, 0.36, 0.40, 0.45))
        expected = (factors[1] + factors[2]) / 2.0
        assert s.median_axis_factor(m.CalibrationAxis.FORWARD) == pytest.approx(expected)

    def test_median_axis_factor_no_samples_is_identity(self):
        m = _session_module()
        s = self._session()
        assert s.median_axis_factor(m.CalibrationAxis.LATERAL) == pytest.approx(1.0)

    def test_axes_to_finalize_defaults_to_required_sorted(self):
        m = _session_module()
        s = self._session()
        s.require_axis(m.CalibrationAxis.LATERAL)
        s.require_axis(m.CalibrationAxis.FORWARD)
        assert s.axes_to_finalize(None) == [
            m.CalibrationAxis.FORWARD,
            m.CalibrationAxis.LATERAL,
        ]
        assert s.axes_to_finalize([m.CalibrationAxis.LATERAL]) == [m.CalibrationAxis.LATERAL]

    def test_ir_sets_to_finalize_defaults_to_required_sorted(self):
        s = self._session()
        s.require_ir_set("upper")
        s.require_ir_set("default")
        assert s.ir_sets_to_finalize(None) == ["default", "upper"]
        assert s.ir_sets_to_finalize(["upper"]) == ["upper"]

    def test_adding_evidence_marks_gate_pending(self):
        m = _session_module()
        s = self._session()
        s.finish_gate()
        assert s.gate_completed is True
        s.add_drive_sample(m.DriveCalibrationSample(m.CalibrationAxis.FORWARD, 0.30, 0.30, "board"))
        assert s.gate_completed is False


@requires_libstp
class TestIrCalibrationSet:
    def test_has_minimum_samples(self):
        m = _session_module()
        ir = m.IrCalibrationSet()
        assert ir.has_minimum_samples(5) is False
        ir.add(0, [1.0, 2.0, 3.0])
        ir.add(1, [1.0, 2.0])
        assert ir.has_minimum_samples(3) is False  # port 1 short
        ir.add(1, [3.0])
        assert ir.has_minimum_samples(3) is True


# --------------------------------------------------------------------------- #
# 3. Gate composition contract: median factor folded via calibrate_axis        #
# --------------------------------------------------------------------------- #
@requires_libstp
class TestGateCompositionContract:
    """The value the gate stores must COMPOSE with an already-applied scale.

    Mirrors ``CalibrationGate._finalize_drive_axes``:
    ``trim.calibrate_axis(axis, requested=median_factor, measured=1.0)``.
    """

    def test_median_factor_composes_onto_existing_scale(self):
        m = _session_module()
        s = m.SetupCalibrationSession(_FakeRobot())
        trim = _make_trim_service()

        current = 1.2
        trim.set_axis_scale("forward", current)

        for truth in (0.30, 0.25, 0.40):
            s.add_drive_sample(
                m.DriveCalibrationSample(m.CalibrationAxis.FORWARD, 0.36, truth, "board")
            )
        median_factor = s.median_axis_factor(m.CalibrationAxis.FORWARD)

        trim.calibrate_axis("forward", requested_m=median_factor, measured_m=1.0)

        assert trim.get_axis_scale("forward") == pytest.approx(current * median_factor)
        # ... and is NOT the overwrite the old code did:
        assert trim.get_axis_scale("forward") != pytest.approx(median_factor)


# --------------------------------------------------------------------------- #
# 4. The gate uses ONLY calibrate_axis to set the trim (no set_axis_scale).     #
# --------------------------------------------------------------------------- #
class TestGateUsesOnlyCalibrateAxis:
    def test_finalize_drive_axes_calls_calibrate_axis_not_set_axis_scale(self):
        source = _STEPS_PATH.read_text()
        # Isolate the trim-applying method.
        start = source.index("def _finalize_drive_axes")
        end = source.index("async def _run_drive_fallback")
        finalize = source[start:end]
        assert "calibrate_axis" in finalize
        assert "set_axis_scale" not in finalize
