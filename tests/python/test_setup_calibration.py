"""Tests for the setup-time calibration session + finalizing gate.

The setup calibration replaces the old ``calibrate_distance`` (which rewrote the
odometry ``ticks_to_rad`` baseline). Distance error is now corrected purely on
the trim layer: :class:`CalibrationGate` assigns each axis' median
``collect_drive`` scale as the absolute trim scale via
``MotionTrimService.set_axis_scale``.

Per-sample the trim scale is ``odom / ground_truth`` (see
:pyattr:`DriveCalibrationSample.scale`). The internal odometry regulates the
drive, so when it over-reports a commanded distance under-shoots physically and
the scale is ``> 1`` to lengthen it back onto target (verified on hardware: a
``< 1`` scale drives the robot short). Because both quantities are measured over
the same physical drive, the scale is independent of the active scale, so it *is*
the absolute target scale and is SET, never composed. Composing it would multiply
the scale on every run and diverge geometrically (the robot drives further and
further the more you calibrate).
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
# 1. DriveCalibrationSample.scale convention                                   #
# --------------------------------------------------------------------------- #
@requires_libstp
class TestDriveSampleScale:
    def test_scale_is_odom_over_ground_truth(self):
        m = _session_module()
        # Odom over-reported (0.36 vs 0.30 actually travelled) -> lengthen the
        # next drive with a scale > 1.
        sample = m.DriveCalibrationSample(
            axis=m.CalibrationAxis.FORWARD,
            odom_distance_m=0.36,
            ground_truth_distance_m=0.30,
            source="calibration_board",
        )
        assert sample.scale == pytest.approx(0.36 / 0.30)
        assert sample.scale > 1.0

    def test_scale_is_positive_despite_board_sign_flip(self):
        m = _session_module()
        # 180° board mounting offset -> ground truth carries opposite sign.
        sample = m.DriveCalibrationSample(
            axis=m.CalibrationAxis.FORWARD,
            odom_distance_m=0.50,
            ground_truth_distance_m=-0.55,
            source="calibration_board",
        )
        assert sample.scale == pytest.approx(0.50 / 0.55)
        assert sample.scale > 0.0

    def test_scale_handles_zero_ground_truth(self):
        m = _session_module()
        sample = m.DriveCalibrationSample(
            axis=m.CalibrationAxis.LATERAL,
            odom_distance_m=0.20,
            ground_truth_distance_m=0.0,
            source="manual_entry",
        )
        assert sample.scale == pytest.approx(1.0)


# --------------------------------------------------------------------------- #
# 2. Session aggregation + bookkeeping                                         #
# --------------------------------------------------------------------------- #
@requires_libstp
class TestSessionAggregation:
    def _session(self):
        return _session_module().SetupCalibrationSession(_FakeRobot())

    def test_median_axis_scale_odd(self):
        m = _session_module()
        s = self._session()
        for truth in (0.30, 0.25, 0.40):  # scales 0.36/truth, unsorted
            s.add_drive_sample(
                m.DriveCalibrationSample(m.CalibrationAxis.FORWARD, 0.36, truth, "board")
            )
        scales = sorted(0.36 / t for t in (0.30, 0.25, 0.40))
        assert s.median_axis_scale(m.CalibrationAxis.FORWARD) == pytest.approx(scales[1])

    def test_median_axis_scale_even_averages_middle_two(self):
        m = _session_module()
        s = self._session()
        for truth in (0.30, 0.36, 0.40, 0.45):
            s.add_drive_sample(
                m.DriveCalibrationSample(m.CalibrationAxis.FORWARD, 0.36, truth, "board")
            )
        scales = sorted(0.36 / t for t in (0.30, 0.36, 0.40, 0.45))
        expected = (scales[1] + scales[2]) / 2.0
        assert s.median_axis_scale(m.CalibrationAxis.FORWARD) == pytest.approx(expected)

    def test_median_axis_scale_no_samples_is_identity(self):
        m = _session_module()
        s = self._session()
        assert s.median_axis_scale(m.CalibrationAxis.LATERAL) == pytest.approx(1.0)

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
# 3. Gate assignment contract: median scale SET absolutely, never composed.    #
# --------------------------------------------------------------------------- #
@requires_libstp
class TestGateAssignsAbsoluteScale:
    """The gate must ASSIGN the median scale absolutely, not compose.

    ``scale = odom / ground_truth`` is measured over the same physical drive, so
    it is independent of the currently-applied scale. Composing it
    (``S_new = S_current * scale``) would multiply the scale on *every* calibration
    run and diverge geometrically — the bug where the robot drives further and
    further the more often you calibrate. Mirrors
    ``CalibrationGate._finalize_drive_axes``:
    ``trim.set_axis_scale(axis, median_scale)``.
    """

    def test_median_scale_is_assigned_not_composed(self):
        m = _session_module()
        s = m.SetupCalibrationSession(_FakeRobot())
        trim = _make_trim_service()

        # A previously-applied scale must be REPLACED, not multiplied onto.
        trim.set_axis_scale("forward", 1.2)

        for truth in (0.30, 0.25, 0.40):
            s.add_drive_sample(
                m.DriveCalibrationSample(m.CalibrationAxis.FORWARD, 0.36, truth, "board")
            )
        median_scale = s.median_axis_scale(m.CalibrationAxis.FORWARD)

        trim.set_axis_scale("forward", median_scale)

        assert trim.get_axis_scale("forward") == pytest.approx(median_scale)

    def test_repeated_calibration_does_not_diverge(self):
        """Re-applying the same evidence must be idempotent (no runaway growth)."""
        m = _session_module()
        s = m.SetupCalibrationSession(_FakeRobot())
        trim = _make_trim_service()

        for truth in (0.30, 0.25, 0.40):
            s.add_drive_sample(
                m.DriveCalibrationSample(m.CalibrationAxis.FORWARD, 0.36, truth, "board")
            )
        median_scale = s.median_axis_scale(m.CalibrationAxis.FORWARD)

        # Calibrate three times in a row from the same (scale-invariant) scale.
        for _ in range(3):
            trim.set_axis_scale("forward", median_scale)

        # Absolute assignment converges immediately and stays put; composing
        # would have produced median_scale ** 3 here.
        assert trim.get_axis_scale("forward") == pytest.approx(median_scale)


# --------------------------------------------------------------------------- #
# 4. The gate uses set_axis_scale (absolute), never calibrate_axis (composing). #
# --------------------------------------------------------------------------- #
class TestGateAssignsScaleAbsolutely:
    def test_finalize_drive_axes_calls_set_axis_scale_not_calibrate_axis(self):
        source = _STEPS_PATH.read_text()
        # Isolate the trim-applying method.
        start = source.index("def _finalize_drive_axes")
        end = source.index("async def _run_drive_fallback")
        finalize = source[start:end]
        # Composing calibrate_axis would diverge on repeated calibration.
        assert "set_axis_scale" in finalize
        assert "calibrate_axis" not in finalize


# --------------------------------------------------------------------------- #
# 5. Missing-board behaviour: fall back to a manual measurement, never silently #
#    mark distance calibrated when nothing was trimmed.                         #
# --------------------------------------------------------------------------- #
class TestMissingBoardFallsBackToManualMeasurement:
    """When ``collect_drive`` ran without a calibration board the axis has no
    samples and the board cannot supply ground truth. The gate must still drive a
    measured fallback and ask the operator to measure it by hand — and it must NOT
    advertise distance as calibrated unless an axis was actually trimmed."""

    def _finalize_source(self) -> str:
        source = _STEPS_PATH.read_text()
        start = source.index("def _finalize_drive_axes")
        end = source.index("async def _run_drive_fallback")
        return source[start:end]

    def test_no_board_no_longer_rejects_axis(self):
        finalize = self._finalize_source()
        # The old behaviour bailed out with a "rejecting" warning when no board
        # was present. That short-circuit must be gone so the fallback runs.
        assert "rejecting" not in finalize.lower()
        assert "_run_drive_fallback" in finalize

    def test_finalize_returns_trimmed_count(self):
        finalize = self._finalize_source()
        assert "-> int" in finalize
        assert "trimmed += 1" in finalize
        assert "return trimmed" in finalize

    def test_fallback_prompts_manual_measurement_without_board(self):
        source = _STEPS_PATH.read_text()
        start = source.index("async def _run_drive_fallback")
        end = source.index("async def _collect_fallback_drive")
        fallback = source[start:end]
        # Without board ground truth the operator is prompted via the measure
        # screen, and that hand measurement is recorded as a manual sample.
        assert "DistanceMeasureScreen" in fallback
        assert "manual_entry" in fallback

    def test_distance_flag_is_gated_on_trimmed_axes(self):
        source = _STEPS_PATH.read_text()
        start = source.index("async def _execute_step")
        end = source.index("async def _finalize_drive_axes")
        execute = source[start:end]
        # set_distance_calibrated() must sit behind a truthiness check on the
        # trimmed-axis count, not run unconditionally after finishing the gate.
        flag_idx = execute.index("set_distance_calibrated()")
        guard_idx = execute.index("if trimmed_axes:")
        assert guard_idx < flag_idx


# --------------------------------------------------------------------------- #
# 6. The fallback drive announces itself before moving, and the drive distance #
#    is configurable via a cm count.                                           #
# --------------------------------------------------------------------------- #
class TestFallbackDriveAnnouncesAndIsConfigurable:
    def _fallback_source(self) -> str:
        source = _STEPS_PATH.read_text()
        start = source.index("async def _run_drive_fallback")
        end = source.index("async def _collect_fallback_drive")
        return source[start:end]

    def test_confirm_screen_precedes_the_drive(self):
        fallback = self._fallback_source()
        # The robot moves on its own — the operator must be told first and able to
        # skip, *before* the driving screen / motion starts.
        confirm_idx = fallback.index("self.confirm(")
        drive_idx = fallback.index("self.run_with_ui(")
        assert confirm_idx < drive_idx
        assert "if not proceed:" in fallback

    def test_fallback_drive_does_not_pin_absolute_heading(self):
        fallback = self._fallback_source()
        # The gate can run before any mark_heading_reference(); pinning an absolute
        # heading (heading=…) would raise "No heading reference set".
        assert "heading=" not in fallback
        assert "drive_forward(" in fallback
        assert "strafe_right(" in fallback

    def test_gate_accepts_a_cm_count(self):
        source = _STEPS_PATH.read_text()
        start = source.index("def calibration_gate(")
        end = source.index("return CalibrationGate(", start)
        factory = source[start:end]
        # A single fallback_cm overrides both per-axis fallback distances.
        assert "fallback_cm: float | None = None" in factory
        assert "forward_fallback_cm = float(fallback_cm)" in factory
        assert "lateral_fallback_cm = float(fallback_cm)" in factory


# --------------------------------------------------------------------------- #
# 7. collect_drive's manual_measurement flag: ask vs reject without a board.   #
# --------------------------------------------------------------------------- #
class TestCollectDriveManualMeasurementFlag:
    def _collect_source(self) -> str:
        source = _STEPS_PATH.read_text()
        start = source.index("class CollectDrive(")
        end = source.index("class CollectIrSet(")
        return source[start:end]

    def test_factory_exposes_the_flag(self):
        source = _STEPS_PATH.read_text()
        start = source.index("def collect_drive(")
        end = source.index("return CollectDrive(", start)
        factory = source[start:end]
        assert "manual_measurement: bool = True" in factory

    def test_flag_gates_reject_vs_prompt(self):
        collect = self._collect_source()
        # When manual_measurement is off, no board -> reject (return, no prompt).
        # When on, the operator is prompted via DistanceMeasureScreen.
        assert "if not self._manual_measurement:" in collect
        reject_idx = collect.index("if not self._manual_measurement:")
        prompt_idx = collect.index("DistanceMeasureScreen")
        assert reject_idx < prompt_idx
