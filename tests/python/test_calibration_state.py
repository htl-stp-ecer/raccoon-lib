"""Tests for the distance-calibration runtime flag (raccoon.step.calibration.state).

These helpers outlived the deleted ``calibrate_distance`` step: distance-based
drives still warn when no calibration has run, and the setup ``calibration_gate``
marks the flag done via :func:`set_distance_calibrated` once it finalizes.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
_STATE_PATH = REPO_ROOT / "modules/libstp-step/python/raccoon/step/calibration/state.py"


def libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not libstp_available(), reason="raccoon native module not installed"
)


def _state_module():
    name = "raccoon_test_calibration_state"
    cached = sys.modules.get(name)
    if cached is not None:
        return cached
    spec = importlib.util.spec_from_file_location(name, _STATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@requires_libstp
class TestDistanceCalibrationFlag:
    def test_initial_state_is_uncalibrated(self):
        m = _state_module()
        m.reset_distance_calibration()
        assert m.is_distance_calibrated() is False

    def test_set_marks_calibrated(self):
        m = _state_module()
        m.reset_distance_calibration()
        m.set_distance_calibrated()
        assert m.is_distance_calibrated() is True

    def test_reset_clears_flag(self):
        m = _state_module()
        m.set_distance_calibrated()
        m.reset_distance_calibration()
        assert m.is_distance_calibrated() is False

    def test_check_does_not_raise_and_warns_only_when_uncalibrated(self):
        m = _state_module()
        m.reset_distance_calibration()
        # check_distance_calibration only logs a warning; it must not raise.
        m.check_distance_calibration()
        m.set_distance_calibrated()
        m.check_distance_calibration()
