"""Smoke test for raccoon.localization.Localization.enable_recording.

Exercises the Python surface: construct a Localization with a real (mock-
driver-backed) odometry, turn on recording to a tempfile, let a handful of
ticks happen, then verify the JSONL file parses cleanly and contains the
expected header + frame shape. The C++ recorder is covered by
tests/cpp/localization/test_recorder.cpp — this file just guards the binding
glue and the env-var auto-wire helper.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

from raccoon.foundation import Pose
from raccoon.hal import Motor, platform
from raccoon.kinematics_differential import DifferentialKinematics
from raccoon.localization import (
    Localization,
    LocalizationConfig,
    Observation,
    _auto_enable_recording,
)


def _make_odom():
    """Build a platform-canonical odometry instance.

    The mock driver bundle wires an in-process SimWorld-backed odometry; this
    is the same path GenericRobot.odometry uses under tests.
    """
    left = Motor(port=0)
    right = Motor(port=1)
    kin = DifferentialKinematics(left, right, wheelbase=0.2, wheel_radius=0.04)
    return platform.Platform.create_odometry(kin)


def _settle(seconds: float = 0.2) -> None:
    # Tick period is 5 ms; this gives ~40 ticks. Enough for downsampled + forced
    # frames to appear.
    time.sleep(seconds)


def test_enable_recording_writes_parseable_jsonl(tmp_path: Path) -> None:
    out = tmp_path / "rec.jsonl"
    loc = Localization(_make_odom(), LocalizationConfig(tick_period_ms=5, particle_count=16))
    try:
        assert loc.enable_recording(str(out), record_hz=50.0, tick_hz=200.0)
        loc.observe(Observation(pose=Pose(), sigma=(0.02, 0.02, 0.01)))
        _settle()
    finally:
        loc.stop()
        del loc

    assert out.exists()
    lines = out.read_text().splitlines()
    assert lines, "expected at least the header line"
    header = json.loads(lines[0])
    assert header["kind"] == "header"
    assert header["format_version"] == 1
    assert header["units"]["position"] == "m"
    assert header["units"]["heading"] == "rad"
    assert header["tick_hz"] == pytest.approx(200.0)
    assert header["record_hz"] == pytest.approx(50.0)

    for raw in lines[1:]:
        frame = json.loads(raw)
        assert frame["kind"] == "frame"
        assert len(frame["pose"]) == 3
        assert len(frame["sigma"]) == 3
        for particle in frame["particles"]:
            assert len(particle) == 4


def test_enable_recording_returns_false_on_bad_path() -> None:
    bad = "/proc/self/does-not-exist/recording.jsonl"
    loc = Localization(_make_odom(), LocalizationConfig(tick_period_ms=5))
    try:
        assert loc.enable_recording(bad) is False
    finally:
        loc.stop()


def test_auto_enable_recording_env_off(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("LIBSTP_RECORD_LOCALIZATION", raising=False)
    monkeypatch.delenv("LIBSTP_RECORDING_PATH", raising=False)
    loc = Localization(_make_odom(), LocalizationConfig(tick_period_ms=5))
    try:
        assert _auto_enable_recording(loc) is False
    finally:
        loc.stop()


def test_auto_enable_recording_env_on(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    out = tmp_path / "auto.jsonl"
    monkeypatch.setenv("LIBSTP_RECORD_LOCALIZATION", "1")
    monkeypatch.setenv("LIBSTP_RECORDING_PATH", str(out))
    monkeypatch.setenv("LIBSTP_RECORDING_HZ", "25.0")
    loc = Localization(_make_odom(), LocalizationConfig(tick_period_ms=5))
    try:
        assert _auto_enable_recording(loc) is True
        _settle(seconds=0.1)
    finally:
        loc.stop()
        del loc
    assert out.exists()
    header = json.loads(out.read_text().splitlines()[0])
    assert header["record_hz"] == pytest.approx(25.0)
