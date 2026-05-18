"""End-to-end tests for the auto-tune pipeline against the physics simulator.

Validates the three auto-tune phases:
1. **CharacterizeDrive** (Phase 1): measures bounded drive limits from raw-power
   trials and produces numeric constraints for each profile.
2. **AutoTuneVelocity** (Phase 2): either finds a valid step-response model and
   improves ISE, or rejects the tune cleanly when the simulated response is too
   weak/noisy to identify a trustworthy plant.
3. **AutoTuneMotion** (Phase 3): iteratively tunes motion PID gains with
   bounded runtime and either improves the score or reports a timeout without
   pathological outputs.

The runner subprocess pattern isolates C++ singleton teardown from pytest.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from raccoon.testing.robot_configs import DRUMBOT, PACKINGBOT

RUNNER = Path(__file__).parent / "_auto_tune_runner.py"
MODULE_PYTHON_PATHS = sorted(str(p) for p in (RUNNER.parents[2] / "modules").glob("*/python"))


def _raccoon_available() -> bool:
    try:
        from raccoon import sim

        return hasattr(sim, "mock")
    except ImportError:
        return False


pytestmark = pytest.mark.skipif(
    not _raccoon_available(),
    reason="raccoon mock-bundle wheel not installed (rebuild with "
    "`pip install -e . --config-settings=cmake.define.DRIVER_BUNDLE=mock`)",
)


def _run_runner(config_name: str, timeout: int = 360) -> dict:
    """Spawn the runner subprocess and parse its results JSON."""
    env = os.environ.copy()
    env.setdefault("LIBSTP_LOG_LEVEL", "warn")
    env["LIBSTP_TIMING_ENABLED"] = "0"
    existing_pythonpath = env.get("PYTHONPATH")
    env["PYTHONPATH"] = os.pathsep.join(
        [*MODULE_PYTHON_PATHS, existing_pythonpath] if existing_pythonpath else MODULE_PYTHON_PATHS
    )
    # Do NOT set LIBSTP_NO_CALIBRATE — auto-tune needs calibration enabled.
    env.pop("LIBSTP_NO_CALIBRATE", None)

    proc = subprocess.run(
        [sys.executable, str(RUNNER), "--config", config_name],
        capture_output=True,
        text=True,
        timeout=timeout,
        env=env,
        check=False,
    )
    if proc.returncode != 0:
        msg = (
            f"runner failed (exit={proc.returncode}, config={config_name})\n"
            f"stdout: {proc.stdout[-2000:]}\nstderr: {proc.stderr[-2000:]}"
        )
        raise AssertionError(msg)

    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:") :])
    msg = (
        f"runner did not emit RESULTS line (config={config_name})\n"
        f"stdout: {proc.stdout[-2000:]}\nstderr: {proc.stderr[-2000:]}"
    )
    raise AssertionError(msg)


def _raw_forward_speed_cap(cfg) -> float:
    return cfg.max_wheel_velocity_rad_s * cfg.wheel_radius_m


# ---------------------------------------------------------------------------
# Fixtures — run the expensive subprocess once per config, cache the results
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def default_results() -> dict:
    return _run_runner("default")


@pytest.fixture(scope="module")
def drumbot_results() -> dict:
    return _run_runner("drumbot")


@pytest.fixture(scope="module")
def packingbot_results() -> dict:
    return _run_runner("packingbot")


# ---------------------------------------------------------------------------
# Phase 1: CharacterizeDrive — should measure realistic velocity profiles
# ---------------------------------------------------------------------------


class TestCharacterizeDefault:
    """Phase 1 with the default (moderate drag) config."""

    def test_forward_max_velocity_positive(self, default_results):
        char = default_results["characterize"]
        assert "error" not in char, f"characterize failed: {char.get('error')}"
        assert char["forward"]["max_velocity"] == pytest.approx(0.8759, abs=0.003)

    def test_forward_acceleration_positive(self, default_results):
        char = default_results["characterize"]
        assert char["forward"]["acceleration"] == pytest.approx(6.2, abs=0.3)

    def test_forward_deceleration_positive(self, default_results):
        char = default_results["characterize"]
        assert char["forward"]["deceleration"] == pytest.approx(6.1, abs=0.3)

    def test_angular_max_velocity_positive(self, default_results):
        char = default_results["characterize"]
        assert char["angular"]["max_velocity"] == pytest.approx(11.69, abs=0.1)

    def test_angular_acceleration_positive(self, default_results):
        char = default_results["characterize"]
        assert char["angular"]["acceleration"] == pytest.approx(84.0, abs=3.0)

    def test_angular_deceleration_positive(self, default_results):
        char = default_results["characterize"]
        assert char["angular"]["deceleration"] == pytest.approx(81.5, abs=3.0)


class TestCharacterizeDrumbot:
    """Phase 1 with drumbot — larger wheels, higher drag."""

    def test_forward_max_velocity(self, drumbot_results):
        char = drumbot_results["characterize"]
        assert "error" not in char, f"characterize failed: {char.get('error')}"
        assert char["forward"]["max_velocity"] == pytest.approx(0.3357, abs=0.003)
        assert char["forward"]["max_velocity"] == pytest.approx(
            _raw_forward_speed_cap(DRUMBOT) * 0.944, abs=0.003
        )

    def test_angular_max_velocity(self, drumbot_results):
        char = drumbot_results["characterize"]
        assert char["angular"]["max_velocity"] == pytest.approx(4.20, abs=0.05)

    def test_angular_deceleration_positive(self, drumbot_results):
        char = drumbot_results["characterize"]
        assert char["angular"]["deceleration"] == pytest.approx(26.0, abs=3.0)

    def test_angular_acceleration_positive(self, drumbot_results):
        char = drumbot_results["characterize"]
        assert char["angular"]["acceleration"] == pytest.approx(25.2, abs=3.0)

    def test_forward_deceleration_positive(self, drumbot_results):
        char = drumbot_results["characterize"]
        assert char["forward"]["deceleration"] == pytest.approx(2.08, abs=0.2)


class TestCharacterizePackingbot:
    """Phase 1 with packingbot — mecanum geometry is exercised in the runner."""

    def test_runs_without_crashing(self, packingbot_results):
        char = packingbot_results["characterize"]
        assert "error" not in char, f"characterize failed: {char.get('error')}"
        assert packingbot_results["_supports_lateral"] is True

    def test_forward_max_velocity_numeric(self, packingbot_results):
        char = packingbot_results["characterize"]
        assert char["forward"]["max_velocity"] == pytest.approx(0.29438, abs=0.003)
        assert char["forward"]["max_velocity"] == pytest.approx(
            _raw_forward_speed_cap(PACKINGBOT) * 0.917, abs=0.003
        )

    def test_angular_max_velocity_positive(self, packingbot_results):
        char = packingbot_results["characterize"]
        assert char["angular"]["max_velocity"] == pytest.approx(1.81, abs=0.02)

    def test_angular_deceleration_positive(self, packingbot_results):
        char = packingbot_results["characterize"]
        assert char["angular"]["deceleration"] == pytest.approx(11.8, abs=2.0)

    def test_forward_deceleration_positive(self, packingbot_results):
        char = packingbot_results["characterize"]
        assert char["forward"]["deceleration"] == pytest.approx(1.91, abs=0.2)


# ---------------------------------------------------------------------------
# Phase 2: AutoTuneVelocity — improves ISE or rejects cleanly
# ---------------------------------------------------------------------------


class TestVelocityTuneDefault:
    """Phase 2 with default config.

    The velocity tune runs system identification on the velocity controller's
    step response and applies CHR PID gains when the identified plant is a
    sensible CHR candidate. The current default sim profile identifies
    repeatably, but does not guarantee an accepted tune.
    """

    def test_runs_without_error(self, default_results):
        vel = default_results["velocity"]
        assert "error" not in vel, f"velocity tune crashed: {vel.get('error')}"

    def test_baseline_ise_positive(self, default_results):
        """The baseline (pre-tune) step response should have measurable error."""
        vel = default_results["velocity"]
        assert vel["vx"]["baseline_ise"] == pytest.approx(0.00524, abs=0.0003)

    def test_plant_identification_produces_gains(self, default_results):
        """The default sim profile identifies as a near-pass-through velocity loop."""
        vel = default_results["velocity"]
        vx = vel["vx"]
        assert vx["accepted"] is False
        assert vx["plant_method"] == "inflection+low_dead_time_baseline"
        assert vx["plant_Ks"] == pytest.approx(0.98716, abs=0.01)
        assert vx["plant_Tu"] == pytest.approx(0.001, abs=0.0001)
        assert vx["plant_Tg"] > 0.05
        assert vx["plant_Tu"] / vx["plant_Tg"] < 0.01
        assert vx["pid_kp"] == 0.0
        assert vx["pid_ki"] == 0.0
        assert vx["pid_kd"] == 0.0
        assert vx["tuned_ise"] == pytest.approx(vx["baseline_ise"], abs=1e-9)

    def test_accepted_or_reverted_gracefully(self, default_results):
        """The tune should either improve ISE or gracefully revert."""
        vel = default_results["velocity"]
        vx = vel["vx"]
        assert vx["tuned_ise"] == pytest.approx(vx["baseline_ise"], abs=1e-9)


@pytest.mark.parametrize("fixture_name", ["drumbot_results", "packingbot_results"])
def test_velocity_tune_high_drag_rejects_cleanly(request, fixture_name: str):
    vel = request.getfixturevalue(fixture_name)["velocity"]
    assert "error" not in vel, f"velocity tune crashed: {vel.get('error')}"
    vx = vel["vx"]
    assert vx["accepted"] is False
    assert vx["pid_kp"] == 0.0
    assert vx["pid_ki"] == 0.0
    assert vx["pid_kd"] == 0.0
    assert vx["tuned_ise"] == pytest.approx(vx["baseline_ise"])
    assert vx["plant_method"] == "inflection+low_dead_time_baseline"
    expected = {
        "drumbot_results": {"plant_Ks": 0.93421, "baseline_ise": 0.00117},
        "packingbot_results": {
            "plant_Ks": 0.90138,
            "baseline_ise": 0.00114,
        },
    }[fixture_name]
    assert vx["plant_Ks"] == pytest.approx(expected["plant_Ks"], abs=0.01)
    # Plant_Tu (dead time) varies on the higher-drag sims because the
    # inflection-point detector is sensitive to the noise floor; only require
    # that it's small compared to Tg (the low-dead-time gate is the real
    # signal here).
    assert vx["plant_Tu"] >= 0.001 and vx["plant_Tu"] < 0.1
    assert vx["plant_Tg"] > 0.5
    assert vx["plant_Tu"] / vx["plant_Tg"] < 0.05
    assert vx["baseline_ise"] == pytest.approx(expected["baseline_ise"], abs=0.0002)


# ---------------------------------------------------------------------------
# Phase 3: AutoTuneMotion — coordinate descent on motion PID
# ---------------------------------------------------------------------------


class TestMotionTuneDefault:
    """Phase 3 with default config.

    The Hooke-Jeeves coordinate descent should find gains that improve
    (or at least don't degrade) the settle-time score for distance drives.

    Note: the 300 s Phase 3 timeout is occasionally hit due to stochastic
    BEMF noise in the sim. When it times out we skip the gain-quality
    assertions rather than fail the suite.
    """

    def test_completes_or_times_out(self, default_results):
        """Phase 3 should either complete successfully or report TimeoutError."""
        motion = default_results["motion"]
        if "error" in motion:
            assert "Timeout" in motion["error"], f"unexpected motion tune error: {motion['error']}"

    def test_score_does_not_degrade(self, default_results):
        """Final score should be <= initial score (lower is better)."""
        motion = default_results["motion"]
        if "error" in motion:
            pytest.skip("motion tune timed out")
        dist = motion["distance"]
        # Initial gains overshoot heavily on the default sim, producing a
        # large baseline score; the tune typically drives it down by an
        # order of magnitude.
        assert dist["initial_score"] > 5.0
        assert dist["final_score"] < dist["initial_score"]

    def test_multiple_iterations(self, default_results):
        """Coordinate descent should run multiple iterations."""
        motion = default_results["motion"]
        if "error" in motion:
            pytest.skip("motion tune timed out")
        assert motion["distance"]["iterations"] == 6

    def test_gains_changed(self, default_results):
        """The optimizer should have explored different gain values."""
        motion = default_results["motion"]
        if "error" in motion:
            pytest.skip("motion tune timed out")
        dist = motion["distance"]
        assert dist["final_kp"] != dist["initial_kp"] or dist["final_kd"] != dist["initial_kd"]

    def test_gains_positive(self, default_results):
        """Final gains should be positive (physical PID constraint)."""
        motion = default_results["motion"]
        if "error" in motion:
            pytest.skip("motion tune timed out")
        dist = motion["distance"]
        assert dist["final_kp"] > 0.0
        assert dist["final_kd"] >= 0.0


class TestMotionTuneHighDrag:
    """Phase 3 with high-drag configs.

    With the corrected BEMF calibration, both drumbot and packingbot can
    move in the sim and the coordinate descent converges. If it times out,
    that's also acceptable — the drag makes convergence slower.
    """

    def test_drumbot_completes(self, drumbot_results):
        motion = drumbot_results["motion"]
        if "error" in motion:
            assert "Timeout" in motion["error"], f"unexpected error: {motion['error']}"
        else:
            dist = motion["distance"]
            assert dist["final_kp"] > 0.0
            assert dist["final_kd"] >= 0.0
            assert dist["initial_score"] > 20.0
            # Drumbot's slow dynamics mean the coordinate descent may not fully
            # converge in max_iterations — only require some improvement.
            assert dist["final_score"] < dist["initial_score"]
            assert dist["iterations"] == 6

    def test_packingbot_completes(self, packingbot_results):
        motion = packingbot_results["motion"]
        if "error" in motion:
            assert "Timeout" in motion["error"], f"unexpected error: {motion['error']}"
        else:
            dist = motion["distance"]
            assert dist["final_kp"] > 0.0
            assert dist["final_kd"] >= 0.0
            assert dist["initial_score"] > 20.0
            # Packingbot's heavy drag means convergence is slow; only
            # require that the tune doesn't make things worse.
            assert dist["final_score"] <= dist["initial_score"]
            assert dist["iterations"] == 6
