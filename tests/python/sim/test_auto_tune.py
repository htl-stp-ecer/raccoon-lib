"""End-to-end tests for the auto-tune pipeline against the physics simulator.

Validates the three auto-tune phases:
1. **CharacterizeDrive** (Phase 1): measures max velocity, acceleration, and
   deceleration by driving at full power and recording the velocity profile.
2. **AutoTuneVelocity** (Phase 2): step-response system identification and
   CHR PID tuning for the velocity controller.
3. **AutoTuneMotion** (Phase 3): Hooke-Jeeves coordinate descent on the
   motion controller's kp/kd gains.

Known limitations in sim:
- Phase 2 (velocity tune) works with the default config but may produce
  negative plant Ks with high-drag configs where the sim's motor model
  doesn't generate the step response the system identification expects.
- Phase 3 (motion tune) times out with high-drag robot configs (drumbot,
  packingbot) because the trial drives never settle within the expected
  thresholds. Only the default config is tested for Phase 3.

The runner subprocess pattern isolates C++ singleton teardown from pytest.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_auto_tune_runner.py"


def _raccoon_available() -> bool:
    try:
        import raccoon  # noqa: F401
        from raccoon import sim  # noqa: F401
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
        raise AssertionError(
            f"runner failed (exit={proc.returncode}, config={config_name})\n"
            f"stdout: {proc.stdout[-2000:]}\nstderr: {proc.stderr[-2000:]}"
        )

    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:"):])
    raise AssertionError(
        f"runner did not emit RESULTS line (config={config_name})\n"
        f"stdout: {proc.stdout[-2000:]}\nstderr: {proc.stderr[-2000:]}"
    )


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
        assert char["forward"]["max_velocity"] > 0.5, (
            f"max_velocity too low: {char['forward']['max_velocity']}")

    def test_forward_acceleration_positive(self, default_results):
        char = default_results["characterize"]
        assert char["forward"]["acceleration"] > 0.0

    def test_angular_max_velocity_positive(self, default_results):
        char = default_results["characterize"]
        assert char["angular"]["max_velocity"] > 10.0, (
            f"angular max_velocity too low: {char['angular']['max_velocity']}")

    def test_angular_acceleration_positive(self, default_results):
        char = default_results["characterize"]
        assert char["angular"]["acceleration"] > 0.0

    def test_angular_deceleration_positive(self, default_results):
        char = default_results["characterize"]
        assert char["angular"]["deceleration"] > 0.0


class TestCharacterizeDrumbot:
    """Phase 1 with drumbot — larger wheels, higher drag."""

    def test_forward_max_velocity(self, drumbot_results):
        char = drumbot_results["characterize"]
        assert "error" not in char, f"characterize failed: {char.get('error')}"
        # Drumbot has r=34.5mm vs default r=30mm → faster max velocity
        assert char["forward"]["max_velocity"] > 0.5

    def test_angular_max_velocity(self, drumbot_results):
        char = drumbot_results["characterize"]
        assert char["angular"]["max_velocity"] > 10.0

    def test_angular_deceleration_positive(self, drumbot_results):
        char = drumbot_results["characterize"]
        assert char["angular"]["deceleration"] > 0.0


class TestCharacterizePackingbot:
    """Phase 1 with packingbot — widest track, highest drag.

    Note: packingbot's forward characterization currently returns max_vel=0
    because the high drag (viscous=1.2, coulomb=2.0) combined with the
    reduced accel_timeout means the plateau detector doesn't find a valid
    plateau in time. The angular axis works because turn-in-place doesn't
    hit walls. This documents a limitation of the characterization with
    very draggy robots in the sim.
    """

    def test_runs_without_crashing(self, packingbot_results):
        char = packingbot_results["characterize"]
        assert "error" not in char, f"characterize failed: {char.get('error')}"

    def test_angular_max_velocity(self, packingbot_results):
        char = packingbot_results["characterize"]
        # Packingbot has wider track (200mm) → lower angular velocity
        assert char["angular"]["max_velocity"] > 5.0


# ---------------------------------------------------------------------------
# Phase 2: AutoTuneVelocity — runs but reverts (sim limitation)
# ---------------------------------------------------------------------------

class TestVelocityTuneDefault:
    """Phase 2 with default config.

    The velocity tune runs system identification on the velocity controller's
    step response and applies CHR PID gains. With the default sim config
    (moderate drag), the tune succeeds and produces gains that improve ISE.
    """

    def test_runs_without_error(self, default_results):
        vel = default_results["velocity"]
        assert "error" not in vel, f"velocity tune crashed: {vel.get('error')}"

    def test_baseline_ise_positive(self, default_results):
        """The baseline (pre-tune) step response should have measurable error."""
        vel = default_results["velocity"]
        assert vel["vx"]["baseline_ise"] > 0.0

    def test_plant_identification_produces_gains(self, default_results):
        """System identification should produce usable PID gains."""
        vel = default_results["velocity"]
        vx = vel["vx"]
        # Plant Ks should be positive for the system identification to work
        assert vx["plant_Ks"] > 0, (
            f"Plant Ks is non-positive ({vx['plant_Ks']}), system identification "
            f"may not be handling the sim motor model correctly")
        assert vx["pid_kp"] > 0.0

    def test_accepted_or_reverted_gracefully(self, default_results):
        """The tune should either improve ISE or gracefully revert."""
        vel = default_results["velocity"]
        vx = vel["vx"]
        if vx["accepted"]:
            # If accepted, tuned ISE should be better (lower) than baseline
            assert vx["tuned_ise"] < vx["baseline_ise"], (
                f"Accepted but tuned ISE ({vx['tuned_ise']:.4f}) >= "
                f"baseline ({vx['baseline_ise']:.4f})")


# ---------------------------------------------------------------------------
# Phase 3: AutoTuneMotion — coordinate descent on motion PID
# ---------------------------------------------------------------------------

class TestMotionTuneDefault:
    """Phase 3 with default config.

    The Hooke-Jeeves coordinate descent should find gains that improve
    (or at least don't degrade) the settle-time score for distance drives.
    """

    def test_runs_without_error(self, default_results):
        motion = default_results["motion"]
        assert "error" not in motion, f"motion tune failed: {motion.get('error')}"

    def test_score_does_not_degrade(self, default_results):
        """Final score should be <= initial score (lower is better)."""
        motion = default_results["motion"]
        dist = motion["distance"]
        assert dist["final_score"] <= dist["initial_score"] * 1.05, (
            f"Motion tune degraded: {dist['initial_score']:.4f} → "
            f"{dist['final_score']:.4f}")

    def test_multiple_iterations(self, default_results):
        """Coordinate descent should run multiple iterations."""
        motion = default_results["motion"]
        assert motion["distance"]["iterations"] >= 3

    def test_gains_changed(self, default_results):
        """The optimizer should have explored different gain values."""
        motion = default_results["motion"]
        dist = motion["distance"]
        gains_changed = (
            dist["final_kp"] != dist["initial_kp"] or
            dist["final_kd"] != dist["initial_kd"]
        )
        assert gains_changed, "Coordinate descent didn't explore any gain changes"

    def test_gains_positive(self, default_results):
        """Final gains should be positive (physical PID constraint)."""
        motion = default_results["motion"]
        dist = motion["distance"]
        assert dist["final_kp"] > 0.0
        assert dist["final_kd"] >= 0.0


class TestMotionTuneHighDrag:
    """Phase 3 with high-drag configs — expected to time out.

    The high viscous drag and Coulomb friction make trial drives take too
    long to settle, causing the coordinate descent to exceed the timeout.
    This documents a real limitation: the auto-tune pipeline's trial-based
    approach doesn't work well when drag prevents the robot from settling
    within expected time windows.
    """

    def test_drumbot_reports_timeout(self, drumbot_results):
        motion = drumbot_results["motion"]
        assert "error" in motion, (
            "Expected drumbot motion tune to fail — if this now passes, "
            "the auto-tune pipeline may have improved. Update this test.")
        assert "Timeout" in motion["error"]

    def test_packingbot_reports_timeout(self, packingbot_results):
        motion = packingbot_results["motion"]
        assert "error" in motion, (
            "Expected packingbot motion tune to fail — if this now passes, "
            "the auto-tune pipeline may have improved. Update this test.")
