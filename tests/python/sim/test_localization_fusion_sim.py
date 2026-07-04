"""End-to-end: continuous automatic line-sensor fusion keeps the particle filter
map-corrected under odometry drift — no resync, no ground truth.

Two subprocess runs of the SAME drifted drive across the game-table grid:
  - ``fusion``   : the ContinuousLocalizationFusion service runs in the background
  - ``nofusion`` : it does not (filter is pure dead-reckoning)

Asserts the service substantially cuts the localization error, and that without
it the filter just tracks the (biased) odometry. Wall-clock (the localization
worker integrates on the real clock; fast time freezes it)."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_localization_fusion_runner.py"
SCENE = Path(
    "/media/tobias/TobiasSSD/projects/Botball/competition/Ecer2026/cube-bot/"
    "config/2026-game-table__sim_drivable.ftmap"
)


def _available() -> bool:
    try:
        from raccoon import sim

        return hasattr(sim, "mock") and hasattr(sim.mock, "set_odometry_drift")
    except ImportError:
        return False


pytestmark = [
    pytest.mark.skipif(not _available(), reason="raccoon mock bundle not installed"),
    pytest.mark.skipif(not SCENE.exists(), reason="cube-bot game-table ftmap not present"),
]


def _run(mode: str) -> dict:
    env = os.environ.copy()
    env["RACCOON_SIM"] = "1"
    env.setdefault("LIBSTP_LOG_LEVEL", "error")
    env.setdefault("LIBSTP_NO_CALIBRATE", "1")
    env["LIBSTP_TIMING_ENABLED"] = "0"
    env.pop("RACCOON_SIM_FASTTIME", None)
    proc = subprocess.run(
        [sys.executable, str(RUNNER), "--mode", mode],
        capture_output=True,
        text=True,
        timeout=120,
        env=env,
        check=False,
    )
    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:") :])[mode]
    msg = f"no RESULTS for {mode}\nstdout:{proc.stdout[-800:]}\nstderr:{proc.stderr[-800:]}"
    raise AssertionError(msg)


@pytest.fixture(scope="module")
def runs() -> dict:
    return {"fusion": _run("fusion"), "nofusion": _run("nofusion")}


def test_drift_is_actually_injected(runs):
    """Sanity: the 25% odometry bias produces a real dead-reckoning error."""
    assert runs["nofusion"]["dr_err"] > 10.0


def test_without_fusion_filter_just_dead_reckons(runs):
    """No fusion → the filter only predicts from the biased odometry, so its
    error tracks the dead-reckoning error (no map correction)."""
    r = runs["nofusion"]
    assert abs(r["pf_err"] - r["dr_err"]) < 5.0


def test_fusion_corrects_the_drift(runs):
    """With the always-on fusion service the filter localizes against the map and
    ends much closer to ground truth than dead-reckoning — automatically, with no
    resync step."""
    r = runs["fusion"]
    assert r["pf_err"] < 0.7 * r["dr_err"], (
        f"fusion filter err {r['pf_err']:.1f}cm should be well under "
        f"dead-reckoning {r['dr_err']:.1f}cm"
    )
