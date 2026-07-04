"""``to_absolute()`` corrects injected odometry drift; the plain baseline doesn't.

The mock sim is driftless by construction (the dead-reckoned odometry equals the
ground truth), so a drift-correcting pass is invisible — it lands exactly where
the baseline does. This test makes the value of ``to_absolute`` observable by
INJECTING a per-axis odometry bias (``sim.mock.set_odometry_drift``, a SIM-ONLY
hook that perturbs ONLY the dead-reckoned odometry the relative controllers read,
never the physical pose) and modelling ABSOLUTE LANDMARK sightings by periodically
snapping the localization particle filter onto the ground truth
(``localization.observe``). See ``_to_absolute_drift_runner.py``.

The same short L-path runs three ways (each in its own subprocess, mirroring
``test_optimize_sideaction_equivalence``):

- ``ref``      — no drift → the intended (target) endpoint.
- ``baseline`` — plain ``seq()`` WITH drift → the relative controllers believe the
                 biased odometry, so the robot physically DRIFTS off target.
- ``absolute`` — ``optimize(...).to_absolute()`` WITH the SAME drift → the
                 ``GotoWaypoints`` regulate on the landmark-corrected localization,
                 so the robot stays on target.

The assertion is the demonstration: under identical drift, ``absolute`` ends much
closer to the intended target than ``baseline``.

Runs WALL-CLOCK (the localization worker integrates on the real clock; fast-time
freezes it). Thresholds are generous — the measured split is ~12 cm baseline vs
~4 cm absolute and is stable run-to-run to well under a cm.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_to_absolute_drift_runner.py"


def _raccoon_available() -> bool:
    try:
        from raccoon import sim

        return hasattr(sim, "mock") and hasattr(sim.mock, "set_odometry_drift")
    except ImportError:
        return False


pytestmark = pytest.mark.skipif(
    not _raccoon_available(),
    reason="raccoon mock-bundle wheel with set_odometry_drift not installed",
)


def _run(config: str) -> dict:
    env = os.environ.copy()
    env["RACCOON_SIM"] = "1"
    env.setdefault("LIBSTP_LOG_LEVEL", "error")
    env.setdefault("LIBSTP_NO_CALIBRATE", "1")
    env["LIBSTP_TIMING_ENABLED"] = "0"
    # Deliberately DO NOT set RACCOON_SIM_FASTTIME — the localization worker
    # thread integrates odometry on the real clock and freezes under fast time.
    env.pop("RACCOON_SIM_FASTTIME", None)
    proc = subprocess.run(
        [sys.executable, str(RUNNER), "--config", config],
        capture_output=True,
        text=True,
        timeout=120,
        env=env,
        check=False,
    )
    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:") :])[config]
    msg = (
        f"runner emitted no RESULTS for {config}\n"
        f"stdout:{proc.stdout[-800:]}\nstderr:{proc.stderr[-800:]}"
    )
    raise AssertionError(msg)


def _err_cm(a: list[float], b: list[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


@pytest.fixture(scope="module")
def runs() -> dict[str, dict]:
    return {c: _run(c) for c in ("ref", "baseline", "absolute")}


def test_all_three_complete(runs):
    for cfg in ("ref", "baseline", "absolute"):
        assert runs[cfg]["status"] == "completed", f"{cfg}: {runs[cfg]['status']}"


def test_baseline_drifts_off_target(runs):
    """The relative baseline closes its loop on the biased odometry, so its
    physical endpoint drifts well off the intended (no-drift) target."""
    err = _err_cm(runs["baseline"]["end_gt"], runs["ref"]["end_gt"])
    assert err > 8.0, f"expected baseline to drift >8cm under the injected bias, got {err:.1f}cm"


def test_absolute_corrects_the_drift(runs):
    """to_absolute regulates on the landmark-corrected localization, so under the
    SAME drift it stays close to the intended target — much closer than the
    baseline drifted."""
    base_err = _err_cm(runs["baseline"]["end_gt"], runs["ref"]["end_gt"])
    abs_err = _err_cm(runs["absolute"]["end_gt"], runs["ref"]["end_gt"])
    assert abs_err < 6.0, f"expected to_absolute within 6cm of target, got {abs_err:.1f}cm"
    assert abs_err < 0.6 * base_err, (
        f"expected to_absolute ({abs_err:.1f}cm) to be much closer than "
        f"baseline ({base_err:.1f}cm) under identical drift"
    )
