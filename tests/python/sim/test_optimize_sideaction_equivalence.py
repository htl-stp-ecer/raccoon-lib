"""Baseline-vs-optimized side-action equivalence (headless mock sim).

Verifies that wrapping a mission in ``optimize(...)`` does not change WHICH side
actions run, in WHAT order, or WHETHER they complete — and, for the
geometry-preserving absolute passes, that the run ends at the same pose.

The mission (built in ``_sideaction_compare_runner.py``) interleaves forward
drives with probe side actions, including one ``parallel()`` branch that is
still running when its motion spine ends. Each probe records a ``start`` and a
``done`` event, so a dropped/cancelled branch (the bug fixed in
``executor.py``'s path-end ephemeral join) shows up as a missing ``done``.

Each config runs in its OWN subprocess (the mock C++ bundle races its teardown
across repeated ``use_scene`` cycles), mirroring ``test_path_optimizer.py``.

The ``B_branch`` parallel branch carries a ``settle`` so it is still running
when its spine ends — the case that used to (a) get cancelled at path end and
(b) make the warm-merged relative passes coast past the leg. Both are fixed in
``executor.py`` (path-end join + lazy ephemeral join that doesn't freeze the
control loop), so endpoints now match baseline on every config.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_sideaction_compare_runner.py"
ORDER = ["A_inline", "B_branch", "C_inline"]


def _raccoon_available() -> bool:
    try:
        from raccoon import sim

        return hasattr(sim, "mock")
    except ImportError:
        return False


pytestmark = pytest.mark.skipif(
    not _raccoon_available(),
    reason="raccoon mock-bundle wheel not installed",
)


def _run(config: str) -> dict:
    env = os.environ.copy()
    env["RACCOON_SIM"] = "1"
    env.setdefault("LIBSTP_LOG_LEVEL", "error")
    env.setdefault("LIBSTP_NO_CALIBRATE", "1")
    env["LIBSTP_TIMING_ENABLED"] = "0"
    proc = subprocess.run(
        [sys.executable, str(RUNNER), "--config", config],
        capture_output=True,
        text=True,
        timeout=90,
        env=env,
        check=False,
    )
    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:") :])[config]
    msg = f"runner emitted no RESULTS for {config}\nstdout:{proc.stdout[-500:]}\nstderr:{proc.stderr[-500:]}"
    raise AssertionError(msg)


def _starts(result: dict) -> list[str]:
    return [e[1] for e in result["events"] if e[0] == "start"]


def _dones(result: dict) -> set[str]:
    return {e[1] for e in result["events"] if e[0] == "done"}


# Run each config once per module; the matrix is small but each run is a full
# mock-sim subprocess, so cache them.
@pytest.fixture(scope="module")
def runs() -> dict[str, dict]:
    configs = ["baseline", "merge", "cut_corners", "absolute", "splinify", "absolute_splinify"]
    return {c: _run(c) for c in configs}


def test_baseline_fires_all_probes_in_order(runs):
    base = runs["baseline"]
    assert base["status"] == "completed"
    assert _starts(base) == ORDER
    assert _dones(base) == set(ORDER)


@pytest.mark.parametrize(
    "config", ["merge", "cut_corners", "absolute", "splinify", "absolute_splinify"]
)
def test_side_action_order_preserved(runs, config):
    """optimize() fires the same side actions in the same order as baseline."""
    assert runs[config]["status"] == "completed", runs[config]["status"]
    assert _starts(runs[config]) == ORDER


@pytest.mark.parametrize(
    "config", ["merge", "cut_corners", "absolute", "splinify", "absolute_splinify"]
)
def test_all_side_actions_complete(runs, config):
    """Every side action runs to completion — none is dropped or cancelled.

    This is the regression guard for the path-end ephemeral join: the
    ``B_branch`` parallel branch is still running when its spine ends, and under
    ``absolute``/``splinify`` the spine collapses to an inline move with no
    trailing segment, so without the join the branch would be cancelled and its
    ``done`` event would be missing.
    """
    assert _dones(runs[config]) == set(ORDER)


# Geometry-preserving passes (merge / to_absolute) must land almost exactly on
# the baseline endpoint; corner-cut and spline ride a different curve so they
# only need to land in the same neighbourhood.
_ENDPOINT_TOL_CM = {
    "merge": 8.0,
    "absolute": 8.0,
    "absolute_splinify": 10.0,
    "cut_corners": 14.0,
    "splinify": 14.0,
}


@pytest.mark.parametrize(
    "config", ["merge", "cut_corners", "absolute", "splinify", "absolute_splinify"]
)
def test_endpoint_matches_baseline(runs, config):
    """optimize() ends where baseline does — a SLOW parallel branch no longer
    makes the warm-merged relative passes coast past the leg (overshoot fix)."""
    b = runs["baseline"]["end"]
    e = runs[config]["end"]
    d = math.hypot(e[0] - b[0], e[1] - b[1])
    assert d < _ENDPOINT_TOL_CM[config], f"{config} ended {d:.1f} cm from baseline ({e} vs {b})"
