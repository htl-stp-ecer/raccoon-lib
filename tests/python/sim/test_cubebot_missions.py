"""Integration: run the cube-bot mission chain headless in the sim.

Exercises the FULL stack — real Drive/odometry/localization + scene-driven line
sensors (after the SimWorld line-sensor polarity fix) — on the cube-bot's own
missions from the real start pose on the 2026 game-table ftmap.

This is a SLOW, environment-coupled integration test (it imports the cube-bot
project and runs many motion steps to a timeout), so it is gated behind
``RUN_SIM_CHAIN=1`` and skipped by default. Run it explicitly:

    RUN_SIM_CHAIN=1 .venv-test/bin/python -m pytest \
        tests/python/sim/test_cubebot_missions.py -q

It asserts the invariants that matter for "does the sim work", not full mission
completion (several missions still time out pending per-mission sensor-fidelity
tuning):
  * the harness runs the whole chain without crashing;
  * the line-sensor fix holds — M007 no longer drives straight off the top edge
    (the pre-fix signature was the robot ploughing through every line);
  * no mission flies off the table;
  * the simplest missions (M020, M090) actually complete.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
CUBE = "/media/tobias/TobiasSSD/projects/Botball/competition/Ecer2026/cube-bot"
START = (42.8, 25.63, math.radians(90))


def _raccoon_available() -> bool:
    try:
        from raccoon import sim

        return hasattr(sim, "mock")
    except ImportError:
        return False


pytestmark = [
    pytest.mark.skipif(not _raccoon_available(), reason="mock bundle not installed"),
    pytest.mark.skipif(
        os.environ.get("RUN_SIM_CHAIN") != "1",
        reason="slow sim-chain integration test — set RUN_SIM_CHAIN=1 to run",
    ),
    pytest.mark.skipif(not Path(CUBE).is_dir(), reason="cube-bot project not present"),
]


@pytest.fixture(scope="module")
def chain_result():
    """Run the chain once and return its trajectory result dict."""
    os.environ.setdefault("LIBSTP_LOG_LEVEL", "warn")
    os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
    os.environ["LIBSTP_TIMING_ENABLED"] = "0"
    sys.path.insert(0, str(REPO_ROOT / "tools"))
    sys.path.insert(0, CUBE)
    os.chdir(CUBE)
    import asyncio

    import sim_run_chain as scr  # type: ignore[import-not-found]

    scene = Path(CUBE) / "config" / "2026-game-table.ftmap"
    return asyncio.run(scr._simulate(Path(CUBE), scene, START, timeout=45.0))


def _seg(result, name_sub):
    for i, s in enumerate(result["segments"]):
        if name_sub in s["name"]:
            pts = [t for t in result["traj"] if t["mi"] == i]
            return s, pts
    raise AssertionError(f"no mission matching {name_sub}")


def test_chain_runs_without_crash(chain_result):
    assert chain_result["traj"], "no trajectory was recorded"
    assert chain_result["segments"], "no missions ran"


def test_line_sensor_fix_m007_does_not_run_off_top(chain_result):
    # Pre-fix signature: M007 drove straight through every line to the top edge
    # (~y=105). With sensors working it stops/turns well before that.
    _s, pts = _seg(chain_result, "M007")
    assert pts, "M007 produced no trajectory"
    assert max(p["y"] for p in pts) < 95.0, "M007 still ran off the top — sensor fix regressed"


def test_no_mission_leaves_the_table(chain_result):
    for i, s in enumerate(chain_result["segments"]):
        for p in (t for t in chain_result["traj"] if t["mi"] == i):
            assert -10 <= p["x"] <= 246, f"{s['name']} left table in x: {p['x']:.0f}"
            assert -10 <= p["y"] <= 116, f"{s['name']} left table in y: {p['y']:.0f}"


@pytest.mark.parametrize("name", ["M020", "M090"])
def test_simple_missions_complete(chain_result, name):
    s, _pts = _seg(chain_result, name)
    assert s["status"] == "completed", f"{name} did not complete: {s['status']}"
