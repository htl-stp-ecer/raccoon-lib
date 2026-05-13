"""End-to-end mission tests for ``custom_velocity`` against the simulator.

Proves that:
- The velocity function is called each cycle and its output is scaled and
  applied to the drive controller.
- The until condition terminates the step at the right time/distance.
- Zero velocity leaves the robot stationary.
- Spin-only command produces heading change without significant translation.

Each scenario is tested with three robot configurations:
- ``default``: idealized robot with no motor drag
- ``drumbot``: differential-drive robot with real calibration + drag
- ``packingbot``: mecanum-drive robot with real calibration + drag

Scenario logic lives in ``_custom_velocity_runner.py`` and runs in a
subprocess for the same reason as the drive mission tests — pybind11 /
mock-HAL singleton teardown is fragile at interpreter shutdown.

Skipped if the wheel was built without DRIVER_BUNDLE=mock.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

RUNNER = Path(__file__).parent / "_custom_velocity_runner.py"

ROBOT_CONFIGS = ["default", "drumbot", "packingbot"]
EXPECTED = {
    "default": {
        "forward_1s": (131.0, 50.0, 0.0),
        "forward_until_cm": (70.5, 50.0, 0.0),
        "spin_1s": (50.0, 50.0, -0.39),
        "zero_velocity": (50.0, 50.0, 0.0),
    },
    "drumbot": {
        "forward_1s": (81.7240, 57.1746, 0.44672),
        "forward_until_cm": (70.4869, 52.8968, 0.28276),
        "spin_1s": (49.3115, 51.3974, -2.20810),
        "zero_velocity": (50.0, 50.0, 0.0),
    },
    "packingbot": {
        "forward_1s": (78.4670, 50.1305, 0.02180),
        "forward_until_cm": (70.4972, 50.0316, 0.01570),
        "spin_1s": (50.3886, 50.0763, 1.74259),
        "zero_velocity": (50.0, 50.0, 0.0),
    },
}


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


def _run_runner(config_name: str) -> dict:
    """Spawn the runner subprocess and parse its results JSON."""
    env = os.environ.copy()
    env.setdefault("LIBSTP_LOG_LEVEL", "warn")
    env.setdefault("LIBSTP_NO_CALIBRATE", "1")
    env["LIBSTP_TIMING_ENABLED"] = "0"

    proc = subprocess.run(
        [sys.executable, str(RUNNER), "--config", config_name],
        capture_output=True,
        text=True,
        timeout=60,
        env=env,
        check=False,
    )
    if proc.returncode != 0:
        msg = (
            f"runner failed (exit={proc.returncode}, config={config_name})\n"
            f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
        )
        raise AssertionError(msg)

    for line in proc.stdout.splitlines():
        if line.startswith("RESULTS:"):
            return json.loads(line[len("RESULTS:") :])
    msg = (
        f"runner did not emit RESULTS line (config={config_name})\n"
        f"stdout: {proc.stdout}\nstderr: {proc.stderr}"
    )
    raise AssertionError(msg)


@pytest.fixture(scope="module", params=ROBOT_CONFIGS)
def results(request):
    """Run all scenarios for each robot config and cache the results."""
    return {"_config": request.param, **_run_runner(request.param)}


def test_forward_1s_moves_significantly(results):
    """Forward at 100% for 1 s — robot must travel a meaningful distance."""
    x, y, theta = results["forward_1s"]
    exp_x, exp_y, exp_theta = EXPECTED[results["_config"]]["forward_1s"]
    tol_x = 0.7 if results["_config"] == "default" else 0.3
    assert x == pytest.approx(exp_x, abs=tol_x)
    assert y == pytest.approx(exp_y, abs=0.15)
    assert theta == pytest.approx(exp_theta, abs=0.03)


def test_forward_until_cm_stops_near_target(results):
    """Forward until after_forward_cm(20) — should stop near 20 cm displacement."""
    x, y, theta = results["forward_until_cm"]
    exp_x, exp_y, exp_theta = EXPECTED[results["_config"]]["forward_until_cm"]
    tol_x = 0.4
    assert x == pytest.approx(exp_x, abs=tol_x)
    assert y == pytest.approx(exp_y, abs=0.15)
    assert theta == pytest.approx(exp_theta, abs=0.03)


def test_spin_rotates_without_translating(results):
    """Pure spin at 100% for 1 s — heading changes, position stays."""
    x, y, theta = results["spin_1s"]
    exp_x, exp_y, exp_theta = EXPECTED[results["_config"]]["spin_1s"]
    assert x == pytest.approx(exp_x, abs=0.15)
    assert y == pytest.approx(exp_y, abs=0.15)
    tol_theta = 0.05
    assert theta == pytest.approx(exp_theta, abs=tol_theta)


def test_zero_velocity_no_drift(results):
    """Zero velocity for 0.5 s — robot must not move."""
    x, y, theta = results["zero_velocity"]
    exp_x, exp_y, exp_theta = EXPECTED[results["_config"]]["zero_velocity"]
    assert x == pytest.approx(exp_x, abs=0.01)
    assert y == pytest.approx(exp_y, abs=0.01)
    assert theta == pytest.approx(exp_theta, abs=0.001)
