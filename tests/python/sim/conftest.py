"""Pytest fixtures for the libstp sim Python tests.

These tests import the sim binding directly from the build tree so they work
in a dev loop without a full ``pip install``. Once the package is installed,
the ``raccoon.sim`` import path takes precedence and the direct build-dir
shim becomes a no-op fallback.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
BUILD_SIM_DIR = REPO_ROOT / "build" / "modules" / "libstp-sim"
SCENES_DIR = REPO_ROOT / "scenes"


def _import_sim():
    """Import the sim module, preferring the installed raccoon.sim path."""
    try:
        from raccoon import sim as _sim  # type: ignore[import-not-found]

        return _sim
    except ImportError:
        pass
    if BUILD_SIM_DIR.exists() and str(BUILD_SIM_DIR) not in sys.path:
        sys.path.insert(0, str(BUILD_SIM_DIR))
    try:
        import sim as _sim  # type: ignore[import-not-found]

        return _sim
    except ImportError:
        pytest.skip(
            "sim binding not built — run " "`cmake --build build --target libstp_sim` first"
        )


@pytest.fixture(scope="session")
def sim_module():
    return _import_sim()


@pytest.fixture
def default_robot(sim_module):
    r = sim_module.RobotConfig()
    r.width_cm = 18.0
    r.length_cm = 18.0
    r.wheel_radius_m = 0.03
    r.track_width_m = 0.15
    return r


@pytest.fixture
def default_motors(sim_module):
    m = sim_module.SimMotorMap()
    m.left_port = 0
    m.right_port = 1
    m.max_wheel_velocity_rad_s = 30.0
    m.motor_time_constant_sec = 0.01  # snappy — clean math for tests
    return m


@pytest.fixture
def empty_scene(sim_module):
    m = sim_module.WorldMap()
    m.load_ftmap(str(SCENES_DIR / "empty_table.ftmap"))
    return m


@pytest.fixture
def single_line_scene(sim_module):
    m = sim_module.WorldMap()
    m.load_ftmap(str(SCENES_DIR / "single_line.ftmap"))
    return m


@pytest.fixture
def wall_box_scene(sim_module):
    m = sim_module.WorldMap()
    m.load_ftmap(str(SCENES_DIR / "wall_box.ftmap"))
    return m


@pytest.fixture
def configured_world(sim_module, default_robot, default_motors, empty_scene):
    world = sim_module.SimWorld()
    world.configure(default_robot, default_motors)
    world.set_map(empty_scene)
    world.set_pose(sim_module.Pose2D(50.0, 50.0, 0.0))
    return world


def run(world, seconds: float, dt: float = 0.01) -> None:
    steps = round(seconds / dt)
    for _ in range(steps):
        world.tick(dt)
