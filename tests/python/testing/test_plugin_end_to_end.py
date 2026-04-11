"""End-to-end test: spawn pytest in a synthetic raccoon project.

This proves the full plugin flow against a real pytest invocation:

- A tmp directory is scaffolded to look like a toolchain-generated project
  (``raccoon.project.yml`` + ``src/hardware/robot.py``).
- A test file that uses the ``robot`` + ``scene`` + ``run_step`` fixtures is
  written into ``tests/``.
- Pytest is run as a subprocess from the scaffold root so the plugin's
  auto-discovery (walking upward from cwd) and project-root sys.path
  injection are exercised exactly as a user would hit them.

Runs in a subprocess so any pybind11 / mock-HAL teardown quirks stay
isolated from this outer pytest session — same pattern as
``test_drive_mission.py``.
"""
from __future__ import annotations

import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest


def _sim_available() -> bool:
    try:
        from raccoon import sim as _sim  # noqa: F401
        return hasattr(_sim, "mock")
    except ImportError:
        return False


pytestmark = pytest.mark.skipif(
    not _sim_available(),
    reason="raccoon mock-bundle wheel not installed",
)


PROJECT_YML = """\
name: fake_project
uuid: 00000000-0000-0000-0000-000000000000

robot:
  drive:
    kinematics:
      type: differential
      wheel_radius: 0.03
      wheelbase: 0.15
      left_motor: left_motor
      right_motor: right_motor
  motion_pid:
    linear:
      max_velocity: 0.8
      acceleration: 1.5
      deceleration: 1.5
    lateral:
      max_velocity: 0.5
      acceleration: 1.0
      deceleration: 1.0
    angular:
      max_velocity: 6.0
      acceleration: 12.0
      deceleration: 12.0
  physical:
    width_cm: 18.0
    length_cm: 18.0

definitions:
  left_motor:
    type: Motor
    port: 0
    inverted: false
  right_motor:
    type: Motor
    port: 1
    inverted: false
  imu:
    type: IMU
  button:
    type: DigitalSensor
    port: 10
"""


ROBOT_PY = '''\
"""Synthetic robot module — mimics what the toolchain generates."""
from types import SimpleNamespace

from raccoon.hal import IMU, Motor
from raccoon.kinematics_differential import DifferentialKinematics
from raccoon.drive import Drive, ChassisVelocityControlConfig
from raccoon.odometry_stm32 import Stm32Odometry, Stm32OdometryConfig
from raccoon.motion import UnifiedMotionPidConfig, AxisConstraints


class _Defs:
    left = Motor(port=0, inverted=False)
    right = Motor(port=1, inverted=False)
    imu = IMU()


class Robot:
    defs = _Defs()
    kinematics = DifferentialKinematics(
        left_motor=defs.left,
        right_motor=defs.right,
        wheelbase=0.15,
        wheel_radius=0.03,
    )
    drive = Drive(
        kinematics=kinematics,
        vel_config=ChassisVelocityControlConfig(),
        imu=defs.imu,
    )

    # odometry — use the stock STM32 bridge, same as real robots
    from raccoon.hal import OdometryBridge as _OB
    _bridge = _OB()
    odometry = Stm32Odometry(
        imu=defs.imu,
        kinematics=kinematics,
        bridge=_bridge,
        config=Stm32OdometryConfig(),
    )

    motion_pid_config = UnifiedMotionPidConfig()
    motion_pid_config.linear = AxisConstraints(0.8, 1.5, 1.5)
    motion_pid_config.lateral = AxisConstraints(0.5, 1.0, 1.0)
    motion_pid_config.angular = AxisConstraints(6.0, 12.0, 12.0)

    width_cm = 18.0
    length_cm = 18.0
'''


USER_TEST = '''\
"""User-written step test — uses the plugin fixtures."""
from raccoon.step.motion.drive_dsl import drive_forward
from raccoon.testing.sim import pose


def test_drive_ten_cm(robot, scene, run_step):
    scene("empty_table.ftmap", start=(20.0, 50.0, 0.0))
    run_step(drive_forward(cm=10.0), robot, timeout=8.0)
    # Ground-truth sim pose should have advanced ~10 cm in +x from start.
    assert pose().x > 28.0, f"didn't move — pose.x = {pose().x}"
    assert pose().x < 35.0, f"overshot — pose.x = {pose().x}"


def test_derived_sim_config(robot_sim_config, project_info):
    # Prove the plugin parsed our fake project yml correctly.
    assert robot_sim_config.wheel_radius_m == 0.03
    assert robot_sim_config.track_width_m == 0.15
    assert robot_sim_config.left_motor_port == 0
    assert robot_sim_config.right_motor_port == 1
    assert project_info.root.is_dir()
'''


def _scaffold_project(root: Path) -> None:
    (root / "raccoon.project.yml").write_text(PROJECT_YML)
    hw_dir = root / "src" / "hardware"
    hw_dir.mkdir(parents=True)
    (root / "src" / "__init__.py").write_text("")
    (hw_dir / "__init__.py").write_text("")
    (hw_dir / "robot.py").write_text(ROBOT_PY)

    tests_dir = root / "tests"
    tests_dir.mkdir()
    (tests_dir / "test_user_step.py").write_text(USER_TEST)

    # Copy the bundled empty_table scene into the project's scenes dir so
    # the test is self-contained even if raccoon wasn't installed with
    # bundled scenes.
    scenes_dir = root / "scenes"
    scenes_dir.mkdir()
    src_scene = (
        Path(__file__).resolve().parents[3] / "scenes" / "empty_table.ftmap"
    )
    (scenes_dir / "empty_table.ftmap").write_bytes(src_scene.read_bytes())


def test_plugin_runs_in_synthetic_project(tmp_path: Path) -> None:
    _scaffold_project(tmp_path)

    env = os.environ.copy()
    env["LIBSTP_LOG_LEVEL"] = "warn"
    env["LIBSTP_NO_CALIBRATE"] = "1"
    # Strip any test-runner artifacts that might point pytest at the
    # library's own tests instead of the scaffold.
    env.pop("PYTEST_ADDOPTS", None)

    proc = subprocess.run(
        [
            sys.executable,
            "-m", "pytest",
            # Force-load the plugin by module path. When the wheel ships
            # the pytest11 entry point this is redundant, but it keeps
            # the test working on a raccoon install that predates the
            # entry point being registered.
            "-p", "raccoon.testing.pytest_plugin",
            "-p", "no:cacheprovider",
            "-o", "filterwarnings=",  # tolerate the deprecation warning path
            "-o", "addopts=",
            "tests/test_user_step.py",
            "-v",
        ],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )

    # Dump output on failure so debugging the scaffold is tractable.
    if proc.returncode != 0:
        raise AssertionError(
            f"pytest subprocess failed (exit={proc.returncode})\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )

    # Both synthetic tests should have passed. We look for per-test
    # PASSED markers rather than the "2 passed" summary line because the
    # plugin's pytest_sessionfinish hook os._exit()s the subprocess
    # early to sidestep a known pybind11 mock-HAL teardown segfault,
    # and the exit happens before the terminal summary is flushed.
    assert "test_drive_ten_cm PASSED" in proc.stdout, proc.stdout
    assert "test_derived_sim_config PASSED" in proc.stdout, proc.stdout
