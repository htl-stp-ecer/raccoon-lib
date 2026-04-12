"""Standalone runner for spline() test scenarios.

Invoked as a subprocess by ``test_spline_path.py`` so the C++ extension
state stays isolated from pytest's process. Prints a single JSON line of
results, then ``os._exit``s before any C++ destructors run.

Accepts an optional ``--config`` argument (``default``, ``drumbot``, or
``packingbot``) to select the robot configuration for the sim.
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import sys
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[3]
SCENES_DIR = REPO_ROOT / "scenes"


def _build_robot(cfg):
    """Build a HAL robot whose geometry matches the given SimRobotConfig."""
    from raccoon.hal import IMU, Motor, OdometryBridge
    from raccoon.kinematics_differential import DifferentialKinematics
    from raccoon.drive import Drive, ChassisVelocityControlConfig
    from raccoon.odometry_stm32 import Stm32Odometry, Stm32OdometryConfig
    from raccoon.motion import UnifiedMotionPidConfig, AxisConstraints

    left = Motor(cfg.left_motor_port, cfg.left_motor_inverted)
    right = Motor(cfg.right_motor_port, cfg.right_motor_inverted)
    imu = IMU()
    kin = DifferentialKinematics(
        left, right, cfg.track_width_m, cfg.wheel_radius_m)
    drive_obj = Drive(kin, ChassisVelocityControlConfig(), imu)
    bridge = OdometryBridge()
    odom = Stm32Odometry(
        imu=imu, kinematics=kin, bridge=bridge, config=Stm32OdometryConfig()
    )

    pid_cfg = UnifiedMotionPidConfig()
    pid_cfg.linear = AxisConstraints(0.8, 1.5, 1.5)
    pid_cfg.lateral = AxisConstraints(0.5, 1.0, 1.0)
    pid_cfg.angular = AxisConstraints(6.0, 12.0, 12.0)

    return SimpleNamespace(
        drive=drive_obj,
        odometry=odom,
        kinematics=kin,
        motion_pid_config=pid_cfg,
        _refs=(left, right, imu, bridge),
    )


def _get_config(name: str):
    from raccoon.testing.sim import SimRobotConfig

    if name == "drumbot":
        from raccoon.testing.robot_configs import DRUMBOT
        return DRUMBOT
    elif name == "packingbot":
        from raccoon.testing.robot_configs import PACKINGBOT
        return PACKINGBOT
    else:
        return SimRobotConfig(
            wheel_radius_m=0.03,
            track_width_m=0.15,
            wheelbase_m=0.15,
            left_motor_port=0,
            right_motor_port=1,
            max_wheel_velocity_rad_s=30.0,
            motor_time_constant_sec=0.02,
        )


def _log(msg: str) -> None:
    sys.stderr.write(f"[spline_path_runner] {msg}\n")
    sys.stderr.flush()


async def _scenarios(config_name: str):
    from raccoon.step.motion.spline_path import spline
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.testing.sim import pose, use_scene

    cfg = _get_config(config_name)
    _log(f"building robot with config={config_name}")
    robot = _build_robot(cfg)
    out: dict[str, list[float]] = {}

    # ---- Scenario 1: straight line (two waypoints, forward only) ----
    # Should behave like a normal drive_forward(30)
    _log("scenario 1: spline straight line 30cm")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = spline((15, 0), (30, 0))
        await asyncio.wait_for(step.run_step(robot), timeout=10.0)
        p = pose()
        out["straight_line"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={p.theta:.3f}")

    # ---- Scenario 2: reference drive_forward for comparison ----
    _log("scenario 2: drive_forward(30) for comparison")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = drive_forward(cm=30.0)
        await asyncio.wait_for(step.run_step(robot), timeout=10.0)
        p = pose()
        out["drive_forward_30"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={p.theta:.3f}")

    # ---- Scenario 3: gentle S-curve ----
    # Start at (50, 50, 0), curve right then back to center
    _log("scenario 3: S-curve spline")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = spline(
            (15, 0),
            (30, -10),   # curve right (negative = right in left-positive frame)
            (45, 0),     # curve back to center line
        )
        await asyncio.wait_for(step.run_step(robot), timeout=12.0)
        p = pose()
        out["s_curve"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={p.theta:.3f}")

    # ---- Scenario 4: 90° left turn via spline ----
    # Start heading +x, curve left to end heading +y
    _log("scenario 4: 90-degree left curve")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 30.0, 0.0)):
        step = spline(
            (15, 5),
            (20, 15),
            (15, 25),
        )
        await asyncio.wait_for(step.run_step(robot), timeout=12.0)
        p = pose()
        out["curve_left_90"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={p.theta:.3f}")

    # ---- Scenario 5: half-speed straight line ----
    _log("scenario 5: spline straight at speed=0.5")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = spline((15, 0), (30, 0), speed=0.5)
        await asyncio.wait_for(step.run_step(robot), timeout=12.0)
        p = pose()
        out["straight_half_speed"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={p.theta:.3f}")

    _log("all scenarios complete")
    sys.stdout.write("RESULTS:" + json.dumps(out) + "\n")
    sys.stdout.flush()
    os._exit(0)


def main() -> None:
    config_name = "default"
    if "--config" in sys.argv:
        idx = sys.argv.index("--config")
        if idx + 1 < len(sys.argv):
            config_name = sys.argv[idx + 1]

    try:
        asyncio.run(_scenarios(config_name))
    except BaseException as e:  # noqa: BLE001
        import traceback
        sys.stderr.write(f"ERR: {type(e).__name__}: {e}\n")
        traceback.print_exc(file=sys.stderr)
        sys.stderr.flush()
        os._exit(2)

    sys.stdout.write("RESULTS:{}\n")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
