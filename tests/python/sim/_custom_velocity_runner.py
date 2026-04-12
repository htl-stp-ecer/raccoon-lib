"""Standalone runner for custom_velocity end-to-end sim scenarios.

Invoked as a subprocess by ``test_custom_velocity_mission.py`` so C++
extension teardown races stay isolated from pytest. Prints a single JSON
line of results, then ``os._exit``s before any C++ destructors run.

Accepts an optional ``--config`` argument (``default``, ``drumbot``, or
``packingbot``) to select the robot configuration for the sim.

Scenarios
---------
forward_1s
    Drive forward at 100% for 1 second via after_seconds(1.0).
    Verifies the velocity is actually applied and the robot moves.

forward_until_cm
    Drive forward at 60% until after_cm(20) fires.
    Verifies the until condition terminates the step at the right distance.

spin_1s
    Pure rotation at 100% for 1 second.
    Verifies omega is applied (heading changes) while vx/vy stay near zero.

zero_velocity
    Emit (0, 0, 0) for 0.5 s.
    Verifies the step terminates cleanly and the robot does not drift.
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
    pid_cfg.linear = AxisConstraints(0.8, 1.5, 1.5)   # max 0.8 m/s forward
    pid_cfg.lateral = AxisConstraints(0.5, 1.0, 1.0)  # max 0.5 m/s lateral
    pid_cfg.angular = AxisConstraints(6.0, 12.0, 12.0) # max 6.0 rad/s

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
    sys.stderr.write(f"[cv-runner] {msg}\n")
    sys.stderr.flush()


async def _scenarios(config_name: str):
    from raccoon.step.motion.custom_velocity_dsl import custom_velocity
    from raccoon.step.condition import after_seconds, after_forward_cm
    from raccoon.testing.sim import pose, use_scene

    cfg = _get_config(config_name)
    _log(f"building robot with config={config_name}")
    robot = _build_robot(cfg)
    out: dict[str, object] = {}

    # ------------------------------------------------------------------
    # Scenario 1: full forward for 1 second
    # max linear = 0.8 m/s → expect ~80 cm forward travel
    # ------------------------------------------------------------------
    _log("scenario 1: forward at 100% for 1 s")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        robot.odometry.reset()
        await asyncio.wait_for(
            custom_velocity(lambda r, dt: (1.0, 0.0, 0.0))
            .until(after_seconds(1.0))
            .run_step(robot),
            timeout=10.0,
        )
        p = pose()
        out["forward_1s"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={math.degrees(p.theta):.1f}°")

    # ------------------------------------------------------------------
    # Scenario 2: forward at 60% until 20 cm forward displacement
    # Uses after_forward_cm (pose-based) because the mock sim does not
    # accumulate get_path_length() — pose is always correct.
    # ------------------------------------------------------------------
    _log("scenario 2: forward at 60% until after_forward_cm(20)")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        robot.odometry.reset()
        await asyncio.wait_for(
            custom_velocity(lambda r, dt: (0.6, 0.0, 0.0))
            .until(after_forward_cm(20))
            .run_step(robot),
            timeout=10.0,
        )
        p = pose()
        out["forward_until_cm"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}")

    # ------------------------------------------------------------------
    # Scenario 3: pure spin at 100% for 1 second
    # max angular = 6.0 rad/s → expect ~6 rad rotation (~344°)
    # ------------------------------------------------------------------
    _log("scenario 3: pure spin at 100% for 1 s")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        robot.odometry.reset()
        await asyncio.wait_for(
            custom_velocity(lambda r, dt: (0.0, 0.0, 1.0))
            .until(after_seconds(1.0))
            .run_step(robot),
            timeout=10.0,
        )
        p = pose()
        out["spin_1s"] = [p.x, p.y, p.theta]
        _log(f"  ended at theta={math.degrees(p.theta):.1f}°")

    # ------------------------------------------------------------------
    # Scenario 4: zero velocity for 0.5 s — robot should not drift
    # ------------------------------------------------------------------
    _log("scenario 4: zero velocity for 0.5 s")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        robot.odometry.reset()
        await asyncio.wait_for(
            custom_velocity(lambda r, dt: (0.0, 0.0, 0.0))
            .until(after_seconds(0.5))
            .run_step(robot),
            timeout=5.0,
        )
        p = pose()
        out["zero_velocity"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}")

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

    os._exit(0)


if __name__ == "__main__":
    main()
