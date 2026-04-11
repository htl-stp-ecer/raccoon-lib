"""Standalone runner that executes the drive mission scenarios.

Invoked as a subprocess by ``test_drive_mission.py`` so the C++ extension
state stays isolated from pytest's process. Prints a single JSON line of
results, then ``os._exit``s before any C++ destructors run.
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[3]
SCENES_DIR = REPO_ROOT / "scenes"


def _build_robot():
    from raccoon.hal import IMU, Motor, OdometryBridge
    from raccoon.kinematics_differential import DifferentialKinematics
    from raccoon.drive import Drive, ChassisVelocityControlConfig
    from raccoon.odometry_stm32 import Stm32Odometry, Stm32OdometryConfig
    from raccoon.motion import UnifiedMotionPidConfig, AxisConstraints

    left = Motor(0, False)
    right = Motor(1, False)
    imu = IMU()
    kin = DifferentialKinematics(left, right, 0.15, 0.03)
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


def _sim_config():
    from raccoon.testing.sim import SimRobotConfig
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
    sys.stderr.write(f"[runner] {msg}\n")
    sys.stderr.flush()


async def _scenarios():
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.testing.sim import pose, use_scene

    _log("building robot")
    robot = _build_robot()
    cfg = _sim_config()
    out: dict[str, list[float]] = {}

    _log("scenario 1: drive(30)")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        await asyncio.wait_for(drive_forward(cm=30.0).run_step(robot), timeout=8.0)
        p = pose()
        out["drive_30"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}")

    _log("scenario 2: drive(15)")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(20.0, 50.0, 0.0)):
        await asyncio.wait_for(drive_forward(cm=15.0).run_step(robot), timeout=8.0)
        p = pose()
        out["drive_15"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}")

    _log("scenario 3: wall_box")
    with use_scene(SCENES_DIR / "wall_box.ftmap", robot=cfg, start=(80.0, 50.0, 0.0)):
        try:
            await asyncio.wait_for(
                drive_forward(cm=50.0).run_step(robot), timeout=3.0
            )
        except asyncio.TimeoutError:
            pass
        p = pose()
        out["wall"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}")

    _log("all scenarios complete")
    # Emit results and exit immediately, BEFORE asyncio.run + GC tear down
    # the local C++ wrapper objects. The pybind11 / mock-HAL singleton
    # destruction order is fragile here and segfaults the interpreter
    # otherwise.
    sys.stdout.write("RESULTS:" + json.dumps(out) + "\n")
    sys.stdout.flush()
    os._exit(0)


def main() -> None:
    try:
        results = asyncio.run(_scenarios())
    except BaseException as e:  # noqa: BLE001
        import traceback
        sys.stderr.write(f"ERR: {type(e).__name__}: {e}\n")
        traceback.print_exc(file=sys.stderr)
        sys.stderr.flush()
        os._exit(2)

    sys.stdout.write("RESULTS:" + json.dumps(results) + "\n")
    sys.stdout.flush()
    # Skip Python interpreter shutdown to dodge the pybind11 / mock-HAL
    # singleton teardown segfault.
    os._exit(0)


if __name__ == "__main__":
    main()
