"""Standalone runner for auto-tune sim tests.

Runs the three auto-tune phases against the physics simulator and reports
results as JSON. Each phase is run independently so failures in one don't
block the others.

Accepts ``--config`` (default/drumbot/packingbot) to select the robot.
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

# Auto-tune checks this env var to skip calibration — we need it to run.
os.environ.pop("LIBSTP_NO_CALIBRATE", None)


def _build_robot(cfg):
    """Build a HAL robot matching the given SimRobotConfig."""
    from raccoon.drive import ChassisVelocityControlConfig, Drive
    from raccoon.hal import IMU, Motor, OdometryBridge
    from raccoon.kinematics_differential import DifferentialKinematics
    from raccoon.motion import AxisConstraints, UnifiedMotionPidConfig
    from raccoon.odometry_stm32 import Stm32Odometry, Stm32OdometryConfig

    left = Motor(cfg.left_motor_port, cfg.left_motor_inverted)
    right = Motor(cfg.right_motor_port, cfg.right_motor_inverted)
    imu = IMU()
    kin = DifferentialKinematics(left, right, cfg.track_width_m, cfg.wheel_radius_m)
    drive_obj = Drive(kin, ChassisVelocityControlConfig(), imu)
    bridge = OdometryBridge()
    odom = Stm32Odometry(imu=imu, kinematics=kin, bridge=bridge, config=Stm32OdometryConfig())

    # Start with conservative defaults — auto-tune will improve them
    pid_cfg = UnifiedMotionPidConfig()
    pid_cfg.linear = AxisConstraints(0.3, 0.5, 0.5)
    pid_cfg.lateral = AxisConstraints(0.2, 0.3, 0.3)
    pid_cfg.angular = AxisConstraints(2.0, 3.0, 3.0)

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
    if name == "packingbot":
        from raccoon.testing.robot_configs import PACKINGBOT

        return PACKINGBOT
    return SimRobotConfig(
        wheel_radius_m=0.03,
        track_width_m=0.15,
        wheelbase_m=0.15,
        left_motor_port=0,
        right_motor_port=1,
        max_wheel_velocity_rad_s=30.0,
        motor_time_constant_sec=0.05,
        viscous_drag_coeff=0.5,
        coulomb_friction_rad_s2=1.0,
        bemf_noise_stddev=1.5,
    )


def _log(msg: str) -> None:
    sys.stderr.write(f"[auto_tune_runner] {msg}\n")
    sys.stderr.flush()


async def _run_characterize(robot, cfg):
    """Run Phase 1: drive characterization."""
    from raccoon.step.motion.characterize_drive import CharacterizeDrive
    from raccoon.testing.sim import use_scene

    _log("Phase 1: CharacterizeDrive")
    results = {}

    # Start near the left edge so the robot has ~190 cm of forward travel
    # before hitting the right wall — enough for a 0.9 m/s plateau.
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(10.0, 50.0, 0.0)):
        step = CharacterizeDrive(
            axes=["forward", "angular"],
            trials=1,
            power_percent=100,
            accel_timeout=1.5,
            decel_timeout=2.0,
            persist=False,
        )
        await asyncio.wait_for(step._execute_step(robot), timeout=30.0)

        for axis, r in step.results.items():
            results[axis] = {
                "max_velocity": r.max_velocity,
                "acceleration": r.acceleration,
                "deceleration": r.deceleration,
            }
            _log(
                f"  {axis}: v_max={r.max_velocity:.4f}, "
                f"accel={r.acceleration:.4f}, decel={r.deceleration:.4f}"
            )

    return results


async def _run_velocity_tune(robot, cfg):
    """Run Phase 2: velocity controller tuning."""
    from raccoon.step.motion.auto_tune import AutoTuneVelocity
    from raccoon.testing.sim import use_scene

    _log("Phase 2: AutoTuneVelocity")
    results = {}

    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = AutoTuneVelocity(
            axes=["vx"],
            persist=False,
            csv_dir=None,
        )
        await asyncio.wait_for(step._execute_step(robot), timeout=30.0)

        for axis, r in step.results.items():
            results[axis] = {
                "accepted": r.accepted,
                "plant_Ks": r.plant.Ks,
                "plant_Tu": r.plant.Tu,
                "plant_Tg": r.plant.Tg,
                "plant_method": r.plant.method,
                "pid_kp": r.pid.kp,
                "pid_ki": r.pid.ki,
                "pid_kd": r.pid.kd,
                "baseline_ise": r.baseline_ise,
                "tuned_ise": r.tuned_ise,
            }
            status = "ACCEPTED" if r.accepted else "REVERTED"
            _log(f"  {axis}: {status} | ISE {r.baseline_ise:.4f} -> {r.tuned_ise:.4f}")

    return results


async def _run_motion_tune(robot, cfg):
    """Run Phase 3: motion controller tuning (distance only for speed)."""
    from raccoon.step.motion.auto_tune import AutoTuneMotion
    from raccoon.testing.sim import use_scene

    _log("Phase 3: AutoTuneMotion (distance only)")
    results = {}

    # Center of table — motion tune drives 0.5 m forward/backward alternately,
    # so we need ~50 cm of clearance in each direction.
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(100.0, 50.0, 0.0)):
        step = AutoTuneMotion(
            axes=["distance"],
            persist=False,
            csv_dir=None,
        )
        await asyncio.wait_for(step._execute_step(robot), timeout=300.0)

        for name, r in step.results.items():
            results[name] = {
                "initial_kp": r.initial_kp,
                "initial_kd": r.initial_kd,
                "final_kp": r.final_kp,
                "final_kd": r.final_kd,
                "initial_score": r.initial_score,
                "final_score": r.final_score,
                "iterations": r.iterations,
            }
            improvement = (
                (1 - r.final_score / r.initial_score) * 100 if r.initial_score > 0 else 0.0
            )
            _log(
                f"  {name}: kp {r.initial_kp:.4f}->{r.final_kp:.4f}, "
                f"kd {r.initial_kd:.4f}->{r.final_kd:.4f} | "
                f"score {r.initial_score:.4f}->{r.final_score:.4f} "
                f"({improvement:+.1f}%)"
            )

    return results


async def _scenarios(config_name: str):
    cfg = _get_config(config_name)
    _log(f"config={config_name}")
    robot = _build_robot(cfg)
    out: dict[str, object] = {}

    # Phase 1: Characterize
    try:
        out["characterize"] = await _run_characterize(robot, cfg)
    except Exception as e:
        _log(f"Phase 1 FAILED: {e}")
        out["characterize"] = {"error": str(e)}

    # Phase 2: Velocity tune
    try:
        out["velocity"] = await _run_velocity_tune(robot, cfg)
    except Exception as e:
        _log(f"Phase 2 FAILED: {e}")
        out["velocity"] = {"error": str(e)}

    # Phase 3: Motion tune (uses updated constraints from Phase 1)
    try:
        out["motion"] = await _run_motion_tune(robot, cfg)
    except Exception as e:
        import traceback

        _log(f"Phase 3 FAILED: {type(e).__name__}: {e}")
        traceback.print_exc(file=sys.stderr)
        out["motion"] = {"error": f"{type(e).__name__}: {e}"}

    _log("all phases complete")
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
    except BaseException as e:
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
