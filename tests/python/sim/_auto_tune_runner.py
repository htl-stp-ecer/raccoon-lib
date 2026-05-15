"""Standalone runner for auto-tune sim tests.

Runs the three auto-tune phases against the physics simulator and reports
results as JSON. Each phase is run independently so failures in one don't
block the others.

Accepts ``--config`` (default/drumbot/packingbot) to select the robot.
"""

from __future__ import annotations

import asyncio
import importlib.util
import json
import os
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[3]
SCENES_DIR = REPO_ROOT / "scenes"
for module_python_dir in sorted((REPO_ROOT / "modules").glob("*/python")):
    path_str = str(module_python_dir)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)


def _load_local_module(module_name: str, relative_path: str) -> None:
    """Force selected motion modules to load from the workspace source tree."""
    module_path = REPO_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        msg = f"Unable to load local module {module_name} from {module_path}"
        raise ImportError(msg)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    parent_name, _, child_name = module_name.rpartition(".")
    if parent_name:
        parent = sys.modules.get(parent_name)
        if parent is not None:
            setattr(parent, child_name, module)


_load_local_module(
    "raccoon.step.motion.characterize_drive",
    "modules/libstp-motion/python/raccoon/step/motion/characterize_drive.py",
)
_load_local_module(
    "raccoon.step.motion.auto_tune",
    "modules/libstp-motion/python/raccoon/step/motion/auto_tune.py",
)

# Auto-tune checks this env var to skip calibration — we need it to run.
os.environ.pop("LIBSTP_NO_CALIBRATE", None)


def _apply_motor_calibrations(motors, cfg) -> None:
    from raccoon.foundation import MotorCalibration

    per_port = getattr(cfg, "motor_calibration_by_port", {}) or {}
    for motor in motors:
        ticks_to_rad, vel_lpf_alpha = per_port.get(
            motor.port,
            (getattr(cfg, "ticks_to_rad", None), 1.0),
        )
        if ticks_to_rad is None:
            continue
        motor.set_calibration(MotorCalibration(ticks_to_rad, vel_lpf_alpha))


def _build_robot(cfg):
    """Build a HAL robot matching the given SimRobotConfig."""
    from raccoon.drive import ChassisVelocityControlConfig, Drive
    from raccoon.hal import IMU, Motor
    from raccoon.hal import platform as _platform
    from raccoon.motion import AxisConstraints, UnifiedMotionPidConfig

    imu = IMU()

    if getattr(cfg, "drivetrain", "diff") == "mecanum":
        from raccoon.kinematics_mecanum import MecanumKinematics

        fl = Motor(cfg.fl_motor_port, cfg.fl_motor_inverted)
        fr = Motor(cfg.fr_motor_port, cfg.fr_motor_inverted)
        bl = Motor(cfg.bl_motor_port, cfg.bl_motor_inverted)
        br = Motor(cfg.br_motor_port, cfg.br_motor_inverted)
        kin = MecanumKinematics(
            front_left_motor=fl,
            front_right_motor=fr,
            back_left_motor=bl,
            back_right_motor=br,
            wheelbase=cfg.wheelbase_m,
            track_width=cfg.track_width_m,
            wheel_radius=cfg.wheel_radius_m,
        )
        _apply_motor_calibrations((fl, fr, bl, br), cfg)
        kin.set_max_wheel_speed(cfg.max_wheel_velocity_rad_s)
        refs = (fl, fr, bl, br, imu)
    else:
        from raccoon.kinematics_differential import DifferentialKinematics

        left = Motor(cfg.left_motor_port, cfg.left_motor_inverted)
        right = Motor(cfg.right_motor_port, cfg.right_motor_inverted)
        kin = DifferentialKinematics(left, right, cfg.track_width_m, cfg.wheel_radius_m)
        _apply_motor_calibrations((left, right), cfg)
        kin.set_max_wheel_speed(cfg.max_wheel_velocity_rad_s)
        refs = (left, right, imu)

    drive_obj = Drive(kin, ChassisVelocityControlConfig(), imu)
    odom = _platform.Platform.create_odometry(kin)

    # Start with conservative defaults — auto-tune will improve them.
    pid_cfg = UnifiedMotionPidConfig()
    pid_cfg.linear = AxisConstraints(0.3, 0.5, 0.5)
    pid_cfg.lateral = AxisConstraints(0.2, 0.3, 0.3)
    pid_cfg.angular = AxisConstraints(2.0, 3.0, 3.0)

    from raccoon.localization import Localization, LocalizationConfig

    localization = Localization(odom, LocalizationConfig(tick_period_ms=5))

    return SimpleNamespace(
        drive=drive_obj,
        odometry=odom,
        kinematics=kin,
        motion_pid_config=pid_cfg,
        localization=localization,
        _refs=refs,
    )


def _get_config(name: str):
    from raccoon.testing.sim import SimRobotConfig

    if name == "drumbot":
        from raccoon.testing.robot_configs import DRUMBOT

        return replace(DRUMBOT, bemf_noise_stddev=0.0)
    if name == "packingbot":
        from raccoon.testing.robot_configs import PACKINGBOT

        return replace(PACKINGBOT, bemf_noise_stddev=0.0)
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
        bemf_noise_stddev=0.0,
    )


def _log(msg: str) -> None:
    sys.stderr.write(f"[auto_tune_runner] {msg}\n")
    sys.stderr.flush()


def _table_bounds(scene_path: Path) -> tuple[float, float]:
    data = json.loads(scene_path.read_text())
    table = data.get("table", {})
    return float(table["widthCm"]), float(table["heightCm"])


def _robot_x_extents(cfg) -> tuple[float, float]:
    """Return (backward_cm, forward_cm) extents from the rotation center."""
    half_length = cfg.length_cm * 0.5
    backward = cfg.rotation_center_forward_cm + half_length
    forward = half_length - cfg.rotation_center_forward_cm
    return backward, forward


def _safe_forward_start(
    cfg, scene_path: Path, *, margin_cm: float = 8.0
) -> tuple[float, float, float]:
    """Start pose with enough footprint clearance for forward characterization."""
    table_width, table_height = _table_bounds(scene_path)
    backward, forward = _robot_x_extents(cfg)
    x = max(backward + margin_cm, 10.0)
    x = min(x, table_width - forward - margin_cm)
    y = table_height * 0.5
    return x, y, 0.0


def _safe_center_start(
    cfg, scene_path: Path, *, margin_cm: float = 8.0
) -> tuple[float, float, float]:
    """Centered start pose that respects the robot footprint on all sides."""
    table_width, table_height = _table_bounds(scene_path)
    backward, forward = _robot_x_extents(cfg)
    half_width = cfg.width_cm * 0.5 + abs(cfg.rotation_center_strafe_cm)
    min_x = backward + margin_cm
    max_x = table_width - forward - margin_cm
    min_y = half_width + margin_cm
    max_y = table_height - half_width - margin_cm
    return (
        max(min_x, min(max_x, table_width * 0.5)),
        max(min_y, min(max_y, table_height * 0.5)),
        0.0,
    )


async def _run_characterize(robot, cfg):
    """Run Phase 1: drive characterization."""
    from raccoon.step.motion.characterize_drive import CharacterizeDrive
    from raccoon.testing.sim import use_scene

    _log("Phase 1: CharacterizeDrive")
    results = {}
    scene_path = SCENES_DIR / "empty_table.ftmap"
    start_pose = _safe_forward_start(cfg, scene_path)
    _log(f"  characterize start pose={start_pose}")

    with use_scene(scene_path, robot=cfg, start=start_pose):
        step = CharacterizeDrive(
            axes=["forward", "angular"],
            trials=1,
            power_percent=100,
            accel_timeout=2.5,
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
    scene_path = SCENES_DIR / "empty_table.ftmap"
    start_pose = _safe_center_start(cfg, scene_path)
    _log(f"  velocity tune start pose={start_pose}")

    with use_scene(scene_path, robot=cfg, start=start_pose):
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
    scene_path = SCENES_DIR / "empty_table.ftmap"
    start_pose = _safe_center_start(cfg, scene_path)
    _log(f"  motion tune start pose={start_pose}")

    with use_scene(scene_path, robot=cfg, start=start_pose):
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
    out["_config"] = config_name
    out["_supports_lateral"] = robot.drive.supports_lateral_motion()

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
