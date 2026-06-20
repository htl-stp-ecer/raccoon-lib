#!/usr/bin/env python3
"""Run a cube-bot mission headless in the simulator.

Loads a mission from a raccoon project (default: cube-bot), builds a mecanum
sim robot whose geometry + line-sensor mounts are derived from the project's
``Robot`` class, places it on the game-table scene at a given start pose, and
drives the mission's step tree to completion under the mock bundle — no
hardware, no transport.

This exercises the REAL motion/sensor/condition pipeline:
``drive``/``strafe``/``line_follow``/``turn`` regulate on odometry, ``until``
conditions read sim-driven sensors, servos no-op on mock. (``goto`` /
``to_absolute`` regulate on the localization filter, which currently mis-tracks
lateral motion — see tests/python/sim/test_goto_sim.py.)

Run under the mock test venv:

    .venv-test/bin/python tools/sim_run_mission.py --mission m007 --start 50,50,0
    .venv-test/bin/python tools/sim_run_mission.py --list

The START pose is in table centimetres + heading degrees and MUST match where
the real robot is placed for that mission — pass the real value with --start.
"""

from __future__ import annotations

import argparse
import asyncio
import importlib
import inspect
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROJECT = "/media/tobias/TobiasSSD/projects/Botball/competition/Ecer2026/cube-bot"
DEFAULT_SCENE = "config/2026-game-table.ftmap"


def _make_sim_config(project: Path):
    """Derive a mecanum SimRobotConfig + line-sensor mounts from the project Robot."""
    from src.hardware.robot import Robot  # type: ignore[import-not-found]

    from raccoon.testing.sim import LineSensorMount, SimRobotConfig

    kin = Robot.kinematics  # MecanumKinematics built at class definition

    # Motor ports / inversions from the project Defs (mirrors robot.py wiring).
    from src.hardware.defs import Defs  # type: ignore[import-not-found]

    def port(m):
        return m.port

    line_sensors = []
    for sensor, sp in Robot._sensor_positions.items():
        # Each _sensor_positions key is a HAL sensor with a .port; the sim
        # attaches a line sensor on that analog port at the mount offset.
        p = getattr(sensor, "port", None)
        if p is None:
            continue
        line_sensors.append(
            LineSensorMount(
                analog_port=int(p),
                forward_cm=float(sp.forward_cm),
                strafe_cm=float(sp.strafe_cm),
                name=f"port{p}",
            )
        )

    cfg = SimRobotConfig(
        drivetrain="mecanum",
        fl_motor_port=port(Defs.front_left_motor),
        fr_motor_port=port(Defs.front_right_motor),
        bl_motor_port=port(Defs.rear_left_motor),
        br_motor_port=port(Defs.rear_right_motor),
        fl_motor_inverted=Defs.front_left_motor.inverted,
        fr_motor_inverted=Defs.front_right_motor.inverted,
        bl_motor_inverted=Defs.rear_left_motor.inverted,
        br_motor_inverted=Defs.rear_right_motor.inverted,
        wheelbase_m=0.125,
        track_width_m=0.20,
        wheel_radius_m=0.0375,
        max_wheel_velocity_rad_s=30.0,
        width_cm=float(getattr(Robot, "width_cm", 23.5)),
        length_cm=float(getattr(Robot, "length_cm", 26.0)),
        rotation_center_forward_cm=float(getattr(Robot, "rotation_center_forward_cm", 0.0)),
        line_sensors=line_sensors,
    )
    return cfg


def _augment_services(robot):
    """Give the SimpleNamespace robot the GenericRobot ``get_service`` API.

    Heading steps (mark_heading_reference / turn_to_heading) call
    ``robot.get_service(HeadingReferenceService)``; the sim robot builder does
    not provide it. Mirror GenericRobot's lazy cache.
    """
    robot._services = {}

    def get_service(cls):
        svc = robot._services.get(cls)
        if svc is None:
            svc = cls(robot)
            robot._services[cls] = svc
        return svc

    robot.get_service = get_service
    return robot


def discover_missions(project: Path):
    import src.missions as M_pkg  # type: ignore[import-not-found]

    from raccoon.mission.api import Mission

    found = []
    for mi in __import__("pkgutil").iter_modules(M_pkg.__path__):
        try:
            mod = importlib.import_module(f"src.missions.{mi.name}")
        except Exception as e:
            print(f"[skip module {mi.name}] {type(e).__name__}: {e}")
            continue
        for nm, obj in inspect.getmembers(mod, inspect.isclass):
            if issubclass(obj, Mission) and obj.__module__ == mod.__name__:
                found.append((nm, obj))
    found.sort()
    return found


async def _run_mission(project: Path, scene: Path, cls, start, timeout: float):
    sys.path.insert(0, str(REPO_ROOT / "tests" / "python" / "sim"))
    from _robot_builder import build_robot  # type: ignore[import-not-found]

    from raccoon.testing.sim import pose, use_scene

    cfg = _make_sim_config(project)
    robot = _augment_services(build_robot(cfg, enable_localization=True))

    mission = cls()
    seq = mission.sequence().resolve()

    with use_scene(str(scene), robot=cfg, start=start):
        p0 = pose()
        print(f"  start: ({p0.x:.1f}, {p0.y:.1f}, {math.degrees(p0.theta):.1f}°)")
        try:
            await asyncio.wait_for(seq.run_step(robot), timeout=timeout)
            status = "completed"
        except TimeoutError:
            status = f"TIMEOUT after {timeout:.0f}s"
        except Exception as e:
            status = f"{type(e).__name__}: {e}"
        p = pose()
        print(f"  end:   ({p.x:.1f}, {p.y:.1f}, {math.degrees(p.theta):.1f}°)  [{status}]")
        return status


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--project", default=DEFAULT_PROJECT)
    ap.add_argument("--scene", default=None, help="ftmap (default: project game table)")
    ap.add_argument("--mission", default=None, help="substring filter; omit to run all")
    ap.add_argument("--start", default="50,50,0", help="table pose 'x_cm,y_cm,heading_deg'")
    ap.add_argument("--timeout", type=float, default=40.0)
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    project = Path(args.project).resolve()
    if not (project / "src").is_dir():
        sys.exit(f"no src/ under {project}")
    sys.path.insert(0, str(project))
    os.chdir(project)
    os.environ.setdefault("LIBSTP_LOG_LEVEL", "warn")
    os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
    os.environ["LIBSTP_TIMING_ENABLED"] = "0"

    scene = Path(args.scene) if args.scene else (project / DEFAULT_SCENE)
    if not scene.exists():
        sys.exit(f"scene not found: {scene}")

    missions = discover_missions(project)
    if args.mission:
        missions = [(n, c) for (n, c) in missions if args.mission.lower() in n.lower()]
    if args.list:
        for nm, _ in missions:
            print(nm)
        return
    if not missions:
        sys.exit("no missions matched")

    sx, sy, sdeg = (float(v) for v in args.start.split(","))
    start = (sx, sy, math.radians(sdeg))

    print(f"scene={scene.name}  start=({sx},{sy},{sdeg}°)  timeout={args.timeout:.0f}s\n")
    for nm, cls in missions:
        print(f"# {nm}")
        try:
            asyncio.run(_run_mission(project, scene, cls, start, args.timeout))
        except Exception as e:
            print(f"  [harness error] {type(e).__name__}: {e}")
        print()


if __name__ == "__main__":
    main()
