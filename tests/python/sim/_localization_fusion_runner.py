"""Subprocess runner: the ContinuousLocalizationFusion SERVICE keeps localization
map-corrected under injected odometry drift — automatically, NO resync step, NO
ground-truth feeding, reading REAL sim IR line sensors over the game-table map.

Emits one ``RESULTS:<json>`` line with the dead-reckoning vs particle-filter
end-of-run error (cm) against ground truth. Invoked once per mode by
``test_localization_fusion_sim.py`` (subprocess isolates the C++ mock state).

Modes:
  - ``fusion``   — fusion service running (expected: filter error << dead-reckoning)
  - ``nofusion`` — service NOT started (expected: filter error ≈ dead-reckoning)
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parent))

import contextlib  # noqa: E402

from _robot_builder import build_robot  # noqa: E402

# Game-table map with the tape grid (7 lines); the sim_drivable copy drops walls.
SCENE = Path(
    "/media/tobias/TobiasSSD/projects/Botball/competition/Ecer2026/cube-bot/"
    "config/2026-game-table__sim_drivable.ftmap"
)
START = (42.8, 20.0, math.radians(90))
DRIFT = (0.75, 0.75, 1.0)
SENS = [(0, 6.0, -4.0), (1, 6.0, 0.0), (2, 6.0, 4.0)]


def _pose_obs(x_m, y_m, h):
    from raccoon.foundation import Pose
    from raccoon.localization import Observation

    o = Observation()
    p = Pose()
    p.position = [x_m, y_m, 0.0]
    p.heading = h
    o.pose = p
    o.sigma = (0.02, 0.02, 0.02)
    return o


async def _run(mode: str, out: dict) -> None:
    import raccoon.sim as S
    from raccoon import TableMap
    from raccoon.foundation import Pose  # noqa: F401
    from raccoon.localization import Localization, LocalizationConfig
    from raccoon.robot.geometry import SensorPosition
    from raccoon.robot.localization_fusion import ContinuousLocalizationFusion
    from raccoon.sensor_ir import IRSensor
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.testing.sim import LineSensorMount, SimRobotConfig, pose, use_scene

    cfg = SimRobotConfig(
        drivetrain="mecanum",
        wheelbase_m=0.125,
        track_width_m=0.20,
        wheel_radius_m=0.0375,
        max_wheel_velocity_rad_s=30.0,
        line_sensors=[
            LineSensorMount(analog_port=p, forward_cm=f, strafe_cm=s, name=f"port{p}")
            for (p, f, s) in SENS
        ],
    )
    robot = build_robot(cfg, enable_localization=False)
    robot._services = {}
    robot.get_service = lambda c: robot._services.setdefault(c, c(robot))
    for lvl in ("trace", "debug", "info", "warn", "warning", "error"):
        setattr(robot, lvl, lambda *a, **k: None)

    irs = {}
    for p, f, s in SENS:
        ir = IRSensor(p)
        ir.setCalibration(700.0, 300.0)  # sim scale 1023=black / 0=white
        irs[ir] = SensorPosition(forward_cm=f, strafe_cm=s)
    robot.all_sensors = lambda: dict(irs)

    tmap = TableMap()
    tmap.load(str(SCENE))
    loc = Localization(robot.odometry, LocalizationConfig(tick_period_ms=5), tmap)
    robot.localization = loc

    with use_scene(SCENE, robot=cfg, start=START):
        S.mock.set_odometry_drift(*DRIFT)
        loc.observe(_pose_obs(START[0] / 100, START[1] / 100, START[2]))
        await asyncio.sleep(0.1)

        task = None
        if mode == "fusion":
            fusion = ContinuousLocalizationFusion(robot, hz=60.0, anchor_sigma_cm=8.0)
            task = asyncio.create_task(fusion.run())

        await asyncio.wait_for(drive_forward(55).run_step(robot), timeout=25)

        if task is not None:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

        gp = pose()
        odo = robot.odometry.get_pose()
        lp = loc.get_pose()

    c, s = math.cos(START[2]), math.sin(START[2])
    ox, oy = odo.position[0] * 100, odo.position[1] * 100
    dr_x = START[0] + (ox * c - oy * s)
    dr_y = START[1] + (ox * s + oy * c)
    pf_x, pf_y = lp.position[0] * 100, lp.position[1] * 100
    out[mode] = {
        "dr_err": math.hypot(dr_x - gp.x, dr_y - gp.y),
        "pf_err": math.hypot(pf_x - gp.x, pf_y - gp.y),
        "gt": [gp.x, gp.y],
    }


def main() -> None:
    mode = "fusion"
    if "--mode" in sys.argv:
        mode = sys.argv[sys.argv.index("--mode") + 1]
    out: dict = {}
    try:
        asyncio.run(_run(mode, out))
        sys.stdout.write("RESULTS:" + json.dumps(out) + "\n")
        sys.stdout.flush()
    except BaseException as e:
        import traceback

        sys.stderr.write(f"ERR: {type(e).__name__}: {e}\n")
        traceback.print_exc(file=sys.stderr)
        os._exit(2)
    os._exit(0)


if __name__ == "__main__":
    main()
