#!/usr/bin/env python3
"""Run the cube-bot chain on the v2 ramp map and dump the trajectory + layer.

Polls pose + current_layer continuously so tools/visualize_v2_map.py can show
where the robot rides the ramp plane. Runs M007..M050 then M070, M080 (the
ramp missions) so the full ramp interaction is captured.

    RACCOON_SIM_FASTTIME=1 .venv-test/bin/python tools/dump_v2_trajectory.py --out /tmp/v2_traj.json
"""

from __future__ import annotations

import argparse
import asyncio
import importlib
import inspect
import json
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))
sys.path.insert(0, str(REPO_ROOT / "tests" / "python" / "sim"))
import sim_run_chain as H  # noqa: E402

os.environ["RACCOON_SIM"] = "1"
os.environ.setdefault("LIBSTP_LOG_LEVEL", "error")
os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
os.environ["LIBSTP_TIMING_ENABLED"] = "0"

START = (42.8, 25.63, 90.0)
CHAIN = ["m007", "m010", "m020", "m030", "m040", "m050", "m070", "m080"]
V2_SCENE = Path(H.DEFAULT_PROJECT) / "config" / "2026-game-table-v2.ftmap"


def _mission(prefix: str):
    import src.missions as M_pkg  # type: ignore[import-not-found]
    from raccoon.mission.api import Mission

    for mi in __import__("pkgutil").iter_modules(M_pkg.__path__):
        if mi.name.lower().startswith(prefix):
            mod = importlib.import_module(f"src.missions.{mi.name}")
            for _n, obj in inspect.getmembers(mod, inspect.isclass):
                if issubclass(obj, Mission) and obj.__module__ == mod.__name__:
                    return obj
    raise SystemExit(prefix)


async def _main(out_path):
    from raccoon import sim as _sim
    from raccoon.testing.sim import pose, use_scene

    project = Path(H.DEFAULT_PROJECT)
    sys.path.insert(0, str(project))
    H._install_fake_transport()
    cfg = H._make_sim_config()
    from _robot_builder import build_robot  # type: ignore[import-not-found]

    robot = H._augment_services(build_robot(cfg, enable_localization=True))
    from src.hardware.defs import defs as _defs  # type: ignore[import-not-found]
    from src.hardware.robot import Robot  # type: ignore[import-not-found]

    robot.motion_pid_config = Robot.motion_pid_config
    robot.defs = _defs
    H._calibrate_ir_sensors()

    traj: list[dict] = []
    running = {"on": True, "mission": ""}

    async def poll():
        while running["on"]:
            p = pose()
            traj.append({
                "x": round(p.x, 2), "y": round(p.y, 2),
                "theta": round(math.degrees(p.theta), 1),
                "layer": _sim.mock.current_layer(),
                "m": running["mission"],
            })
            await asyncio.sleep(0.04)

    start = (START[0], START[1], math.radians(START[2]))
    with use_scene(str(V2_SCENE), robot=cfg, start=start):
        poller = asyncio.create_task(poll())
        await asyncio.sleep(0.05)
        for pfx in CHAIN:
            running["mission"] = pfx
            cls = _mission(pfx)
            try:
                await asyncio.wait_for(cls().sequence().resolve().run_step(robot), timeout=120)
                st = "ok"
            except Exception as e:  # noqa: BLE001
                st = type(e).__name__
            q = pose()
            print(f"  {pfx}: ({q.x:.1f},{q.y:.1f},{math.degrees(q.theta):.0f}) L{_sim.mock.current_layer()} [{st}]")
        running["on"] = False
        await asyncio.gather(poller, return_exceptions=True)

    Path(out_path).write_text(json.dumps({"traj": traj}))
    n_ramp = sum(1 for p in traj if p["layer"] >= 1)
    print(f"wrote {out_path}: {len(traj)} pts ({n_ramp} on ramp plane)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/v2_traj.json")
    args = ap.parse_args()
    import sim_fasttime

    sim_fasttime.enable()
    asyncio.run(_main(args.out))
    os._exit(0)


if __name__ == "__main__":
    main()
