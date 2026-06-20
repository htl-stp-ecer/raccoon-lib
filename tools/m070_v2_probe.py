#!/usr/bin/env python3
"""Per-step M070 trace under the v2 ramp map (does the ramp let M070 complete?).

Seeds M007..M050 (seq) on the v2 multi-layer map, then runs M070 step-by-step.
With the ramp authored as a layer + transitions, the robot should switch onto
the ramp plane and its sensor steps should fire on the ramp tape — so the final
`drive_backward().until(over_line(rear.left))` step that stalled at the east
wall on the v1 map should now complete.

    RACCOON_SIM_FASTTIME=1 .venv-test/bin/python tools/m070_v2_probe.py
"""

from __future__ import annotations

import asyncio
import importlib
import inspect
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
SEED = ["m007", "m010", "m020", "m030", "m040", "m050"]
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


async def _main(target, per_step_timeout, scene):
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

    start = (START[0], START[1], math.radians(START[2]))
    print(f"scene: {Path(scene).name}")
    with use_scene(str(scene), robot=cfg, start=start):
        await asyncio.sleep(0.05)
        for p in SEED:
            await asyncio.wait_for(_mission(p)().sequence().resolve().run_step(robot), timeout=120)
        pe = pose()
        print(f"\n=== {target} entry: ({pe.x:.1f},{pe.y:.1f},{math.degrees(pe.theta):.1f}°) ===")

        raw = _mission(target)().sequence()
        stalled = False
        for i, st in enumerate(raw.steps):
            r = st.resolve() if hasattr(st, "resolve") else st
            try:
                sig = r._generate_signature()[:48]
            except Exception:
                sig = type(r).__name__
            try:
                await asyncio.wait_for(r.run_step(robot), timeout=per_step_timeout)
                stt = "ok"
            except TimeoutError:
                stt = "TIMEOUT"
            except Exception as e:  # noqa: BLE001
                stt = type(e).__name__
            q = pose()
            from raccoon import sim as _sim
            layer = _sim.mock.current_layer()
            print(f"  [{i:2d}] {sig:48s} -> ({q.x:7.2f},{q.y:7.2f},{math.degrees(q.theta):6.1f}°) L{layer} {stt}")
            if stt == "TIMEOUT":
                print("  ^^^ STALLED HERE"); stalled = True; break
        if not stalled:
            print(f"\n*** {target} COMPLETED on the v2 ramp map ***")


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "m070"
    pst = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
    import sim_fasttime

    sim_fasttime.enable()
    asyncio.run(_main(target, pst, V2_SCENE))
    os._exit(0)


if __name__ == "__main__":
    main()
