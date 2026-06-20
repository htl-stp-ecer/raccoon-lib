#!/usr/bin/env python3
"""Measure run-to-run endpoint variance of a baseline mission in the mock sim.

The mock sim auto-ticks on WALL-CLOCK by default, so the per-cycle integration
dt depends on real elapsed time (and system load). That makes the same mission
land on a slightly different endpoint each run — the suspected source of the
~1cm residual delta between optimize(merge) and baseline.

This probe runs the SAME mission N times from the SAME start pose (re-posing
each run, NO chaining) and reports the endpoint spread. Run it both ways:

    .venv-test/bin/python tools/determinism_probe.py --mission M007 -n 5
    RACCOON_SIM_FASTTIME=1 .venv-test/bin/python tools/determinism_probe.py --mission M007 -n 5

Wall-clock mode should show spread; FASTTIME (fixed virtual dt) should be ~0.
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
sys.path.insert(0, str(REPO_ROOT / "tools"))
sys.path.insert(0, str(REPO_ROOT / "tests" / "python" / "sim"))

import sim_run_chain as H  # noqa: E402

os.environ.setdefault("LIBSTP_LOG_LEVEL", "error")
os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
os.environ["LIBSTP_TIMING_ENABLED"] = "0"
os.environ["RACCOON_SIM"] = "1"


def _find_mission(prefix: str):
    import src.missions as M_pkg  # type: ignore[import-not-found]

    from raccoon.mission.api import Mission

    for mi in __import__("pkgutil").iter_modules(M_pkg.__path__):
        if not mi.name.lower().startswith(prefix.lower()):
            continue
        mod = importlib.import_module(f"src.missions.{mi.name}")
        for _nm, obj in inspect.getmembers(mod, inspect.isclass):
            if issubclass(obj, Mission) and obj.__module__ == mod.__name__:
                return obj
    raise SystemExit(f"no mission module starting with {prefix!r}")


async def _run_once(robot, scene, start, cls, timeout):
    from raccoon.testing.sim import pose, use_scene

    with use_scene(str(scene), robot=H._make_sim_config(), start=start):
        await asyncio.sleep(0.05)
        try:
            await asyncio.wait_for(cls().sequence().resolve().run_step(robot), timeout=timeout)
            status = "ok"
        except TimeoutError:
            status = "TIMEOUT"
        except Exception as e:  # noqa: BLE001
            status = f"{type(e).__name__}"
        p = pose()
        return (p.x, p.y, math.degrees(p.theta), status)


async def _main(args, fast):
    project = Path(args.project).resolve()
    sys.path.insert(0, str(project))
    os.chdir(project)

    from _robot_builder import build_robot  # type: ignore[import-not-found]

    scene = H._sim_scene_path(project / H.DEFAULT_SCENE)
    H._install_fake_transport()
    cfg = H._make_sim_config()
    robot = H._augment_services(build_robot(cfg, enable_localization=True))
    try:
        from src.hardware.defs import defs as _defs  # type: ignore[import-not-found]
        from src.hardware.robot import Robot  # type: ignore[import-not-found]

        robot.motion_pid_config = Robot.motion_pid_config
        robot.defs = _defs
    except Exception as e:  # noqa: BLE001
        print(f"  [warn] {e}")
    H._calibrate_ir_sensors()

    cls = _find_mission(args.mission)
    sx, sy, sdeg = (float(v) for v in args.start.split(","))
    start = (sx, sy, math.radians(sdeg))

    mode = "FASTTIME" if fast else "wall-clock"
    print(f"\n=== {args.mission} x{args.n}  [{mode}]  start=({sx},{sy},{sdeg}°) ===")
    rows = []
    for i in range(args.n):
        x, y, th, st = await _run_once(robot, scene, start, cls, args.timeout)
        rows.append((x, y, th))
        print(f"  run {i}: ({x:7.2f}, {y:7.2f}, {th:6.1f}°)  [{st}]")

    xs = [r[0] for r in rows]
    ys = [r[1] for r in rows]
    ths = [r[2] for r in rows]

    def spread(v):
        return max(v) - min(v)

    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    pos_dev = [math.hypot(x - mx, y - my) for x, y in zip(xs, ys)]
    print(
        f"  --- spread: x={spread(xs):.3f}cm  y={spread(ys):.3f}cm  "
        f"theta={spread(ths):.3f}°  max|pos-mean|={max(pos_dev):.3f}cm"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--project", default=H.DEFAULT_PROJECT)
    ap.add_argument("--mission", default="m007")
    ap.add_argument("-n", type=int, default=5)
    ap.add_argument("--start", default="42.8,25.63,90")
    ap.add_argument("--timeout", type=float, default=40.0)
    args = ap.parse_args()
    # enable() installs an event-loop policy and MUST run before asyncio.run.
    import sim_fasttime

    fast = sim_fasttime.enable()
    asyncio.run(_main(args, fast))
    os._exit(0)


if __name__ == "__main__":
    main()
