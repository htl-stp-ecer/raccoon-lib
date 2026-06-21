#!/usr/bin/env python3
"""Count full-stop events in a mission's trajectory — the smoothness metric.

A sensor-driven robot brakes to ~0 at every ``.until()`` boundary; the sensor-
carry profile flows through same-direction seams instead. Fewer stop-events =
cleaner motion. Run ONE config per process (no localization-thread leakage).

    RACCOON_SIM_FASTTIME=1 .venv-test/bin/python tools/measure_smoothness.py --config baseline --target m050
    RACCOON_SIM_FASTTIME=1 .venv-test/bin/python tools/measure_smoothness.py --config time_optimal --target m050
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

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "python" / "sim"))
import sim_run_chain as H  # noqa: E402

os.environ.update(
    RACCOON_SIM="1", LIBSTP_LOG_LEVEL="error", LIBSTP_NO_CALIBRATE="1", LIBSTP_TIMING_ENABLED="0"
)

START = (42.8, 25.63, 90.0)
ALL = ["m007", "m010", "m020", "m030", "m040", "m050", "m070", "m080", "m090"]
SCENE = Path(H.DEFAULT_PROJECT) / "config" / "2026-game-table__sim_drivable.ftmap"


def _load(prefix):
    import src.missions as M  # type: ignore[import-not-found]
    from raccoon.mission.api import Mission

    mod = next(
        importlib.import_module(f"src.missions.{mi.name}")
        for mi in __import__("pkgutil").iter_modules(M.__path__)
        if mi.name.startswith(prefix)
    )
    return next(o for _, o in inspect.getmembers(mod, inspect.isclass)
                if issubclass(o, Mission) and o.__module__ == mod.__name__)


async def _main(config, target):
    import sim_fasttime

    sim_fasttime.enable()
    proj = Path(H.DEFAULT_PROJECT)
    sys.path.insert(0, str(proj))
    from _robot_builder import build_robot  # type: ignore[import-not-found]
    from raccoon.testing.sim import pose, use_scene
    from raccoon.step.motion import optimize

    H._install_fake_transport()
    cfg = H._make_sim_config()
    robot = H._augment_services(build_robot(cfg, enable_localization=True))
    from src.hardware.defs import defs as _d  # type: ignore[import-not-found]
    from src.hardware.robot import Robot  # type: ignore[import-not-found]

    robot.motion_pid_config = Robot.motion_pid_config
    robot.defs = _d
    H._calibrate_ir_sensors()

    seed = ALL[: ALL.index(target)]
    speeds: list[float] = []
    running = {"on": True}

    async def poll():
        last = None
        while running["on"]:
            p = pose()
            if last is not None:
                speeds.append(math.hypot(p.x - last[0], p.y - last[1]) / 0.04)
            last = (p.x, p.y)
            await asyncio.sleep(0.04)

    with use_scene(str(SCENE), robot=cfg, start=(START[0], START[1], math.radians(START[2]))):
        poller = asyncio.create_task(poll())
        await asyncio.sleep(0.05)
        for s in seed:
            await asyncio.wait_for(_load(s)().sequence().resolve().run_step(robot), timeout=60)
        raw = _load(target)().sequence()
        step = optimize(raw).time_optimal() if config == "time_optimal" else raw
        speeds.clear()  # measure only the target mission
        try:
            await asyncio.wait_for(step.resolve().run_step(robot), timeout=60)
            st = "ok"
        except Exception as e:  # noqa: BLE001
            st = type(e).__name__
        running["on"] = False
        await asyncio.gather(poller, return_exceptions=True)

    # Stop-event = speed dips below 2 cm/s then recovers above 5 cm/s.
    stops, below = 0, False
    for v in speeds:
        if v < 2.0 and not below:
            stops += 1
            below = True
        elif v >= 5.0:
            below = False
    moving = sum(1 for v in speeds if v >= 2.0)
    print(f"{target} {config:12s} [{st}]  stop-events={stops}  moving={moving}/{len(speeds)} samples")
    if _OUT:
        import json

        Path(_OUT).write_text(json.dumps({"config": config, "target": target,
                                          "status": st, "speeds_cmps": speeds}))
        print(f"  wrote {_OUT}")


_OUT = None


def main():
    global _OUT  # noqa: PLW0603
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="baseline")
    ap.add_argument("--target", default="m050")
    ap.add_argument("--out", default=None, help="dump per-sample speed (cm/s) to JSON")
    args = ap.parse_args()
    _OUT = args.out
    asyncio.run(_main(args.config, args.target))
    os._exit(0)


if __name__ == "__main__":
    main()
