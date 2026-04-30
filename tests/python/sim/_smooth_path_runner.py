"""Standalone runner for smooth_path() test scenarios.

Invoked as a subprocess by ``test_smooth_path.py`` so the C++ extension
state stays isolated from pytest's process. Prints a single JSON line of
results, then ``os._exit``s before any C++ destructors run.

Accepts an optional ``--config`` argument (``default``, ``drumbot``, or
``packingbot``) to select the robot configuration for the sim.
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SCENES_DIR = REPO_ROOT / "scenes"
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _robot_builder import build_robot as _build_robot  # noqa: E402
from _robot_builder import get_config as _get_config  # noqa: E402


def _log(msg: str) -> None:
    sys.stderr.write(f"[smooth_path_runner] {msg}\n")
    sys.stderr.flush()


async def _scenarios(config_name: str):
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.smooth_path import smooth_path
    from raccoon.step.motion.turn_dsl import turn_right
    from raccoon.step.sequential import seq
    from raccoon.testing.sim import pose, use_scene

    cfg = _get_config(config_name)
    _log(f"building robot with config={config_name}")
    robot = _build_robot(cfg)
    out: dict[str, list[float]] = {}

    # ---- Scenario 1: single segment (should behave like normal drive) ----
    _log("scenario 1: smooth_path single drive(30)")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = smooth_path(drive_forward(cm=30.0), correct=False)
        await asyncio.wait_for(step.run_step(robot), timeout=8.0)
        p = pose()
        out["single_drive"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}")

    # ---- Scenario 2: two same-type drives (should carry velocity) ----
    _log("scenario 2: smooth_path drive(20) + drive(20)")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = smooth_path(
            drive_forward(cm=20.0),
            drive_forward(cm=20.0),
            correct=False,
        )
        await asyncio.wait_for(step.run_step(robot), timeout=8.0)
        p = pose()
        out["two_drives"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}")

    # ---- Scenario 3: same total distance via seq() for comparison ----
    _log("scenario 3: seq drive(20) + drive(20) for timing comparison")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = seq([drive_forward(cm=20.0), drive_forward(cm=20.0)])
        await asyncio.wait_for(step.run_step(robot), timeout=8.0)
        p = pose()
        out["two_drives_seq"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}")

    # ---- Scenario 4: three drives ----
    _log("scenario 4: smooth_path drive(15) + drive(15) + drive(15)")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(30.0, 50.0, 0.0)):
        step = smooth_path(
            drive_forward(cm=15.0),
            drive_forward(cm=15.0),
            drive_forward(cm=15.0),
            correct=False,
        )
        await asyncio.wait_for(step.run_step(robot), timeout=8.0)
        p = pose()
        out["three_drives"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}")

    # ---- Scenario 5: cross-type drive + turn + drive ----
    _log("scenario 5: smooth_path drive(20) + turn_right(90) + drive(20)")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = smooth_path(
            drive_forward(cm=20.0),
            turn_right(90),
            drive_forward(cm=20.0),
            correct=False,
        )
        await asyncio.wait_for(step.run_step(robot), timeout=12.0)
        p = pose()
        out["drive_turn_drive"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={p.theta:.3f}")

    # ---- Scenario 6: cross-type via seq() for comparison ----
    _log("scenario 6: seq drive(20) + turn_right(90) + drive(20)")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = seq(
            [
                drive_forward(cm=20.0),
                turn_right(90),
                drive_forward(cm=20.0),
            ]
        )
        await asyncio.wait_for(step.run_step(robot), timeout=12.0)
        p = pose()
        out["drive_turn_drive_seq"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={p.theta:.3f}")

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
