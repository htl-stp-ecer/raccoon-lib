"""Standalone runner for smooth_path() optimizer integration tests.

Invoked as a subprocess by ``test_path_optimizer.py`` to isolate C++
extension state. Prints a single ``RESULTS:<json>`` line to stdout.

Scenarios
---------
reference_drive_turn_drive
    Unoptimized drive(50)+turn_right(90)+drive(30) from (50, 50, 0).
    Used as the ground-truth endpoint for comparison tests.

merge_two_drives
    smooth_path(drive(20)+drive(20), optimize=True) — should reach
    same endpoint as drive(40).

merge_ref_drive
    drive(40) alone — reference for the merge test.

corner_cut_5cm
    smooth_path(drive(50)+turn_right(90)+drive(30), corner_cut_cm=5).
    Should end near the reference endpoint.

spline_drive_turn_drive
    smooth_path(drive(50)+turn_right(90)+drive(30), spline=True).
    Should end near the reference endpoint.

merge_with_barrier
    smooth_path(drive(20)+background(noop)+drive(20), optimize=True).
    Side action is a barrier — both drive segments fire separately.
    Endpoint should match drive(40) reference.
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
    sys.stderr.write(f"[optimizer_runner] {msg}\n")
    sys.stderr.flush()


async def _scenarios(config_name: str) -> None:
    from raccoon.step.base import Step
    from raccoon.step.logic.background import background
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.smooth_path import smooth_path
    from raccoon.step.motion.turn_dsl import turn_right
    from raccoon.testing.sim import pose, use_scene

    # Non-drive step used as a background() barrier
    class _Noop(Step):
        def required_resources(self):
            return frozenset({"servo:99"})

        def _generate_signature(self):
            return "Noop()"

        async def _execute_step(self, robot):
            pass

    cfg = _get_config(config_name)
    robot = _build_robot(cfg)
    out: dict[str, list[float]] = {}

    # ---- Scenario: reference drive+turn+drive (unoptimized) ----
    _log("reference: drive(50) + turn_right(90) + drive(30)")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = smooth_path(drive_forward(50), turn_right(90), drive_forward(30), correct=False)
        await asyncio.wait_for(step.run_step(robot), timeout=20.0)
        p = pose()
        out["reference_drive_turn_drive"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={p.theta:.3f}")

    # ---- Scenario: optimize=True merges two drives ----
    _log("merge: drive(20) + drive(20) with optimize=True")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = smooth_path(drive_forward(20), drive_forward(20), optimize=True, correct=False)
        await asyncio.wait_for(step.run_step(robot), timeout=12.0)
        p = pose()
        out["merge_two_drives"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}")

    # ---- Scenario: reference single drive(40) for merge comparison ----
    _log("ref: drive(40)")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = drive_forward(40)
        await asyncio.wait_for(step.run_step(robot), timeout=12.0)
        p = pose()
        out["merge_ref_drive"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}")

    # ---- Scenario: corner_cut_cm=5 ----
    _log("corner_cut: drive(50) + turn_right(90) + drive(30) + corner_cut_cm=5")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = smooth_path(
            drive_forward(50),
            turn_right(90),
            drive_forward(30),
            corner_cut_cm=5.0,
            correct=False,
        )
        await asyncio.wait_for(step.run_step(robot), timeout=20.0)
        p = pose()
        out["corner_cut_5cm"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={p.theta:.3f}")

    # ---- Scenario: spline=True ----
    _log("spline: drive(50) + turn_right(90) + drive(30) + spline=True")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = smooth_path(
            drive_forward(50),
            turn_right(90),
            drive_forward(30),
            spline=True,
            correct=False,
        )
        await asyncio.wait_for(step.run_step(robot), timeout=20.0)
        p = pose()
        out["spline_drive_turn_drive"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}, theta={p.theta:.3f}")

    # ---- Scenario: optimize=True with background() barrier ----
    _log("merge_barrier: drive(20) + background(noop) + drive(20) + optimize=True")
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        step = smooth_path(
            drive_forward(20),
            background(_Noop()),
            drive_forward(20),
            optimize=True,
            correct=False,
        )
        await asyncio.wait_for(step.run_step(robot), timeout=12.0)
        p = pose()
        out["merge_with_barrier"] = [p.x, p.y, p.theta]
        _log(f"  ended at x={p.x:.2f}, y={p.y:.2f}")

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
