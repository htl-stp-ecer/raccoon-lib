"""Isolated debug runner for corner_cut_5cm scenario.

Run with:
    python _debug_corner_cut.py [--after-ref]

--after-ref: also run the three preceding scenarios first (to test cross-scenario leakage)
"""

from __future__ import annotations

import asyncio
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SCENES_DIR = REPO_ROOT / "scenes"
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _robot_builder import build_robot as _build_robot  # noqa: E402
from _robot_builder import get_config as _get_config  # noqa: E402


def _log(msg: str) -> None:
    sys.stderr.write(f"[debug_cc] {msg}\n")
    sys.stderr.flush()


async def _run_preceding_scenarios(robot, cfg) -> None:
    """Run merge_two_drives + merge_ref_drive + reference_drive_turn_drive first."""
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.smooth_path import smooth_path
    from raccoon.step.motion.turn_dsl import turn_right
    from raccoon.testing.sim import pose, use_scene

    _log("=== Running preceding scenarios (cross-scenario leakage test) ===")

    async def _s1():
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
            step = smooth_path(drive_forward(20), drive_forward(20), optimize=True)
            await asyncio.wait_for(step.run_step(robot), timeout=12.0)
            p = pose()
            _log(f"  merge_two_drives ended at x={p.x:.3f}, y={p.y:.3f}, theta={p.theta:.4f}")

    await _s1()

    async def _s2():
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
            step = drive_forward(40)
            await asyncio.wait_for(step.run_step(robot), timeout=12.0)
            p = pose()
            _log(f"  merge_ref_drive ended at x={p.x:.3f}, y={p.y:.3f}, theta={p.theta:.4f}")

    await _s2()

    async def _s3():
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
            step = smooth_path(drive_forward(50), turn_right(90), drive_forward(30))
            await asyncio.wait_for(step.run_step(robot), timeout=20.0)
            p = pose()
            _log(
                f"  reference_drive_turn_drive ended at x={p.x:.3f}, y={p.y:.3f}, theta={p.theta:.4f}"
            )

    await _s3()

    _log("=== Preceding scenarios done ===")


async def _run_corner_cut(robot, cfg) -> None:
    from raccoon.step.motion.drive_dsl import drive_forward
    from raccoon.step.motion.smooth_path import smooth_path
    from raccoon.step.motion.turn_dsl import turn_right
    from raccoon.testing.sim import pose, use_scene

    _log("=== Running corner_cut_5cm in isolation ===")

    async def _cc():
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
            step = smooth_path(
                drive_forward(50),
                turn_right(90),
                drive_forward(30),
                corner_cut_cm=5.0,
            )

            # Print compiled plan info
            _log(f"  absolute_plan: {step._absolute_plan}")
            _log(f"  fallback_reason: {step._absolute_bridge_fallback_reason}")
            if step._absolute_plan is not None:
                _log(f"  absolute nodes ({len(step._absolute_plan.nodes)}):")
                for i, n in enumerate(step._absolute_plan.nodes):
                    _log(f"    [{i}] {n}")
            _log(f"  passes_applied: {step._plan.passes_applied}")
            _log("  legacy IR nodes:")
            for i, n in enumerate(step._plan.nodes):
                _log(f"    [{i}] {n}")

            # Print pre-run pose
            odom_pose = robot.odometry.get_pose()
            _log(f"  PRE-RUN: odom pos={list(odom_pose.position)}, heading={odom_pose.heading:.4f}")
            try:
                loc_pose = robot.localization.get_pose()
                _log(
                    f"  PRE-RUN: loc  pos={list(loc_pose.position)}, heading={loc_pose.heading:.4f}"
                )
            except Exception as e:
                _log(f"  PRE-RUN: loc unavailable: {e}")

            await asyncio.wait_for(step.run_step(robot), timeout=20.0)
            p = pose()
            _log(f"  RESULT: x={p.x:.3f}, y={p.y:.3f}, theta={p.theta:.4f}")
            _log(f"  Expected: x≈100, y≈20, theta≈{-3.14159/2:.4f}")

    await _cc()


async def main() -> None:
    run_preceding = "--after-ref" in sys.argv

    os.environ.setdefault("LIBSTP_LOG_LEVEL", "warn")
    os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
    os.environ["LIBSTP_TIMING_ENABLED"] = "0"

    cfg = _get_config("default")
    robot = _build_robot(cfg)

    if run_preceding:
        await _run_preceding_scenarios(robot, cfg)

    await _run_corner_cut(robot, cfg)

    _log("=== Done ===")
    os._exit(0)


if __name__ == "__main__":
    asyncio.run(main())
