"""Standalone runner for goto / GotoWaypoints end-to-end sim tests.

Drives the real closed loop — ``Goto.on_update`` reads
``robot.localization.get_pose()`` (the particle filter), commands a body
velocity, ``Drive`` runs host IK → mock motors → ``SimWorld`` integrates →
odometry → localization — entirely under the mock bundle, no hardware.

Frame note (confirmed by ``_localization_cross_motion_runner.py``):
``localization.get_pose()`` is expressed RELATIVE TO THE START pose, so the
localization origin is ``(0, 0, 0)`` no matter where ``use_scene(start=…)``
places the robot on the table. Goto targets are therefore given in that
start-relative frame; the sim ground-truth ``pose()`` equals the target plus
the start offset (here 50 cm, 50 cm).

Sub-processed + ``os._exit(0)`` to dodge the pybind11 / mock-HAL teardown
races the other sim runners work around. Prints one ``RESULTS:<json>`` line.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SCENES_DIR = REPO_ROOT / "scenes"
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _robot_builder import build_robot as _build_robot  # noqa: E402
from _robot_builder import get_config as _get_config  # noqa: E402

START = (50.0, 50.0, 0.0)  # cm, cm, rad — robot start on the empty table


def _log(msg: str) -> None:
    sys.stderr.write(f"[goto_runner] {msg}\n")
    sys.stderr.flush()


async def _run(out: dict, key: str, label: str, coro) -> None:
    _log(label)
    try:
        await coro
    except Exception as exc:
        out[key] = {"error": f"{type(exc).__name__}: {exc}"}
        _log(f"  failed: {type(exc).__name__}: {exc}")


def _record(out: dict, key: str, robot, pose_fn) -> None:
    loc = robot.localization.get_pose()
    gt = pose_fn()
    out[key] = {
        "loc": [float(loc.position[0]), float(loc.position[1]), float(loc.heading)],
        "gt_m": [float(gt.x / 100.0), float(gt.y / 100.0), float(gt.theta)],
    }
    _log(f"  {key}: loc={out[key]['loc']} gt_m={out[key]['gt_m']}")


async def _scenarios(config_name: str) -> None:
    from raccoon.step.motion import drive_forward, optimize, turn_right
    from raccoon.step.motion.goto import Goto
    from raccoon.testing.sim import pose, use_scene

    cfg = _get_config(config_name)
    robot = _build_robot(cfg, enable_localization=True)
    is_mecanum = getattr(cfg, "drivetrain", "diff") == "mecanum"
    out: dict = {}

    # --- goto forward 20 cm (both drivetrains) ---
    async def _forward() -> None:
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
            await asyncio.wait_for(
                Goto(x_m=0.20, y_m=0.0, theta_rad=0.0).run_step(robot), timeout=15.0
            )
            _record(out, "goto_forward", robot, pose)

    await _run(out, "goto_forward", "goto forward 0.20 m", _forward())

    # --- goto rotate-in-place + drive: target ahead, face +45° (both) ---
    async def _with_heading() -> None:
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
            await asyncio.wait_for(
                Goto(x_m=0.20, y_m=0.0, theta_rad=math.radians(45.0)).run_step(robot),
                timeout=15.0,
            )
            _record(out, "goto_with_heading", robot, pose)

    await _run(out, "goto_with_heading", "goto forward + face 45°", _with_heading())

    # --- mecanum-only: pure strafe + diagonal (needs vy) ---
    if is_mecanum:

        async def _strafe() -> None:
            with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
                # Pure holonomic strafe converges on the proportional goto law,
                # but the P-controller approaches the target asymptotically and
                # the empty-table sim has wheel saturation/friction, so the last
                # centimetre is slow — give it a generous budget.
                await asyncio.wait_for(
                    Goto(x_m=0.0, y_m=0.15, theta_rad=0.0).run_step(robot),
                    timeout=60.0,
                )
                _record(out, "goto_strafe", robot, pose)

        await _run(out, "goto_strafe", "goto pure strafe +0.15 m (mecanum)", _strafe())

        async def _diag() -> None:
            with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
                await asyncio.wait_for(
                    Goto(x_m=0.20, y_m=0.10, theta_rad=0.0).run_step(robot), timeout=30.0
                )
                _record(out, "goto_diagonal", robot, pose)

        await _run(out, "goto_diagonal", "goto diagonal (0.20, 0.10) (mecanum)", _diag())

    # --- optimize([...]).to_absolute() → GotoWaypoints end-to-end ---
    async def _to_absolute() -> None:
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
            step = optimize([drive_forward(30), turn_right(90), drive_forward(20)]).to_absolute()
            await asyncio.wait_for(step.run_step(robot), timeout=40.0)
            _record(out, "optimize_to_absolute", robot, pose)

    await _run(
        out,
        "optimize_to_absolute",
        "optimize([drive(30), turn_right(90), drive(20)]).to_absolute()",
        _to_absolute(),
    )

    # --- optimize([...]).splinify() → odometry-driven SplineMotion ---
    async def _splinify() -> None:
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
            step = optimize([drive_forward(30), turn_right(90), drive_forward(20)]).splinify()
            await asyncio.wait_for(step.run_step(robot), timeout=20.0)
            _record(out, "optimize_splinify", robot, pose)

    await _run(
        out,
        "optimize_splinify",
        "optimize([drive(30), turn_right(90), drive(20)]).splinify()",
        _splinify(),
    )

    # --- direct spline() through waypoints (forward_cm, left_cm) ---
    async def _direct_spline() -> None:
        from raccoon.step.motion import spline

        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
            step = spline((30, 0), (50, -20), (50, -40))
            await asyncio.wait_for(step.run_step(robot), timeout=20.0)
            _record(out, "direct_spline", robot, pose)

    await _run(out, "direct_spline", "spline((30,0),(50,-20),(50,-40))", _direct_spline())

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


if __name__ == "__main__":
    main()
