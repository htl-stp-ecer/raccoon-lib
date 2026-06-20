"""Empirical sign-matrix probe for the sim odometry/command frame.

Runs the REAL motion steps (strafe_right, drive_forward@heading, turn_right)
through the full host-IK → mock motors → SimWorld → odometry loop and logs,
per ~0.1 s, the controller-facing odometry (robot.odometry.get_pose /
get_heading) against the sim ground-truth pose (raccoon.testing.sim.pose).

The point: lock down which axes the controller closes consistently (forward,
heading) and which one runs away (fixed-distance lateral), with the exact
sign relationship between the COMMAND, the ODOMETRY reading, and the PHYSICAL
ground-truth motion. No hardware.

Prints PROBE:<json> with, per scenario: completed?, odo trajectory samples,
ground-truth trajectory samples.
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCENES_DIR = REPO_ROOT / "scenes"
sys.path.insert(0, str(REPO_ROOT / "tests" / "python" / "sim"))

from _robot_builder import build_robot as _build_robot  # noqa: E402
from _robot_builder import get_config as _get_config  # noqa: E402

START = (50.0, 50.0, 0.0)  # cm, cm, rad


def _log(msg: str) -> None:
    sys.stderr.write(f"[sign_probe] {msg}\n")
    sys.stderr.flush()


def _augment_services(robot):
    robot._services = {}

    def get_service(cls):
        svc = robot._services.get(cls)
        if svc is None:
            svc = cls(robot)
            robot._services[cls] = svc
        return svc

    robot.get_service = get_service
    for level in ("trace", "debug", "info", "warn", "warning", "error"):
        if not hasattr(robot, level):
            setattr(robot, level, lambda *_, **__: None)
    return robot


def _odo(robot):
    p = robot.odometry.get_pose()
    return [
        round(float(p.position[0]), 4),
        round(float(p.position[1]), 4),
        round(float(robot.odometry.get_heading()), 4),
    ]


def _gt(pose_fn):
    g = pose_fn()
    return [round(float(g.x), 2), round(float(g.y), 2), round(float(g.theta), 4)]


async def _run_with_sampling(robot, step, pose_fn, timeout, out, key):
    """Run a step while sampling odo + gt every 0.1 s. Records completed flag."""
    samples = {"odo": [], "gt": []}

    async def _sampler():
        try:
            while True:
                samples["odo"].append(_odo(robot))
                samples["gt"].append(_gt(pose_fn))
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            return

    sampler = asyncio.ensure_future(_sampler())
    completed = False
    err = None
    try:
        await asyncio.wait_for(step.run_step(robot), timeout=timeout)
        completed = True
    except TimeoutError:
        completed = False
    except Exception as exc:
        err = f"{type(exc).__name__}: {exc}"
    finally:
        sampler.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await sampler

    # thin the samples (start, a few mid, end)
    def thin(lst):
        if len(lst) <= 8:
            return lst
        idx = [0, 1, 2, len(lst) // 2, -3, -2, -1]
        return [lst[i] for i in idx]

    out[key] = {
        "completed": completed,
        "error": err,
        "odo": thin(samples["odo"]),
        "gt": thin(samples["gt"]),
        "odo_final": _odo(robot),
        "gt_final": _gt(pose_fn),
    }
    _log(
        f"{key}: completed={completed} err={err} odo_final={out[key]['odo_final']} gt_final={out[key]['gt_final']}"
    )


async def _scenarios() -> None:
    from raccoon.step.motion import drive_forward, strafe_right, turn_right
    from raccoon.testing.sim import pose, use_scene

    cfg = _get_config("packingbot")
    out: dict = {}

    # --- 1. fixed-distance strafe_right(15) — the runaway ---
    async def s_strafe():
        robot = _augment_services(_build_robot(cfg, enable_localization=True))
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
            await _run_with_sampling(robot, strafe_right(15), pose, 12.0, out, "strafe_right_15")

    await s_strafe()

    # --- 2. forward 20 cm, relative heading (works baseline) ---
    async def s_fwd():
        robot = _augment_services(_build_robot(cfg, enable_localization=True))
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
            await _run_with_sampling(robot, drive_forward(20), pose, 12.0, out, "drive_forward_20")

    await s_fwd()

    # --- 3. forward 20 cm while the robot starts rotated 45° (start theta) ---
    async def s_fwd45():
        robot = _augment_services(_build_robot(cfg, enable_localization=True))
        start45 = (50.0, 50.0, math.radians(45.0))
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=start45):
            await _run_with_sampling(
                robot, drive_forward(20), pose, 12.0, out, "drive_forward_20_at45"
            )

    await s_fwd45()

    # --- 4. turn_right(45) (works baseline) ---
    async def s_turn():
        robot = _augment_services(_build_robot(cfg, enable_localization=True))
        with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
            await _run_with_sampling(robot, turn_right(45), pose, 12.0, out, "turn_right_45")

    await s_turn()

    print("PROBE:" + json.dumps(out))
    sys.stdout.flush()


def main() -> None:
    os.environ.setdefault("LIBSTP_LOG_LEVEL", "warn")
    os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
    os.environ["LIBSTP_TIMING_ENABLED"] = "0"
    asyncio.run(_scenarios())
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
