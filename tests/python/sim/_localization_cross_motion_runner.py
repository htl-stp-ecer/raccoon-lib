"""Standalone runner for the localization cross-motion smoke test.

Runs two back-to-back ``drive_forward`` steps with ``enable_localization=True``.
Each motion's ``start()`` calls ``odometry.reset()`` (Phase ≤3 behaviour).
The Phase-2 Localization service must absorb that reset so the world pose
keeps growing across both motions.

Sub-processed to dodge the same pybind11 / mock-HAL teardown races that
``_drive_mission_runner.py`` works around.
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SCENES_DIR = REPO_ROOT / "scenes"


async def _scenario() -> dict:
    # Make our local _robot_builder importable.
    sys.path.insert(0, str(Path(__file__).parent))
    from raccoon.step.motion.drive_dsl import drive_forward  # noqa: I001
    from raccoon.testing.sim import pose, use_scene
    from _robot_builder import build_robot, get_config  # type: ignore[import-not-found]

    cfg = get_config("default")
    robot = build_robot(cfg, enable_localization=True)
    assert robot.localization is not None, "enable_localization=True must wire the service"

    out: dict[str, object] = {}

    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        # First motion: 20 cm forward.
        await asyncio.wait_for(drive_forward(cm=20.0).run_step(robot), timeout=8.0)
        p1 = robot.localization.get_pose()
        out["after_motion_1"] = [float(p1.position[0]), float(p1.position[1]), float(p1.heading)]
        sim_p1 = pose()
        out["sim_after_motion_1"] = [
            float(sim_p1.x / 100.0),
            float(sim_p1.y / 100.0),
            float(sim_p1.theta),
        ]

        # Second motion: another 20 cm forward. Between the two motions
        # ``drive_forward.start()`` resets the underlying odometry; the
        # localization world pose must keep advancing.
        await asyncio.wait_for(drive_forward(cm=20.0).run_step(robot), timeout=8.0)
        p2 = robot.localization.get_pose()
        out["after_motion_2"] = [float(p2.position[0]), float(p2.position[1]), float(p2.heading)]
        sim_p2 = pose()
        out["sim_after_motion_2"] = [
            float(sim_p2.x / 100.0),
            float(sim_p2.y / 100.0),
            float(sim_p2.theta),
        ]

    sys.stdout.write("RESULTS:" + json.dumps(out) + "\n")
    sys.stdout.flush()
    os._exit(0)


def main() -> None:
    try:
        asyncio.run(_scenario())
    except BaseException as e:
        import traceback

        sys.stderr.write(f"ERR: {type(e).__name__}: {e}\n")
        traceback.print_exc(file=sys.stderr)
        sys.stderr.flush()
        os._exit(2)


if __name__ == "__main__":
    main()
