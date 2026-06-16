from __future__ import annotations

import asyncio
import json
import os
from dataclasses import replace

from _robot_builder import build_robot, get_config

from raccoon.step.motion import LineSide, line_follow
from raccoon.testing.sim import SimRobotConfig, pose, use_scene


class CenteredSensor:
    def probabilityOfBlack(self) -> float:
        return 0.5

    def read(self) -> int:
        return 512


async def _run() -> dict[str, tuple[float, float, float]]:
    cfg = get_config("packingbot")
    if hasattr(cfg, "drivetrain"):
        cfg = replace(cfg, drivetrain="mecanum")
    else:
        cfg = SimRobotConfig(drivetrain="mecanum")
    robot = build_robot(cfg)

    sensor = CenteredSensor()

    with use_scene("scenes/empty_table.ftmap", robot=cfg, start=(50.0, 50.0, 0.0)):
        await (
            line_follow()
            .single(sensor, side=LineSide.LEFT)
            .move(forward=0.35)
            .correct_lateral()
            .distance_cm(30)
        ).run_step(robot)
        p = pose()
        return {"forward_lateral_hold": (float(p.x), float(p.y), float(p.theta))}


if __name__ == "__main__":
    os.environ.setdefault("LIBSTP_LOG_LEVEL", "warn")
    os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
    results = asyncio.run(_run())
    print("RESULTS:" + json.dumps(results))
    os._exit(0)
