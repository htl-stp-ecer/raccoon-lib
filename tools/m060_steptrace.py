"""Per-step trace of M060 to confirm WHICH step times out (strafe vs sensor).

Reuses the cube-bot chain harness setup (fake transport, IR calibration,
project PID/defs, drivable scene) and runs M060's resolved top-level steps
one at a time with a short per-step timeout, printing the step signature +
result + pose. This isolates whether the residual M060 timeout is the
fixed-distance strafe (the bug we fixed) or a sensor-bounded step that can't
trigger from the wrong start position.
"""

from __future__ import annotations

import asyncio
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))
sys.path.insert(0, str(REPO_ROOT / "tests" / "python" / "sim"))

import sim_run_chain as H  # noqa: E402

# M090's end pose from the last chain run (where the harness starts M060).
START = (54.7, 93.2, math.radians(-176))
PER_STEP_TIMEOUT = 8.0


async def _main() -> None:
    from _robot_builder import build_robot  # type: ignore[import-not-found]

    from raccoon.testing.sim import pose, use_scene

    project = Path(H.DEFAULT_PROJECT)
    sys.path.insert(0, str(project))
    real_scene = project / "config" / "2026-game-table.ftmap"
    scene = H._sim_scene_path(real_scene)

    H._install_fake_transport()
    cfg = H._make_sim_config()
    robot = H._augment_services(build_robot(cfg, enable_localization=True))
    try:
        from src.hardware.defs import defs as _defs  # type: ignore[import-not-found]
        from src.hardware.robot import Robot  # type: ignore[import-not-found]

        robot.motion_pid_config = Robot.motion_pid_config
        robot.defs = _defs
    except Exception as e:
        print(f"[warn] defs/pid: {e}")
    H._calibrate_ir_sensors()

    from src.missions.m1060_grab_green_cube_mission import M060GrabGreenCubeMission

    from raccoon.step.motion import mark_heading_reference

    with use_scene(str(scene), robot=cfg, start=START):
        await asyncio.sleep(0.05)
        # Mimic the chain state: earlier missions have set a heading reference,
        # so the heading=0 holds actually execute (instead of erroring early).
        await mark_heading_reference().resolve().run_step(robot)
        seq = M060GrabGreenCubeMission().sequence().resolve()
        # Resolve to the flat list of top-level steps.
        steps = getattr(seq, "_steps", None) or getattr(seq, "steps", None) or [seq]
        print(f"M060 has {len(steps)} top-level steps; start={START}")
        for i, st in enumerate(steps):
            try:
                sig = st._generate_signature()
            except Exception:
                sig = type(st).__name__
            p0 = pose()
            try:
                await asyncio.wait_for(st.run_step(robot), timeout=PER_STEP_TIMEOUT)
                status = "ok"
            except TimeoutError:
                status = f"TIMEOUT {PER_STEP_TIMEOUT:.0f}s"
            except Exception as e:
                status = f"{type(e).__name__}: {str(e)[:50]}"
            p = pose()
            print(
                f"  [{i:2d}] {sig[:54]:54s} {status:14s} "
                f"({p0.x:.0f},{p0.y:.0f},{math.degrees(p0.theta):.0f}) -> "
                f"({p.x:.0f},{p.y:.0f},{math.degrees(p.theta):.0f})"
            )
            if status.startswith("TIMEOUT"):
                print("  -> stopping at first timeout")
                break


def main() -> None:
    os.environ["RACCOON_SIM"] = "1"
    os.environ.setdefault("LIBSTP_LOG_LEVEL", "error")
    os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
    os.environ["LIBSTP_TIMING_ENABLED"] = "0"
    asyncio.run(_main())
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
