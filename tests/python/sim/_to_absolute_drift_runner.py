"""Standalone runner: DEMONSTRATE that ``to_absolute()`` corrects odometry drift.

The mock sim is normally driftless (the dead-reckoned odometry equals the
ground-truth pose), so a drift-correcting pass has nothing to correct and looks
identical to the plain ``seq()`` baseline. This runner injects a per-axis
ODOMETRY drift (``sim.mock.set_odometry_drift``) that biases ONLY the
dead-reckoned odometry the relative motion controllers close their loop on — the
physical ground-truth pose is untouched — and models ABSOLUTE LANDMARK sightings
by periodically snapping the localization particle filter onto the ground truth
(``localization.observe`` with the start-relative ground truth from
``sim.mock.read_odometry``, which reads the unbiased physical pose).

It runs the SAME short motion path three ways and reports the final GROUND-TRUTH
pose of each:

- ``ref``      — plain ``seq()``, NO drift → the intended (target) endpoint.
- ``baseline`` — plain ``seq()`` WITH drift → the relative controllers believe
                 the biased odometry, so the robot physically drifts off target.
- ``absolute`` — ``optimize(steps).to_absolute()`` WITH drift → ``GotoWaypoints``
                 regulates on the landmark-corrected localization, so the robot
                 stays on the intended path despite the same odometry bias.

Run WALL-CLOCK (not fast-time): the localization worker thread integrates
odometry on the real clock, so under fast-time it freezes and never tracks. The
path is short enough that wall-clock is quick, and the drift signal (tens of cm)
dwarfs the wall-clock dt jitter (~1 cm).

Emits one ``RESULTS:<json>`` line: ``{config: {"end_gt": [x_cm,y_cm,theta_rad],
"status": ...}}``. Invoked once per config as a subprocess by
``test_to_absolute_drift_correction.py`` to isolate C++ extension state (same
pattern as ``_sideaction_compare_runner``).
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

import contextlib  # noqa: E402

from _robot_builder import build_robot as _build_robot  # noqa: E402

START = (50.0, 50.0, 0.0)

# Per-axis multiplicative odometry drift. 0.85 = the dead-reckoned forward/strafe
# odometry under-reports by 15 %, so a relative controller that stops when its
# (biased) odometry reads N cm has physically travelled ~N/0.85 cm. Heading is
# left unbiased so turns stay accurate and the demo isolates translational drift.
DRIFT = (0.85, 0.85, 1.0)

# Landmark resync period (real seconds). Each tick snaps localization onto the
# ground truth — a modelled absolute-landmark sighting.
RESYNC_PERIOD_S = 0.25


def _log(msg: str) -> None:
    sys.stderr.write(f"[drift_runner] {msg}\n")
    sys.stderr.flush()


def _augment_services(robot):
    """Service-locator + logger shims GenericRobot would provide (mirrors the
    other sim runners)."""
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
            setattr(robot, level, lambda *a, **k: None)
    return robot


def _mecanum_config():
    """A mecanum SimRobotConfig (like the real cube-bot), so ``GotoWaypoints``
    can drive holonomically to lateral targets — a differential bot can't strafe
    to a waypoint off its heading."""
    from raccoon.testing.sim import SimRobotConfig

    return SimRobotConfig(drivetrain="mecanum")


def _build_steps():
    """A short, purely-geometric L-path (forward / turn / forward). Every leg has
    a known endpoint, so ``to_absolute`` folds the whole path into one
    ``GotoWaypoints`` run regulated on localization. Built on a mecanum bot so
    GotoWaypoints can drive holonomically to the lateral (strafe) waypoint — a
    differential bot can't reach a target off its heading. The drift biases BOTH
    translational axes, so the relative baseline drifts in 2-D while heading
    (unbiased) stays accurate."""
    from raccoon.step.motion.drive_dsl import drive_forward, strafe_right

    return [
        drive_forward(50),
        strafe_right(40),
        drive_forward(40),
    ]


def _wrap(config: str, steps):
    from raccoon.step import seq
    from raccoon.step.motion.path.optimize import optimize

    if config in ("ref", "baseline"):
        return seq(steps)
    if config == "absolute":
        return optimize(steps).to_absolute()
    msg = f"unknown config {config}"
    raise ValueError(msg)


def _make_observation(gt):
    """Build a localization Observation from a start-relative ground-truth tuple
    ``(x_m, y_m, heading_rad, yaw_rate)`` (as returned by sim.mock.read_odometry,
    which reads the UNBIASED physical pose)."""
    from raccoon.foundation import Pose
    from raccoon.localization import Observation

    p = Pose()
    p.position = [float(gt[0]), float(gt[1]), 0.0]
    p.heading = float(gt[2])
    # Tight sigma on x/y (a confident landmark fix), looser on heading.
    return Observation(pose=p, sigma=(0.01, 0.01, 0.02))


async def _resync_loop(robot, stop: asyncio.Event) -> None:
    """Periodically snap localization onto the ground truth — a modelled absolute
    landmark sighting. Only meaningful for the ``absolute`` run (the relative
    controllers never read localization), but harmless to run always."""
    import raccoon.sim as S

    while not stop.is_set():
        try:
            await asyncio.wait_for(stop.wait(), timeout=RESYNC_PERIOD_S)
            break  # stop was set
        except TimeoutError:
            pass
        gt = S.mock.read_odometry()
        try:
            robot.localization.observe(_make_observation(gt))
        except Exception as exc:  # never let the resync kill the run
            _log(f"observe failed: {exc}")


async def _trajectory_loop(traj: list, stop: asyncio.Event) -> None:
    """Poll the GROUND-TRUTH pose into ``traj`` for rendering."""
    from raccoon.testing.sim import pose

    while not stop.is_set():
        p = pose()
        traj.append([p.x, p.y, p.theta])
        try:
            await asyncio.wait_for(stop.wait(), timeout=0.05)
            break
        except TimeoutError:
            pass


async def _run_one(config: str, cfg, out: dict) -> None:
    import raccoon.sim as S
    from raccoon.testing.sim import pose, use_scene

    robot = _augment_services(_build_robot(cfg, enable_localization=True))
    traj: list = []
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
        # ref runs driftless (the intended target); baseline/absolute drift.
        if config != "ref":
            S.mock.set_odometry_drift(*DRIFT)

        stop = asyncio.Event()
        resync = asyncio.create_task(_resync_loop(robot, stop))
        tloop = asyncio.create_task(_trajectory_loop(traj, stop))
        steps = _build_steps()
        step = _wrap(config, steps)
        try:
            await asyncio.wait_for(step.run_step(robot), timeout=60.0)
            status = "completed"
        except Exception as exc:
            status = f"{type(exc).__name__}: {exc}"
            _log(f"  {config} FAILED: {status}")
        finally:
            stop.set()
            for t in (resync, tloop):
                with contextlib.suppress(Exception):
                    await asyncio.wait_for(t, timeout=2.0)
        p = pose()  # GROUND-TRUTH table pose (cm, rad)
        traj.append([p.x, p.y, p.theta])

    out[config] = {"end_gt": [p.x, p.y, p.theta], "status": status, "traj": traj}
    _log(f"  {config}: {status}  end_gt=({p.x:.1f},{p.y:.1f},{math.degrees(p.theta):.0f})")


async def _scenarios(config_name: str) -> None:
    cfg = _mecanum_config()
    out: dict = {}
    await _run_one(config_name, cfg, out)
    sys.stdout.write("RESULTS:" + json.dumps(out) + "\n")
    sys.stdout.flush()
    os._exit(0)


def main() -> None:
    config_name = "ref"
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
    os._exit(0)


if __name__ == "__main__":
    main()
