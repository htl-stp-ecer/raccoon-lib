"""Standalone runner: compare side-action firing in BASELINE vs OPTIMIZED.

Runs ONE representative mission (motion legs interleaved with probe side
actions — inline, parallel ephemeral branches, and a tail parallel branch)
twice for every config: as a plain ``seq(steps)`` baseline and as
``optimize(steps).<passes>()``. Each probe records a ``start`` and a ``done``
event with the virtual sim time and the robot pose at that instant, so the
caller can verify that optimization fires the same side actions, in the same
order, at approximately the same path point, and that every probe still RUNS TO
COMPLETION (the parallel/ephemeral join the executor must honour).

Emits a single ``RESULTS:<json>`` line:

    {config: {"events": [[kind,label,t,x,y,theta], ...],
              "end": [x,y,theta]}, ...}

Invoked as a subprocess by ``test_optimize_sideaction_equivalence.py`` to
isolate C++ extension state (same pattern as ``_path_optimizer_runner.py``).
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

START = (50.0, 50.0, 0.0)


def _log(msg: str) -> None:
    sys.stderr.write(f"[sideaction_runner] {msg}\n")
    sys.stderr.flush()


def _now() -> float:
    return asyncio.get_event_loop().time()


# Module-global event log; reset per scenario run.
_EVENTS: list[list] = []


def _make_probe_cls():
    from raccoon.step.base import Step
    from raccoon.testing.sim import pose

    class _Probe(Step):
        """A non-motion side action that timestamps its start and completion.

        ``settle`` keeps the step busy so a parallel/ephemeral branch is still
        running when its motion spine ends — the case where the executor must
        JOIN (not cancel) the branch. ``resource`` is a non-drive resource so
        the probe can sit in a ``parallel()`` branch without being mistaken for
        a second motion spine.
        """

        def __init__(self, label: str, *, settle: float = 0.0, resource: str = "servo:7") -> None:
            super().__init__()
            self._label = label
            self._settle = settle
            self._resource = resource

        def required_resources(self):
            return frozenset({self._resource})

        def _generate_signature(self) -> str:
            return f"Probe({self._label})"

        async def _execute_step(self, robot) -> None:
            p = pose()
            _EVENTS.append(["start", self._label, _now(), p.x, p.y, p.theta])
            if self._settle:
                await asyncio.sleep(self._settle)
            p = pose()
            _EVENTS.append(["done", self._label, _now(), p.x, p.y, p.theta])

    return _Probe


def _augment_services(robot):
    """Add the service-locator + logger shims GenericRobot would provide.

    Mirrors ``tools/sim_run_chain._augment_services`` — some steps call
    ``robot.get_service(cls)`` (background manager) and ``robot.warn(...)``.
    """
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


def _build_steps(Probe):
    """The mission, rebuilt fresh each run (steps are single-use).

    Forward-only, ONE parallel branch, so every optimize config (merge /
    cut_corners / splinify / to_absolute / time_optimal) compiles and runs it
    identically — the point here is side-action ORDER + firing point +
    completion + that a SLOW branch doesn't make the merged path overshoot.

    ``B_branch`` carries ``settle`` so it is still running when its motion spine
    ends: that is the case the executor must handle by (a) joining it (not
    cancelling) and (b) NOT coasting the warm-started merge past the leg while
    it waits. A trailing ``Probe`` keeps the final motion a real leg the branch
    can join behind.
    """
    from raccoon.step import parallel
    from raccoon.step.motion.drive_dsl import drive_forward

    return [
        drive_forward(30),
        Probe("A_inline"),  # inline barrier between legs
        drive_forward(20),
        parallel(drive_forward(25), Probe("B_branch", settle=0.3)),  # ephemeral branch
        drive_forward(15),
        Probe("C_inline"),
    ]


def _wrap(config: str, steps):
    from raccoon.step import seq
    from raccoon.step.motion.path.optimize import optimize

    if config == "baseline":
        return seq(steps)
    opt = optimize(steps)
    if config == "merge":
        return opt  # decompose+merge are always-on
    if config == "cut_corners":
        return opt.cut_corners(5.0)
    if config == "absolute":
        return opt.to_absolute()
    if config == "splinify":
        return opt.splinify()
    if config == "absolute_splinify":
        return opt.to_absolute().splinify()
    if config == "time_optimal":
        return opt.cut_corners(5.0).time_optimal()
    msg = f"unknown config {config}"
    raise ValueError(msg)


async def _run_one(config: str, cfg, out: dict) -> None:
    from raccoon.testing.sim import pose, use_scene

    Probe = _make_probe_cls()
    _EVENTS.clear()
    robot = _augment_services(_build_robot(cfg, enable_localization=True))
    with use_scene(SCENES_DIR / "empty_table.ftmap", robot=cfg, start=START):
        steps = _build_steps(Probe)
        step = _wrap(config, steps)
        try:
            await asyncio.wait_for(step.run_step(robot), timeout=30.0)
            status = "completed"
        except Exception as exc:
            status = f"{type(exc).__name__}: {exc}"
            _log(f"  {config} FAILED: {status}")
        p = pose()
    out[config] = {
        "events": list(_EVENTS),
        "end": [p.x, p.y, p.theta],
        "status": status,
    }
    _log(
        f"  {config}: {status}  end=({p.x:.1f},{p.y:.1f},{math.degrees(p.theta):.0f}) "
        f"events={len(_EVENTS)}"
    )


async def _scenarios(config_name: str) -> None:
    # ONE config per process — rebuilding the robot / re-entering use_scene
    # several times in a single interpreter races the pybind11 teardown and
    # segfaults, so the test invokes this runner once per config (same pattern
    # as _path_optimizer_runner / optimize_validate).
    cfg = _get_config("default")
    out: dict = {}
    await _run_one(config_name, cfg, out)

    sys.stdout.write("RESULTS:" + json.dumps(out) + "\n")
    sys.stdout.flush()
    os._exit(0)


def main() -> None:
    config_name = "baseline"
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
