"""Validate optimize() passes against the cube-bot mission chain in sim.

For a chosen optimize CONFIG, runs the full cube-bot mission chain with every
mission's sequence wrapped in ``optimize(...config...)`` (best-effort: if a
pass can't compile a mission's path it raises PathBuildError and we fall back
to the plain sequence, recorded as "fallback"). Records each mission's
end pose + status. A separate baseline run (``--config baseline``, plain seq)
is the reference; ``compare`` mode diffs a config's endpoints against it.

This answers "does the robot still correctly & accurately do its missions
while optimized" for: merge/decompose (always-on), cut_corners, to_absolute
(absolute), absolute_heading, splinify, and to_absolute+splinify — plus the
executor's warm-start (velocity carried across merged same-type legs).

Usage::

    .venv-test/bin/python tools/optimize_validate.py --config baseline --out opt_val
    .venv-test/bin/python tools/optimize_validate.py --config merge    --out opt_val
    .venv-test/bin/python tools/optimize_validate.py --config cut_corners --out opt_val
    ...
    .venv-test/bin/python tools/optimize_validate.py --compare opt_val
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))
sys.path.insert(0, str(REPO_ROOT / "tests" / "python" / "sim"))

import sim_run_chain as H  # noqa: E402

START = (42.8, 25.63, 90.0)
TIMEOUT = 45.0

CONFIGS = [
    "baseline",
    "merge",
    "cut_corners",
    "absolute",
    "absolute_heading",
    "splinify",
    "absolute_splinify",
]


def _wrap(seq_steps, config, optimize):
    """Wrap a mission's step list in optimize() per config. Returns (step, note)."""
    if config == "baseline":
        return None, "seq"
    opt = optimize(seq_steps)
    if config == "merge":
        pass  # decompose + merge only (always-on)
    elif config == "cut_corners":
        opt = opt.cut_corners(5.0)
    elif config == "absolute":
        opt = opt.to_absolute()
    elif config == "absolute_heading":
        opt = opt.absolute_heading()
    elif config == "splinify":
        opt = opt.splinify()
    elif config == "absolute_splinify":
        opt = opt.to_absolute().splinify()
    else:
        msg = f"unknown config {config}"
        raise ValueError(msg)
    return opt, config


async def _run(config: str, out_dir: Path, solo: str | None = None) -> dict:
    from _robot_builder import build_robot  # type: ignore[import-not-found]

    from raccoon.testing.sim import pose, use_scene

    try:
        from raccoon.step.motion import optimize
    except ImportError:
        from raccoon.step.motion.path.optimize import optimize

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
    missions = H._ordered_missions(project)
    # The cube-bot run order excludes M060 (not in Robot.missions); drop the
    # appended-unlisted ones so we validate the REAL chain.
    missions = [(nm, cls) for nm, cls in missions if "M060" not in nm]

    start = (START[0], START[1], math.radians(START[2]))
    segments: list[dict] = []

    with use_scene(str(scene), robot=cfg, start=start):
        await asyncio.sleep(0.05)
        for nm, cls in missions:
            # Build the mission, then wrap per config. In --solo mode only the
            # named mission is optimized (the rest run plain seq), so it starts
            # from the correct baseline pose — isolating true per-mission drift
            # from chain-compounding.
            effective = config if (solo is None or solo in nm) else "baseline"
            try:
                raw = cls().sequence()
                step, note = _wrap(raw, effective, optimize)
                run_step = (step if step is not None else raw).resolve()
            except Exception as e:
                # optimize() couldn't compile this path — fall back to plain seq.
                note = f"fallback({type(e).__name__})"
                run_step = cls().sequence().resolve()
            try:
                await asyncio.wait_for(run_step.run_step(robot), timeout=TIMEOUT)
                status = "completed"
            except TimeoutError:
                status = "TIMEOUT"
            except Exception as e:
                status = f"{type(e).__name__}: {str(e)[:70]}"
            p = pose()
            rec = {
                "name": nm,
                "note": note,
                "status": status,
                "end": [round(p.x, 2), round(p.y, 2), round(math.degrees(p.theta), 1)],
            }
            segments.append(rec)
            print(
                f"  [{config:18s}] {nm:34s} {note:22s} {status:12s} "
                f"-> ({p.x:.1f},{p.y:.1f},{math.degrees(p.theta):.0f})"
            )

    result = {"config": config, "solo": solo, "segments": segments}
    out_dir.mkdir(parents=True, exist_ok=True)
    fname = f"solo_{solo}_{config}.json" if solo else f"{config}.json"
    (out_dir / fname).write_text(json.dumps(result, indent=2))
    return result


def _compare(out_dir: Path) -> None:
    base_f = out_dir / "baseline.json"
    if not base_f.exists():
        print("no baseline.json — run --config baseline first")
        return
    base = {s["name"]: s for s in json.loads(base_f.read_text())["segments"]}
    print(f"\n{'mission':34s} {'config':18s} {'note':20s} {'status':10s} {'Δend (cm, °)':>16s}")
    print("-" * 110)
    for config in CONFIGS:
        if config == "baseline":
            continue
        f = out_dir / f"{config}.json"
        if not f.exists():
            continue
        for s in json.loads(f.read_text())["segments"]:
            b = base.get(s["name"])
            if not b:
                continue
            be, se = b["end"], s["end"]
            dxy = math.hypot(se[0] - be[0], se[1] - be[1])
            dth = abs((se[2] - be[2] + 180) % 360 - 180)
            flag = ""
            if s["status"] != "completed":
                flag = "  <<< NOT COMPLETED"
            elif dxy > 8.0 or dth > 12.0:
                flag = "  <<< ENDPOINT DRIFT"
            print(
                f"{s['name']:34s} {config:18s} {s['note']:20s} {s['status']:10s} "
                f"{dxy:7.1f}cm {dth:5.1f}°{flag}"
            )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", choices=CONFIGS)
    ap.add_argument("--out", default=str(REPO_ROOT / "opt_val"))
    ap.add_argument("--compare", metavar="OUTDIR")
    ap.add_argument(
        "--solo",
        metavar="MISSION",
        help="optimize ONLY this mission (substring match); run the rest plain "
        "seq so it starts from the baseline pose — isolates per-mission drift.",
    )
    args = ap.parse_args()

    if args.compare:
        _compare(Path(args.compare))
        return

    os.environ["RACCOON_SIM"] = "1"
    os.environ.setdefault("LIBSTP_LOG_LEVEL", "error")
    os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
    os.environ["LIBSTP_TIMING_ENABLED"] = "0"
    asyncio.run(_run(args.config, Path(args.out), solo=args.solo))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
