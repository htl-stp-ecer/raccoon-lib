#!/usr/bin/env python3
"""Render BASELINE-sim vs OPTIMIZED-sim trajectories per cube-bot mission.

Runs the cube-bot mission chain twice in the mock sim — once as the plain
``seq()`` the mission declares (baseline), once with every mission wrapped in
``optimize(<config>)`` — capturing the full per-tick trajectory of each run.
For every mission it then draws ONE PNG on the game-table ftmap with both
paths overlaid (baseline = grey, optimized = orange), plus a combined overview.
This lets the optimizer's effect on each mission be validated by eye:
"original simulation vs optimize".

The two runs execute in separate subprocesses (each ``os._exit``s to dodge the
pybind11 / mock-HAL teardown races) and dump their trajectory to JSON; the
parent process loads both and renders. Reuses ``sim_run_chain``'s sim setup and
render primitives, and ``optimize_validate``'s per-config ``optimize()`` wrap.

Run under the mock test venv:

    .venv-test/bin/python tools/optimize_compare_viz.py --config absolute_heading
    .venv-test/bin/python tools/optimize_compare_viz.py --config cut_corners --missions M007,M010
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))
sys.path.insert(0, str(REPO_ROOT / "tests" / "python" / "sim"))

import optimize_validate as OV  # noqa: E402
import sim_run_chain as H  # noqa: E402

BASELINE_COLOR = "#9e9e9e"
OPT_COLOR = "#ff7f0e"


# ---------------------------------------------------------------------------
# Worker: run ONE variant of the chain and dump its trajectory to JSON.
# ---------------------------------------------------------------------------


async def _simulate_variant(project: Path, scene: Path, start, timeout: float, config: str):
    """Run the full chain; wrap each mission in optimize(config) unless baseline."""
    from _robot_builder import build_robot  # type: ignore[import-not-found]

    from raccoon.testing.sim import pose, use_scene

    optimize = None
    if config != "baseline":
        try:
            from raccoon.step.motion import optimize as _opt
        except ImportError:
            from raccoon.step.motion.path.optimize import optimize as _opt
        optimize = _opt

    H._install_fake_transport()
    cfg = H._make_sim_config()
    robot = H._augment_services(build_robot(cfg, enable_localization=True))
    try:
        from src.hardware.defs import defs as _defs  # type: ignore[import-not-found]
        from src.hardware.robot import Robot  # type: ignore[import-not-found]

        robot.motion_pid_config = Robot.motion_pid_config
        robot.defs = _defs
    except Exception as e:
        print(f"    [warn] defs/pid: {e}")
    H._calibrate_ir_sensors()

    # Real run order, minus M060 (not in Robot.missions — see optimize_validate).
    missions = [(nm, cls) for nm, cls in H._ordered_missions(project) if "M060" not in nm]

    traj: list[dict] = []
    segments: list[dict] = []
    running = {"on": True, "mi": -1}

    async def _poll():
        while running["on"]:
            p = pose()
            traj.append({"x": p.x, "y": p.y, "theta": p.theta, "mi": running["mi"]})
            await asyncio.sleep(0.04)

    with use_scene(str(scene), robot=cfg, start=start):
        poller = asyncio.create_task(_poll())
        await asyncio.sleep(0.05)
        for mi, (nm, cls) in enumerate(missions):
            running["mi"] = mi
            i0 = len(traj)
            note = "seq"
            try:
                raw = cls().sequence()
                if optimize is not None:
                    step, note = OV._wrap(raw, config, optimize)
                    run_step = (step if step is not None else raw).resolve()
                else:
                    run_step = raw.resolve()
            except Exception as e:
                note = f"fallback({type(e).__name__})"
                run_step = cls().sequence().resolve()
            try:
                await asyncio.wait_for(run_step.run_step(robot), timeout=timeout)
                status = "completed"
            except TimeoutError:
                status = "TIMEOUT"
            except Exception as e:
                status = f"{type(e).__name__}: {str(e)[:50]}"
            p = pose()
            print(
                f"  [{config:16s}] {nm:34s} {note:20s} {status:12s} "
                f"-> ({p.x:.1f},{p.y:.1f},{math.degrees(p.theta):.0f})"
            )
            segments.append(
                {
                    "name": nm,
                    "note": note,
                    "status": status,
                    "i0": i0,
                    "i1": len(traj),
                    "end": [round(p.x, 2), round(p.y, 2), round(math.degrees(p.theta), 1)],
                }
            )
        running["on"] = False
        await asyncio.gather(poller, return_exceptions=True)

    return {
        "config": config,
        "traj": traj,
        "segments": segments,
        "start": list(start),
        "geometry": H._robot_geometry(),
    }


def _run_worker(
    config: str,
    project: Path,
    scene_arg: str | None,
    start_str: str,
    timeout: float,
    out_json: Path,
) -> None:
    project = project.resolve()
    sys.path.insert(0, str(project))
    os.chdir(project)
    os.environ.setdefault("LIBSTP_LOG_LEVEL", "warn")
    os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
    os.environ["LIBSTP_TIMING_ENABLED"] = "0"
    os.environ["RACCOON_SIM"] = "1"

    scene = Path(scene_arg) if scene_arg else (project / H.DEFAULT_SCENE)
    sim_scene = H._sim_scene_path(scene)
    sx, sy, sdeg = (float(v) for v in start_str.split(","))
    start = (sx, sy, math.radians(sdeg))

    result = asyncio.run(_simulate_variant(project, sim_scene, start, timeout, config))
    out_json.write_text(json.dumps(result))
    print(f"  wrote {out_json}")
    sys.stdout.flush()
    os._exit(0)


# ---------------------------------------------------------------------------
# Render: overlay baseline vs optimized.
# ---------------------------------------------------------------------------


def _seg_pts(result: dict, mi: int) -> list[dict]:
    return [t for t in result["traj"] if t["mi"] == mi]


def _delta(a, b) -> tuple[float, float]:
    dxy = math.hypot(a[0] - b[0], a[1] - b[1])
    dth = abs((a[2] - b[2] + 180) % 360 - 180)
    return dxy, dth


def _render(
    base: dict, opt: dict, config: str, segs, table, out_dir: Path, mission_filter: set[str] | None
) -> list[Path]:
    import matplotlib as mpl

    mpl.use("Agg")
    import matplotlib.pyplot as plt

    written: list[Path] = []
    geom = base.get("geometry")
    bsegs = base["segments"]
    osegs = {s["name"]: s for s in opt["segments"]}

    # --- combined overview ---
    fig, ax = plt.subplots(figsize=(15, 7.5))
    H._draw_table(ax, segs, table)
    ax.plot(
        [t["x"] for t in base["traj"]],
        [t["y"] for t in base["traj"]],
        "-",
        color=BASELINE_COLOR,
        lw=1.6,
        alpha=0.9,
        label="baseline seq",
        zorder=4,
    )
    ax.plot(
        [t["x"] for t in opt["traj"]],
        [t["y"] for t in opt["traj"]],
        "-",
        color=OPT_COLOR,
        lw=1.6,
        alpha=0.9,
        label=f"optimize({config})",
        zorder=5,
    )
    s = base["start"]
    ax.plot(s[0], s[1], "*", color="black", ms=15, label="start", zorder=7)
    ax.set_title(f"cube-bot chain — baseline vs optimize({config})", fontsize=13, fontweight="bold")
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=9)
    fig.tight_layout()
    out = out_dir / "overview.png"
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    written.append(out)
    print(f"wrote {out}")

    # --- per-mission overlays ---
    for mi, bseg in enumerate(bsegs):
        name = bseg["name"]
        if mission_filter and not any(f in name for f in mission_filter):
            continue
        oseg = osegs.get(name, {})
        bpts = _seg_pts(base, mi)
        # match by name in the optimized run (same order, but be robust)
        omi = next((i for i, s in enumerate(opt["segments"]) if s["name"] == name), mi)
        opts = _seg_pts(opt, omi)

        fig, ax = plt.subplots(figsize=(15, 7.5))
        H._draw_table(ax, segs, table)
        # faint full context
        ax.plot(
            [t["x"] for t in base["traj"]],
            [t["y"] for t in base["traj"]],
            "-",
            color="#e0e0e0",
            lw=0.6,
            zorder=2,
        )

        if bpts:
            ax.plot(
                [p["x"] for p in bpts],
                [p["y"] for p in bpts],
                "-",
                color=BASELINE_COLOR,
                lw=2.4,
                alpha=0.95,
                label="baseline seq",
                zorder=4,
            )
        if opts:
            ax.plot(
                [p["x"] for p in opts],
                [p["y"] for p in opts],
                "-",
                color=OPT_COLOR,
                lw=2.4,
                alpha=0.95,
                label=f"optimize({config})",
                zorder=5,
            )

        # start (green) shared; end poses for both
        if bpts:
            H._draw_robot(
                ax,
                bpts[0]["x"],
                bpts[0]["y"],
                bpts[0]["theta"],
                geom,
                alpha=1.0,
                body_color="green",
                labels=True,
            )
            ax.plot(bpts[0]["x"], bpts[0]["y"], "o", color="green", ms=9, label="start", zorder=7)
            H._draw_robot(
                ax,
                bpts[-1]["x"],
                bpts[-1]["y"],
                bpts[-1]["theta"],
                geom,
                alpha=0.9,
                body_color=BASELINE_COLOR,
            )
            ax.plot(
                bpts[-1]["x"],
                bpts[-1]["y"],
                "s",
                color=BASELINE_COLOR,
                ms=10,
                markeredgecolor="black",
                label="baseline end",
                zorder=7,
            )
        if opts:
            H._draw_robot(
                ax,
                opts[-1]["x"],
                opts[-1]["y"],
                opts[-1]["theta"],
                geom,
                alpha=0.9,
                body_color=OPT_COLOR,
            )
            ax.plot(
                opts[-1]["x"],
                opts[-1]["y"],
                "D",
                color=OPT_COLOR,
                ms=9,
                markeredgecolor="black",
                label="optimize end",
                zorder=7,
            )

        bend, oend = bseg.get("end"), oseg.get("end")
        dtxt = ""
        if bend and oend:
            dxy, dth = _delta(oend, bend)
            dtxt = f"   Δend = {dxy:.1f} cm / {dth:.1f}°"
        title = (
            f"[{mi}] {name}\n"
            f"baseline [{bseg['status']}] end={bend}   |   "
            f"optimize({config}) [{oseg.get('status','?')}] note={oseg.get('note','?')} "
            f"end={oend}{dtxt}"
        )
        ax.set_title(title, fontsize=10.5, fontweight="bold")
        ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
        fig.tight_layout()
        out = out_dir / f"{mi:02d}_{name}.png"
        fig.savefig(out, dpi=120, bbox_inches="tight")
        plt.close(fig)
        written.append(out)
        print(f"wrote {out}")

    return written


# ---------------------------------------------------------------------------
# Parent orchestration.
# ---------------------------------------------------------------------------


def _spawn_worker(config: str, args, out_json: Path) -> subprocess.Popen:
    cmd = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--_worker",
        config,
        "--out-json",
        str(out_json),
        "--project",
        args.project,
        "--start",
        args.start,
        "--timeout",
        str(args.timeout),
    ]
    if args.scene:
        cmd += ["--scene", args.scene]
    return subprocess.Popen(cmd)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--config",
        default="absolute_heading",
        choices=OV.CONFIGS[1:],
        help="optimize() config to compare against the plain-seq baseline",
    )
    ap.add_argument("--project", default=H.DEFAULT_PROJECT)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--start", default="42.8,25.63,90", help="'x_cm,y_cm,heading_deg'")
    ap.add_argument("--timeout", type=float, default=45.0)
    ap.add_argument("--out", default=str(REPO_ROOT / "opt_viz"))
    ap.add_argument("--missions", default=None, help="comma-separated name filter (e.g. M007,M010)")
    # internal worker mode
    ap.add_argument("--_worker", dest="worker", default=None)
    ap.add_argument("--out-json", default=None)
    args = ap.parse_args()

    if args.worker:
        _run_worker(
            args.worker,
            Path(args.project),
            args.scene,
            args.start,
            args.timeout,
            Path(args.out_json),
        )
        return  # unreachable (os._exit)

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    base_json = out_dir / "_baseline.json"
    opt_json = out_dir / f"_{args.config}.json"

    print(f"Running baseline + optimize({args.config}) chains in parallel subprocesses...\n")
    procs = [
        _spawn_worker("baseline", args, base_json),
        _spawn_worker(args.config, args, opt_json),
    ]
    rcs = [p.wait() for p in procs]
    if any(rc != 0 for rc in rcs):
        msg = f"worker exit codes {rcs} — check output above"
        raise SystemExit(msg)

    base = json.loads(base_json.read_text())
    opt = json.loads(opt_json.read_text())

    project = Path(args.project).resolve()
    scene0 = Path(args.scene) if args.scene else (project / H.DEFAULT_SCENE)
    sys.path.insert(0, str(project))
    cwd = Path.cwd()
    os.chdir(project)
    try:
        segs, table = H._load_segments(scene0)
    finally:
        os.chdir(cwd)

    mission_filter = {m.strip() for m in args.missions.split(",")} if args.missions else None
    written = _render(base, opt, args.config, segs, table, out_dir, mission_filter)
    print(f"\nDone → {out_dir}  ({len(written)} PNG(s))")
    os._exit(0)


if __name__ == "__main__":
    main()
