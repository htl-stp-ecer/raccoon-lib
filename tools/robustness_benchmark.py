#!/usr/bin/env python3
"""Benchmark how INVARIANT a cube-bot competition run is against game-table
variation and noise — the table is never built exactly to spec, the robot is
never placed exactly, and the drivetrain drifts. This tool quantifies *where*
that hurts.

It drives the same headless mock sim as ``tools/sim_run_chain.py`` (real
Drive/odometry/localization + scene-driven line sensors) but runs the mission
chain MANY times under sampled perturbations, then measures how far each
mission's outcome drifts from a clean nominal run. A mission whose end-pose
scatters a lot — or that starts driving off the table — is a weakness.

Four perturbation axes (each a real-world source of table/robot variance):

  start   — start-pose offset (dx, dy, dheading): the robot is never placed
            exactly on its mark.
  geom    — field geometry: every tape line / wall segment is independently
            jittered ("the table is not exactly as specified").
  odo     — odometry drift: per-axis multiplicative wheel-slip / mis-calibration
            (via sim.mock.set_odometry_drift; ground truth is untouched).
  motor   — drivetrain noise: BEMF encoder noise, Coulomb friction, motor lag.

Two analysis modes:

  Monte-Carlo (combined) — all axes sampled together → realistic overall
            robustness per mission.
  OFAT (one-factor-at-a-time) — one axis active at a time at increasing
            magnitude → attributes each mission's instability to a specific axis.

Two granularities:

  chain     — missions run sequentially, pose/heading carry over like a real run
              (failures cascade — realistic).
  isolated  — each mission re-run from its nominal start pose (no cascade —
              cleanest weakness attribution). NOTE: isolated runs only restore
              POSE, not shared state (IR-calibration set, table layer, arm pose),
              so missions on the ramp layer are flagged low-fidelity.

Metric: INVARIANCE / DISPERSION. The nominal run is the reference; for every
trial each mission's end pose is compared to nominal (Euclidean cm + heading
deg). We report mean / p50 / p95 / max dispersion, the hard-failure rate
(left the table / crashed / regressed-to-timeout), and a 0–100 robustness
score per mission.

Run it under the mock test venv (the worker re-execs the same interpreter):

    RACCOON_SIM_FASTTIME=1 .venv-test/bin/python tools/robustness_benchmark.py \
        --mc-trials 24 --ofat-trials 6 --jobs 4 --out robustness_out

Smoke-test the harness first (fast, 1 nominal + a couple perturbed trials):

    RACCOON_SIM_FASTTIME=1 .venv-test/bin/python tools/robustness_benchmark.py --quick
"""

from __future__ import annotations

import argparse
import asyncio
import copy
import json
import math
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = REPO_ROOT / "tools"
DEFAULT_PROJECT = "/media/tobias/TobiasSSD/projects/Botball/competition/Ecer2026/cube-bot"
DEFAULT_SCENE = "config/2026-game-table-v2.ftmap"
# cube-bot's authored start pose (config/robot.yml physical.start_pose).
DEFAULT_START = "41.753,25.500,89.681"

# ── scoring scales (cm / deg at which p95 dispersion costs ~1 e-fold) ──
SCORE_XY_SCALE = 10.0
SCORE_TH_SCALE = 25.0
# A baseline-completing mission whose chain p95 exceeds this is cascade-dominated
# (it runs downstream of a mission that times out in sim), so its OFAT axis
# attribution is upstream noise — excluded from the sensitivity table/heatmap.
CASCADE_CM = 50.0


# ───────────────────────── perturbation model ─────────────────────────
@dataclass
class Sigmas:
    """1-sigma magnitudes for each axis (the 'level 1' stress)."""

    start_xy_cm: float = 1.5  # std of start x/y placement offset
    start_heading_deg: float = 2.0  # std of start heading offset
    geom_shift_cm: float = 1.0  # std of per-segment tape/wall translation
    geom_rot_deg: float = 0.0  # std of per-segment rotation (off by default)
    odo_drift: float = 0.03  # std of per-axis multiplicative odo bias (~3%)
    bemf_noise: float = 0.3  # std of injected BEMF encoder noise
    coulomb_friction: float = 2.0  # std of Coulomb friction (rad/s^2)
    motor_tc_frac: float = 0.3  # fractional std on motor time constant


@dataclass
class Perturb:
    """A single CONCRETE perturbation (already sampled). JSON-serialisable;
    the parent samples it, the worker applies it."""

    start_dx: float = 0.0
    start_dy: float = 0.0
    start_dtheta_rad: float = 0.0
    scene: str = ""  # path to a (possibly perturbed) ftmap
    odo_vx: float = 1.0
    odo_vy: float = 1.0
    odo_yaw: float = 1.0
    bemf_noise: float = 0.0
    coulomb_friction: float = 0.0
    motor_tc_scale: float = 1.0


def _axis_perturb(
    axis: str, level: float, sig: Sigmas, rng, base_scene: str, scene_for_geom
) -> Perturb:
    """Sample a Perturb with ONLY ``axis`` active at ``level`` x sigma.

    ``axis='all'`` activates every axis (Monte-Carlo combined).
    ``scene_for_geom(shift, rot) -> path`` writes a perturbed ftmap and returns
    its path (called only when the geom axis is active)."""
    p = Perturb(scene=base_scene)
    on = axis == "all"

    if on or axis == "start":
        p.start_dx = float(rng.normal(0, sig.start_xy_cm * level))
        p.start_dy = float(rng.normal(0, sig.start_xy_cm * level))
        p.start_dtheta_rad = math.radians(float(rng.normal(0, sig.start_heading_deg * level)))
    if on or axis == "geom":
        p.scene = scene_for_geom(sig.geom_shift_cm * level, sig.geom_rot_deg * level)
    if on or axis == "odo":
        p.odo_vx = 1.0 + float(rng.normal(0, sig.odo_drift * level))
        p.odo_vy = 1.0 + float(rng.normal(0, sig.odo_drift * level))
        p.odo_yaw = 1.0 + float(rng.normal(0, sig.odo_drift * level))
    if on or axis == "motor":
        p.bemf_noise = abs(float(rng.normal(0, sig.bemf_noise * level)))
        p.coulomb_friction = abs(float(rng.normal(0, sig.coulomb_friction * level)))
        p.motor_tc_scale = max(0.1, 1.0 + float(rng.normal(0, sig.motor_tc_frac * level)))
    return p


def _perturb_ftmap_json(data: dict, rng, shift_cm: float, rot_deg: float) -> dict:
    """Independently jitter every tape/wall segment of a (v1 or v2) ftmap.

    Each segment is rigidly translated by N(0, shift) in x and y (preserving its
    length, modelling tape-placement tolerance) and optionally rotated about its
    midpoint by N(0, rot). Layer transitions (the physical ramp seams) are left
    untouched — the ramp structure is rigid; only the tape on it varies."""
    d = copy.deepcopy(data)

    def jitter(seg: dict) -> None:
        keys = ("startX", "startY", "endX", "endY") if "startX" in seg else ("x0", "y0", "x1", "y1")
        if keys[0] not in seg:
            return
        sx, sy, ex, ey = (seg[keys[0]], seg[keys[1]], seg[keys[2]], seg[keys[3]])
        if rot_deg > 0:
            a = math.radians(float(rng.normal(0, rot_deg)))
            mx, my = (sx + ex) / 2.0, (sy + ey) / 2.0
            ca, sa = math.cos(a), math.sin(a)

            def rot(x, y):
                x, y = x - mx, y - my
                return mx + x * ca - y * sa, my + x * sa + y * ca

            sx, sy = rot(sx, sy)
            ex, ey = rot(ex, ey)
        dx, dy = float(rng.normal(0, shift_cm)), float(rng.normal(0, shift_cm))
        seg[keys[0]], seg[keys[1]] = sx + dx, sy + dy
        seg[keys[2]], seg[keys[3]] = ex + dx, ey + dy

    for layer in d.get("layers", []):
        for ln in layer.get("lines", []):
            jitter(ln)
    for ln in d.get("lines", []):  # v1 top-level
        jitter(ln)
    return d


# ───────────────────────── geometry helpers ─────────────────────────
def _angdiff_deg(a_rad: float, b_rad: float) -> float:
    d = math.degrees(a_rad - b_rad)
    return (d + 180.0) % 360.0 - 180.0


# ════════════════════════════ WORKER ════════════════════════════
# Runs exactly ONE trial in its own process (fresh MockPlatform singleton),
# writes a result JSON, then os._exit to dodge pybind teardown races.
def _worker(spec_path: str) -> None:
    spec = json.loads(Path(spec_path).read_text())
    project = Path(spec["project"]).resolve()

    os.chdir(project)
    sys.path.insert(0, str(project))
    sys.path.insert(0, str(TOOLS_DIR))
    sys.path.insert(0, str(REPO_ROOT / "tests" / "python" / "sim"))
    os.environ.setdefault("LIBSTP_LOG_LEVEL", "error")
    os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
    os.environ["LIBSTP_TIMING_ENABLED"] = "0"
    os.environ["RACCOON_SIM"] = "1"
    # No match clock in the headless sim — skip checkpoint waits (competition
    # timing, not motion) so wait_for_checkpoint doesn't block forever.
    os.environ["LIBSTP_NO_CHECKPOINTS"] = "1"

    import sim_fasttime  # type: ignore[import-not-found]
    import sim_run_chain as scr  # type: ignore[import-not-found]

    fast = sim_fasttime.enable()

    result = asyncio.run(_run_trial(scr, spec, fast))
    Path(spec["result_path"]).write_text(json.dumps(result))
    sys.stdout.flush()
    os._exit(0)


async def _run_trial(scr, spec: dict, fast: bool) -> dict:  # noqa: ARG001
    """Apply the spec's perturbation and run a chain or one isolated mission."""
    from _robot_builder import build_robot  # type: ignore[import-not-found]

    from raccoon import sim as _sim
    from raccoon.testing.sim import pose, use_scene

    pert = spec["perturb"]
    bounds = spec["bounds"]  # [xmin, xmax, ymin, ymax]
    include_traj = spec.get("include_traj", False)

    scr._install_fake_transport()
    cfg = scr._make_sim_config()
    # apply motor-axis perturbation onto the sim robot config
    cfg.bemf_noise_stddev = float(pert["bemf_noise"])
    cfg.coulomb_friction_rad_s2 = float(pert["coulomb_friction"])
    cfg.motor_time_constant_sec = float(cfg.motor_time_constant_sec) * float(pert["motor_tc_scale"])

    robot = scr._augment_services(build_robot(cfg, enable_localization=True))
    try:
        from src.hardware.defs import defs as _defs  # type: ignore[import-not-found]
        from src.hardware.robot import Robot  # type: ignore[import-not-found]

        robot.motion_pid_config = Robot.motion_pid_config
        robot.defs = _defs
    except Exception:
        pass
    scr._calibrate_ir_sensors()
    scr._patch_calibration_switch_for_sim()  # keep sim-scale IR thresholds on ramp/upper deck

    discovered = dict(scr._ordered_missions(Path(spec["project"])))
    # Run the canonical competition order from config/missions.yml — drops
    # discovered-but-unlisted extras (test/spare missions) so the benchmark
    # mirrors a real run.
    order = [nm for nm in spec["mission_order"] if nm in discovered]
    if spec["mode"] == "isolated":
        target = spec["mission"]
        missions = [(target, discovered[target])] if target in discovered else []
        start = tuple(spec["iso_start"])
    else:
        missions = [(nm, discovered[nm]) for nm in order]
        start = tuple(spec["start"])

    # apply the sampled start-pose offset
    start = (
        start[0] + pert["start_dx"],
        start[1] + pert["start_dy"],
        start[2] + pert["start_dtheta_rad"],
    )

    sim_scene = scr._sim_scene_path(Path(pert["scene"]))
    timeout = float(spec["timeout"])

    traj: list[dict] = []
    seg_out: list[dict] = []
    left_table = False
    running = {"on": True, "mi": -1}

    def _bounds_check(x, y):
        nonlocal left_table
        if not (bounds[0] <= x <= bounds[1] and bounds[2] <= y <= bounds[3]):
            left_table = True

    async def _poll():
        while running["on"]:
            p = pose()
            _bounds_check(p.x, p.y)
            if include_traj:
                traj.append(
                    {
                        "x": round(p.x, 2),
                        "y": round(p.y, 2),
                        "t": round(p.theta, 4),
                        "mi": running["mi"],
                    }
                )
            await asyncio.sleep(0.04)

    with use_scene(str(sim_scene), robot=cfg, start=start):
        # odometry drift must be set AFTER the scene is configured
        _sim.mock.set_odometry_drift(pert["odo_vx"], pert["odo_vy"], pert["odo_yaw"])
        poller = asyncio.create_task(_poll())
        await asyncio.sleep(0.05)
        for mi, (nm, cls) in enumerate(missions):
            running["mi"] = mi
            p0 = pose()
            layer0 = _sim.mock.current_layer()
            try:
                seq = cls().sequence().resolve()
            except Exception as e:
                seg_out.append(
                    {
                        "name": nm,
                        "status": f"build-skip:{type(e).__name__}",
                        "start": [p0.x, p0.y, p0.theta],
                        "end": [p0.x, p0.y, p0.theta],
                        "layer": layer0,
                    }
                )
                continue
            try:
                await asyncio.wait_for(seq.run_step(robot), timeout=timeout)
                status = "completed"
            except TimeoutError:
                status = "timeout"
            except Exception as e:
                status = f"error:{type(e).__name__}"
            p1 = pose()
            seg_out.append(
                {
                    "name": nm,
                    "status": status,
                    "start": [p0.x, p0.y, p0.theta],
                    "end": [p1.x, p1.y, p1.theta],
                    "layer": layer0,
                }
            )
        running["on"] = False
        await asyncio.gather(poller, return_exceptions=True)

    out = {"segments": seg_out, "left_table": left_table, "start": list(start)}
    if include_traj:
        out["traj"] = traj
    return out


# ════════════════════════════ PARENT ════════════════════════════
def _dispatch(python: str, specs: list[dict], jobs: int, label: str) -> list[dict]:
    """Run a batch of trial specs as parallel worker subprocesses."""
    results: list[dict | None] = [None] * len(specs)
    spec_dir = Path(specs[0]["result_path"]).parent

    def _one(i: int, spec: dict):
        sp = spec_dir / f"spec_{spec['_id']}.json"
        sp.write_text(json.dumps(spec))
        proc = subprocess.run(
            [python, str(Path(__file__).resolve()), "--worker", str(sp)],
            capture_output=True,
            text=True,
            timeout=spec.get("hard_timeout", 600),
            check=False,
        )
        rp = Path(spec["result_path"])
        if rp.exists():
            return i, json.loads(rp.read_text())
        return i, {"_error": (proc.stderr or proc.stdout or "no result")[-800:]}

    done = 0
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {ex.submit(_one, i, s): i for i, s in enumerate(specs)}
        for fut in as_completed(futs):
            i, res = fut.result()
            results[i] = res
            done += 1
            err = " ERR" if res.get("_error") else ""
            print(f"  [{label}] {done}/{len(specs)}{err}", flush=True)
    return [r for r in results if r is not None]


def _percentiles(vals: list[float]) -> dict:
    if not vals:
        return {"mean": 0.0, "p50": 0.0, "p95": 0.0, "max": 0.0, "n": 0}
    import numpy as np

    a = np.asarray(vals, dtype=float)
    return {
        "mean": float(a.mean()),
        "p50": float(np.percentile(a, 50)),
        "p95": float(np.percentile(a, 95)),
        "max": float(a.max()),
        "n": int(a.size),
    }


def _mission_score(p95_xy: float, p95_th: float, failrate: float) -> float:
    base = math.exp(-(p95_xy / SCORE_XY_SCALE) - (p95_th / SCORE_TH_SCALE))
    return max(0.0, min(100.0, 100.0 * (1.0 - failrate) * base))


def _dispersion_vs_nominal(trials: list[dict], nominal_by_name: dict) -> dict:
    """Aggregate per-mission dispersion of end pose vs the nominal reference."""
    per: dict[str, dict] = {}
    for res in trials:
        if res.get("_error"):
            continue
        left = res.get("left_table", False)
        for seg in res["segments"]:
            nm = seg["name"]
            nomp = nominal_by_name.get(nm)
            if nomp is None:
                continue
            rec = per.setdefault(
                nm,
                {
                    "xy": [],
                    "th": [],
                    "fail": 0,
                    "n": 0,
                    "status": {},
                    "nominal_status": nomp["status"],
                },
            )
            ex, ey, et = seg["end"]
            rec["xy"].append(math.hypot(ex - nomp["end"][0], ey - nomp["end"][1]))
            rec["th"].append(abs(_angdiff_deg(et, nomp["end"][2])))
            st = seg["status"]
            rec["status"][st] = rec["status"].get(st, 0) + 1
            # hard failure: off-table, crash, or a regression to timeout when
            # the nominal run completed this mission.
            regressed = (st in ("timeout",) or st.startswith("error")) and nomp[
                "status"
            ] == "completed"
            if left or st.startswith("error") or regressed:
                rec["fail"] += 1
            rec["n"] += 1

    out = {}
    for nm, rec in per.items():
        xy, th = _percentiles(rec["xy"]), _percentiles(rec["th"])
        failrate = rec["fail"] / max(1, rec["n"])
        # A mission that does not COMPLETE in the clean nominal run has no
        # trustworthy reference end pose (it timed out / failed to build at an
        # arbitrary spot), so its end-pose dispersion is sim-fidelity noise, not
        # a robustness signal. Flag it and withhold a score rather than mislead.
        baseline_ok = rec["nominal_status"] == "completed"
        out[nm] = {
            "xy_cm": xy,
            "th_deg": th,
            "fail_rate": failrate,
            "n": rec["n"],
            "status_hist": rec["status"],
            "nominal_status": rec["nominal_status"],
            "baseline_ok": baseline_ok,
            "score": _mission_score(xy["p95"], th["p95"], failrate) if baseline_ok else None,
        }
    return out


# ───────────────────────── reporting / plots ─────────────────────────
def _render(out_dir: Path, report: dict, nominal: dict, mc_trials: list[dict], segs, table) -> None:
    import matplotlib as mpl

    mpl.use("Agg")
    import matplotlib.pyplot as plt

    sim_run = __import__("sim_run_chain")
    order = [s["name"] for s in nominal["segments"]]

    # 1) spaghetti — every MC trajectory faint, nominal bold, per mission colour
    fig, ax = plt.subplots(figsize=(15, 7.5))
    sim_run._draw_table(ax, segs, table)
    for res in mc_trials:
        if res.get("_error") or "traj" not in res:
            continue
        for mi in range(len(order)):
            pts = [t for t in res["traj"] if t["mi"] == mi]
            if pts:
                ax.plot(
                    [p["x"] for p in pts],
                    [p["y"] for p in pts],
                    "-",
                    color=sim_run._COLORS[mi % len(sim_run._COLORS)],
                    lw=0.5,
                    alpha=0.18,
                    zorder=4,
                )
    for mi, seg in enumerate(nominal["segments"]):
        pts = [t for t in nominal.get("traj", []) if t["mi"] == mi]
        col = sim_run._COLORS[mi % len(sim_run._COLORS)]
        if pts:
            ax.plot(
                [p["x"] for p in pts],
                [p["y"] for p in pts],
                "-",
                color=col,
                lw=2.4,
                alpha=0.95,
                zorder=6,
                label=seg["name"],
            )
    ax.plot(nominal["start"][0], nominal["start"][1], "*", color="k", ms=14, zorder=8)
    ax.set_title(
        "cube-bot robustness — nominal (bold) vs Monte-Carlo spread (faint)",
        fontsize=13,
        fontweight="bold",
    )
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=7)
    fig.tight_layout()
    fig.savefig(out_dir / "spaghetti.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 2) robustness score bars (Monte-Carlo, chain)
    mc = report["monte_carlo"]["chain"]
    names = [n for n in order if n in mc and mc[n]["baseline_ok"]]
    scores = [mc[n]["score"] for n in names]
    if not names:
        names, scores = ["(no scored missions)"], [0]
    fig, ax = plt.subplots(figsize=(max(8, len(names) * 1.1), 5))
    cols = ["#2ca02c" if s >= 80 else "#ff7f0e" if s >= 50 else "#d62728" for s in scores]
    ax.bar(range(len(names)), scores, color=cols)
    for i, (_n, s) in enumerate(zip(names, scores, strict=False)):
        ax.text(i, s + 1, f"{s:.0f}", ha="center", fontsize=8)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=40, ha="right", fontsize=8)
    ax.set_ylim(0, 105)
    ax.set_ylabel("robustness score (0–100)")
    ax.set_title("Per-mission robustness (Monte-Carlo, chain) — lower = weaker", fontweight="bold")
    ax.grid(True, axis="y", ls=":", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out_dir / "robustness_scores.png", dpi=130)
    plt.close(fig)

    # 3) OFAT sensitivity heatmap (mission × axis → p95 end-pose drift, cm)
    ofat = report.get("ofat", {})
    axes = ["start", "geom", "odo", "motor"]
    base_ok = {
        n
        for n, m in report["monte_carlo"]["chain"].items()
        if m["baseline_ok"] and m["xy_cm"]["p95"] <= CASCADE_CM
    }
    onames = [n for n in order if n in base_ok and any(n in ofat.get(a, {}) for a in axes)]
    if onames:
        import numpy as np

        M = np.full((len(onames), len(axes)), np.nan)
        for r, n in enumerate(onames):
            for c, a in enumerate(axes):
                cell = ofat.get(a, {}).get(n)
                if cell:
                    M[r, c] = cell["xy_cm"]["p95"]
        fig, ax = plt.subplots(figsize=(max(6, len(axes) * 1.6), max(4, len(onames) * 0.5)))
        im = ax.imshow(M, aspect="auto", cmap="YlOrRd")
        ax.set_xticks(range(len(axes)))
        ax.set_xticklabels(axes)
        ax.set_yticks(range(len(onames)))
        ax.set_yticklabels(onames, fontsize=8)
        for r in range(len(onames)):
            for c in range(len(axes)):
                if not math.isnan(M[r, c]):
                    ax.text(c, r, f"{M[r, c]:.1f}", ha="center", va="center", fontsize=7)
        fig.colorbar(im, ax=ax, label="p95 end-pose drift (cm)")
        ax.set_title("OFAT sensitivity — which axis destabilises each mission", fontweight="bold")
        fig.tight_layout()
        fig.savefig(out_dir / "sensitivity_heatmap.png", dpi=130)
        plt.close(fig)


def _write_markdown(out_dir: Path, report: dict, nominal: dict) -> None:
    order = [s["name"] for s in nominal["segments"]]
    mc = report["monte_carlo"]["chain"]
    iso = report["monte_carlo"].get("isolated", {})
    ofat = report.get("ofat", {})
    axes = ["start", "geom", "odo", "motor"]
    L = [
        "# cube-bot robustness benchmark\n",
        f"- trials: {report['meta']['mc_trials']} Monte-Carlo (chain), "
        f"{report['meta']['ofat_trials']}/axis/level OFAT",
        f"- sigmas: {report['meta']['sigmas']}\n",
        "Metric = invariance: end-pose dispersion vs the clean nominal run. "
        "Only missions that COMPLETE nominally get a score; the rest have no "
        "trustworthy sim baseline (listed separately).\n",
        "## Per-mission robustness (Monte-Carlo, chain)\n",
        "| mission | score | end-drift p50/p95/max (cm) | head p95 (°) | fail% |",
        "|---|--:|--|--:|--:|",
    ]
    no_baseline = []
    for n in order:
        m = mc.get(n)
        if not m:
            continue
        if not m["baseline_ok"]:
            no_baseline.append((n, m["nominal_status"]))
            continue
        L.append(
            f"| {n} | **{m['score']:.0f}** | "
            f"{m['xy_cm']['p50']:.1f}/{m['xy_cm']['p95']:.1f}/{m['xy_cm']['max']:.1f} | "
            f"{m['th_deg']['p95']:.1f} | {100 * m['fail_rate']:.0f}% |"
        )
    if no_baseline:
        L += [
            "\n### No reliable sim baseline",
            "_Nominal run did not complete these (sim-fidelity limit, not a "
            "robustness verdict). Tune sim sensor fidelity or run them on the "
            "real robot._\n",
            "| mission | nominal status |",
            "|---|---|",
        ]
        L += [f"| {n} | {st} |" for n, st in no_baseline]
    if iso:
        L += [
            "\n## Isolated per-mission (no cascade)\n",
            "_Each mission re-run from its nominal start pose, so cascade from "
            "upstream timeouts is removed. Caveat: only POSE is restored — not "
            "arm/cube/grab or heading-reference state — so rows flagged "
            "`low-fidelity` are harness artifacts, not weaknesses._\n",
            "| mission | score | end-drift p95 (cm) | fail% | fidelity |",
            "|---|--:|--:|--:|--|",
        ]
        for n in order:
            m = iso.get(n)
            if not m:
                continue
            score = f"{m['score']:.0f}" if m["baseline_ok"] else "n/a"
            if not m["baseline_ok"]:
                fid = f"no-base({m['nominal_status']})"
            elif m.get("low_fidelity"):
                fid = f"low: {m.get('low_fidelity_reason', '?')}"
            else:
                fid = "ok"
            L.append(
                f"| {n} | {score} | {m['xy_cm']['p95']:.1f} | "
                f"{100 * m['fail_rate']:.0f}% | {fid} |"
            )
    base_ok = {n for n, m in mc.items() if m["baseline_ok"] and m["xy_cm"]["p95"] <= CASCADE_CM}
    if ofat:
        L += [
            "\n## OFAT sensitivity — p95 end-pose drift (cm) per axis\n",
            "_Which single axis destabilises each mission most. Restricted to "
            "missions with a clean nominal baseline; cascade-dominated downstream "
            "missions are omitted (their attribution is upstream noise)._\n",
            "| mission | " + " | ".join(axes) + " | dominant |",
            "|---|" + "--:|" * len(axes) + "--|",
        ]
        for n in order:
            if n not in base_ok:
                continue
            row, vals = [], {}
            for a in axes:
                cell = ofat.get(a, {}).get(n)
                v = cell["xy_cm"]["p95"] if cell else None
                vals[a] = v if v is not None else -1
                row.append(f"{v:.1f}" if v is not None else "–")
            dom = max(vals, key=vals.get) if any(v >= 0 for v in vals.values()) else "–"
            if any(ofat.get(a, {}).get(n) for a in axes):
                L.append(f"| {n} | " + " | ".join(row) + f" | **{dom}** |")
    scored = [(m["score"], n) for n, m in mc.items() if m["baseline_ok"]]
    weak = sorted(scored)[:3]
    L += ["\n## Weakest scored missions", ""]
    L += [f"- **{n}** (score {s:.0f})" for s, n in weak] or ["- (none scored)"]
    (out_dir / "REPORT.md").write_text("\n".join(L) + "\n")


# ───────────────────────── orchestration ─────────────────────────
def _canonical_order(project: Path) -> list[str]:
    """Mission run order from config/missions.yml (setup/shutdown excluded)."""
    txt = (project / "config" / "missions.yml").read_text()
    order = []
    for raw in txt.splitlines():
        line = raw.strip()
        if not line.startswith("- "):
            continue
        body = line[2:].strip()
        name, _, tag = body.partition(":")
        name, tag = name.strip(), tag.strip().lower()
        if tag in ("setup", "shutdown"):
            continue
        order.append(name)
    return order


def _spec(
    _id,
    mode,
    project,
    scene,
    start,
    timeout,
    perturb,
    bounds,
    result_path,
    mission_order,
    include_traj=False,
    mission=None,
    iso_start=None,
) -> dict:
    s = {
        "_id": _id,
        "mode": mode,
        "project": project,
        "scene": scene,
        "start": start,
        "timeout": timeout,
        "perturb": asdict(perturb),
        "bounds": bounds,
        "result_path": str(result_path),
        "mission_order": mission_order,
        "include_traj": include_traj,
        "hard_timeout": max(120, int(timeout * 8)),
    }
    if mission is not None:
        s["mission"] = mission
        s["iso_start"] = iso_start
    return s


def run_benchmark(args) -> None:
    import numpy as np

    out_dir = Path(args.out).resolve()
    (out_dir / "_work").mkdir(parents=True, exist_ok=True)
    work = out_dir / "_work"
    scene_dir = out_dir / "_scenes"
    scene_dir.mkdir(exist_ok=True)

    project = Path(args.project).resolve()
    scene0 = Path(args.scene) if args.scene else (project / DEFAULT_SCENE)
    sx, sy, sdeg = (float(v) for v in args.start.split(","))
    start = (sx, sy, math.radians(sdeg))
    python = sys.executable

    sig = Sigmas(
        start_xy_cm=args.sigma_start_xy,
        start_heading_deg=args.sigma_start_heading,
        geom_shift_cm=args.sigma_geom_shift,
        geom_rot_deg=args.sigma_geom_rot,
        odo_drift=args.sigma_odo,
        bemf_noise=args.sigma_bemf,
        coulomb_friction=args.sigma_coulomb,
        motor_tc_frac=args.sigma_motor_tc,
    )

    # table bounds (with margin) for off-table detection + render context
    sys.path.insert(0, str(TOOLS_DIR))
    sys.path.insert(0, str(project))
    cwd = os.getcwd()  # noqa: PTH109
    os.chdir(project)
    import sim_run_chain as scr  # type: ignore[import-not-found]

    segs, table = scr._load_segments(scene0)
    os.chdir(cwd)
    mission_order = _canonical_order(project)
    bounds = [-8.0, table[0] + 8.0, -8.0, table[1] + 8.0]
    base_data = json.loads(scene0.read_text())
    rng = np.random.default_rng(args.seed)

    def scene_for_geom(shift, rot):
        sd = _perturb_ftmap_json(base_data, rng, shift, rot)
        p = scene_dir / f"geom_{rng.integers(1 << 30)}.ftmap"
        p.write_text(json.dumps(sd))
        return str(p)

    mc_n = 2 if args.quick else args.mc_trials
    ofat_n = 1 if args.quick else args.ofat_trials
    iso_n = 0 if args.quick else args.isolated_trials

    # ── 0) nominal reference (chain, no perturbation, with trajectory) ──
    print("nominal reference run …", flush=True)
    nom_spec = _spec(
        "nominal",
        "chain",
        str(project),
        str(scene0),
        list(start),
        args.timeout,
        Perturb(scene=str(scene0)),
        bounds,
        work / "nominal.json",
        mission_order,
        include_traj=True,
    )
    nominal = _dispatch(python, [nom_spec], 1, "nominal")[0]
    if nominal.get("_error"):
        print("FATAL: nominal run failed:\n" + nominal["_error"], file=sys.stderr)
        sys.exit(1)
    nominal_by_name = {s["name"]: s for s in nominal["segments"]}
    order = [s["name"] for s in nominal["segments"]]
    print(
        f"  nominal ran {len(order)} missions: "
        + ", ".join(f"{s['name']}={s['status']}" for s in nominal["segments"]),
        flush=True,
    )

    report = {
        "meta": {
            "mc_trials": mc_n,
            "ofat_trials": ofat_n,
            "isolated_trials": iso_n,
            "sigmas": asdict(sig),
            "start": list(start),
            "seed": args.seed,
        },
        "nominal": nominal["segments"],
        "monte_carlo": {},
        "ofat": {},
    }

    # ── 1) Monte-Carlo combined (chain, all axes) ──
    print(f"Monte-Carlo (chain, all axes): {mc_n} trials", flush=True)
    mc_specs = []
    for i in range(mc_n):
        p = _axis_perturb("all", 1.0, sig, rng, str(scene0), scene_for_geom)
        mc_specs.append(
            _spec(
                f"mc{i}",
                "chain",
                str(project),
                str(scene0),
                list(start),
                args.timeout,
                p,
                bounds,
                work / f"mc{i}.json",
                mission_order,
                include_traj=True,
            )
        )
    mc_trials = _dispatch(python, mc_specs, args.jobs, "MC") if mc_specs else []
    report["monte_carlo"]["chain"] = _dispersion_vs_nominal(mc_trials, nominal_by_name)

    # ── 2) OFAT sensitivity (chain, one axis at a time, 2 levels) ──
    if ofat_n > 0:
        for axis in ("start", "geom", "odo", "motor"):
            specs = []
            for level in (1.0, 2.0):
                for j in range(ofat_n):
                    p = _axis_perturb(axis, level, sig, rng, str(scene0), scene_for_geom)
                    specs.append(
                        _spec(
                            f"ofat_{axis}_{level:g}_{j}",
                            "chain",
                            str(project),
                            str(scene0),
                            list(start),
                            args.timeout,
                            p,
                            bounds,
                            work / f"ofat_{axis}_{level:g}_{j}.json",
                            mission_order,
                        )
                    )
            print(f"OFAT axis={axis}: {len(specs)} trials", flush=True)
            res = _dispatch(python, specs, args.jobs, f"OFAT:{axis}")
            report["ofat"][axis] = _dispersion_vs_nominal(res, nominal_by_name)

    # ── 3) isolated per-mission diagnostic (no cascade) ──
    if iso_n > 0:
        targets = order if not args.missions else [m for m in order if m in set(args.missions)]
        iso_agg: dict[str, dict] = {}
        for nm in targets:
            nomp = nominal_by_name[nm]
            iso_start = nomp["start"]
            low_fid = nomp.get("layer", 0) != 0
            specs = []
            for i in range(iso_n):
                p = _axis_perturb("all", 1.0, sig, rng, str(scene0), scene_for_geom)
                specs.append(
                    _spec(
                        f"iso_{nm}_{i}",
                        "isolated",
                        str(project),
                        str(scene0),
                        list(start),
                        args.timeout,
                        p,
                        bounds,
                        work / f"iso_{nm}_{i}.json",
                        mission_order,
                        mission=nm,
                        iso_start=iso_start,
                    )
                )
            print(
                f"isolated {nm}: {len(specs)} trials{' [low-fidelity ramp]' if low_fid else ''}",
                flush=True,
            )
            res = _dispatch(python, specs, args.jobs, f"iso:{nm}")
            agg = _dispersion_vs_nominal(res, nominal_by_name).get(nm)
            if agg:
                # Isolated mode restores POSE only — not prior arm/cube/grab or
                # heading-reference state. A mission whose isolated fail-rate is
                # far above its chain fail-rate lost necessary upstream state, so
                # its isolated number is a harness artifact, not a verdict.
                chain_fail = report["monte_carlo"]["chain"].get(nm, {}).get("fail_rate", 0.0)
                lost_state = agg["fail_rate"] - chain_fail > 0.5
                agg["low_fidelity"] = bool(low_fid or lost_state)
                agg["low_fidelity_reason"] = (
                    "ramp-layer" if low_fid else "lost-prior-state" if lost_state else ""
                )
                iso_agg[nm] = agg
        report["monte_carlo"]["isolated"] = iso_agg

    # ── write outputs ──
    (out_dir / "report.json").write_text(json.dumps(report, indent=2))
    _write_markdown(out_dir, report, nominal)
    try:
        _render(out_dir, report, nominal, mc_trials, segs, table)
    except Exception as e:
        print(f"[warn] render failed: {type(e).__name__}: {e}", file=sys.stderr)

    # ── console summary ──
    print("\n" + "=" * 64)
    print("ROBUSTNESS SUMMARY (Monte-Carlo, chain)")
    print("=" * 64)
    mc = report["monte_carlo"]["chain"]
    no_baseline = []
    for n in order:
        m = mc.get(n)
        if not m:
            continue
        if not m["baseline_ok"]:
            no_baseline.append((n, m["nominal_status"]))
            continue
        flag = "  <-- WEAK" if m["score"] < 50 else ""
        print(
            f"  {n:<34} score {m['score']:5.1f}  "
            f"p95 {m['xy_cm']['p95']:5.1f}cm  fail {100 * m['fail_rate']:3.0f}%{flag}"
        )
    if no_baseline:
        print(
            "\n  no reliable sim baseline (nominal run did not complete these —"
            " sim-fidelity, not a robustness verdict):"
        )
        for n, st in no_baseline:
            print(f"    {n:<34} nominal={st}")
    print(f"\nOutputs → {out_dir}")
    print("  report.json  REPORT.md  spaghetti.png  robustness_scores.png  sensitivity_heatmap.png")
    os._exit(0)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--worker", help=argparse.SUPPRESS)
    ap.add_argument("--project", default=DEFAULT_PROJECT)
    ap.add_argument("--scene", default=None, help="ftmap (default: project v2 game table)")
    ap.add_argument("--start", default=DEFAULT_START, help="'x_cm,y_cm,heading_deg'")
    ap.add_argument("--timeout", type=float, default=20.0, help="per-mission timeout (virtual s)")
    ap.add_argument("--out", default=str(REPO_ROOT / "robustness_out"))
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--jobs", type=int, default=4, help="parallel worker subprocesses")
    ap.add_argument("--mc-trials", type=int, default=24, dest="mc_trials")
    ap.add_argument(
        "--ofat-trials",
        type=int,
        default=6,
        dest="ofat_trials",
        help="trials per axis per level (0 disables OFAT)",
    )
    ap.add_argument(
        "--isolated-trials",
        type=int,
        default=8,
        dest="isolated_trials",
        help="trials per mission for isolated diagnostic (0 disables)",
    )
    ap.add_argument("--missions", nargs="*", default=None, help="subset for isolated mode")
    ap.add_argument("--quick", action="store_true", help="smoke test: tiny trial counts")
    # sigma overrides
    ap.add_argument("--sigma-start-xy", type=float, default=1.5)
    ap.add_argument("--sigma-start-heading", type=float, default=2.0)
    ap.add_argument("--sigma-geom-shift", type=float, default=1.0)
    ap.add_argument("--sigma-geom-rot", type=float, default=0.0)
    ap.add_argument("--sigma-odo", type=float, default=0.03)
    ap.add_argument("--sigma-bemf", type=float, default=0.3)
    ap.add_argument("--sigma-coulomb", type=float, default=2.0)
    ap.add_argument("--sigma-motor-tc", type=float, default=0.3)
    args = ap.parse_args()

    if args.worker:
        _worker(args.worker)
        return

    if not os.environ.get("RACCOON_SIM_FASTTIME"):
        print(
            "[warn] RACCOON_SIM_FASTTIME is not set — runs will use wall-clock time "
            "and be SLOW. Re-run with RACCOON_SIM_FASTTIME=1 for ~30x speedup.",
            file=sys.stderr,
        )
    run_benchmark(args)


if __name__ == "__main__":
    main()
