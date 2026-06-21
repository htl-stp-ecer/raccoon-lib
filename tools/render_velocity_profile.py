#!/usr/bin/env python3
"""Render the velocity profile a mission's compiled path WOULD execute — baseline
(brake to zero at every segment boundary) vs time_optimal + sensor-carry (flow
through same-direction sensor seams). DETERMINISTIC: built straight from the
stamped entry/exit speeds, no sim, never hangs.

For each motion segment we draw the trapezoid v(s) = min(accel-from-entry,
decel-to-exit, cruise-cap) over its length; condition-bounded ``.until()`` legs
have unknown length, so a nominal display length is used (clearly an
illustration of the SHAPE — where the robot stops vs flows, not exact timing).

    .venv-test/bin/python tools/render_velocity_profile.py --targets m050 m070 --out /tmp/vel.png
"""

from __future__ import annotations

import argparse
import importlib
import inspect
import math
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "python" / "sim"))
import sim_run_chain as H  # noqa: E402

os.environ.update(RACCOON_SIM="1", LIBSTP_LOG_LEVEL="error", LIBSTP_NO_CALIBRATE="1")

NOMINAL_UNTIL_M = 0.30  # display length for an unknown-endpoint .until() leg
SAMPLES_PER_SEG = 60


def _load(prefix):
    import src.missions as M  # type: ignore[import-not-found]
    from raccoon.mission.api import Mission

    mod = next(
        importlib.import_module(f"src.missions.{mi.name}")
        for mi in __import__("pkgutil").iter_modules(M.__path__)
        if mi.name.startswith(prefix)
    )
    return next(o for _, o in inspect.getmembers(mod, inspect.isclass)
                if issubclass(o, Mission) and o.__module__ == mod.__name__)


def _axis_limits():
    """Real per-axis (cruise, accel, decel) in m/s · m/s² from the project's
    tuned ``Robot.motion_pid_config`` — the SAME constraints the sim executes
    with (robot.yml ``motion_pid.linear``/``lateral``). No fabricated defaults."""
    from src.hardware.robot import Robot  # type: ignore[import-not-found]

    pc = Robot.motion_pid_config
    lin, lat = pc.linear, pc.lateral
    return {
        "linear": (lin.max_velocity, lin.acceleration, lin.deceleration),
        "lateral": (lat.max_velocity, lat.acceleration, lat.deceleration),
    }


def _seg_axis(seg):
    """'lateral' for a strafe leg, else 'linear'."""
    if getattr(seg, "lateral", False):
        return "lateral"
    if seg.axis is not None and "LAT" in str(seg.axis).upper():
        return "lateral"
    return "linear"


def _seg_length(seg):
    from raccoon.step.motion.path.passes.velocity_profile import _length_m

    return _length_m(seg) or NOMINAL_UNTIL_M


def _trapezoid(length, ve, vx, vc, accel, decel):
    """Sample v(s) over [0, length] with REAL asymmetric accel/decel (m/s²)."""
    xs, vs = [], []
    for k in range(SAMPLES_PER_SEG + 1):
        s = length * k / SAMPLES_PER_SEG
        v_acc = math.sqrt(max(0.0, ve * ve + 2 * accel * s))
        v_dec = math.sqrt(max(0.0, vx * vx + 2 * decel * (length - s)))
        xs.append(s)
        vs.append(min(v_acc, v_dec, vc))
    return xs, vs


def _profile(target, carry, limits):
    """Return (cum_dist_cm, speed_cmps, stop_marks_cm) for a compiled mission,
    drawn at the robot's REAL axis cruise/accel/decel.

    carry=False = baseline (rest-to-rest every leg). carry=True keeps the robot
    at the axis cruise across a seam the velocity-profile pass marked as a
    flow-through (stamped exit_speed > 0), instead of braking to zero."""
    from raccoon.step.motion import optimize
    from raccoon.step.motion.path.compiler import PathCompiler
    from raccoon.step.motion.path.ir import Segment

    raw = _load(target)().sequence()
    opt = optimize(raw).time_optimal()
    nodes = PathCompiler(opt._effective_passes()).compile(opt._raw_steps).nodes
    segs = [n for n in nodes if isinstance(n, Segment) and n.kind != "turn"]

    cum, X, V, stops = 0.0, [], [], []
    for i, s in enumerate(segs):
        L = _seg_length(s)
        cruise, accel, decel = limits[_seg_axis(s)]
        cruise *= s.speed_scale or 1.0
        # The pass stamps entry/exit in its own (high-cap) units; what is REAL is
        # the boolean — carry across this seam or not. At a carried seam the robot
        # holds the real axis cruise; otherwise it is at rest.
        carries_in = carry and (s.entry_speed_mps or 0.0) > 1e-6
        carries_out = carry and (s.exit_speed_mps or 0.0) > 1e-6
        ve = cruise if carries_in else 0.0
        vx = cruise if carries_out else 0.0
        xs, vs = _trapezoid(L, ve, vx, cruise, accel, decel)
        for x, v in zip(xs, vs):
            X.append((cum + x) * 100.0)
            V.append(v * 100.0)
        if i > 0 and not carries_in:
            stops.append(cum * 100.0)
        cum += L
    return X, V, stops


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--targets", nargs="+", default=["m050", "m070"])
    ap.add_argument("--out", default="/tmp/vel.png")
    args = ap.parse_args()

    proj = Path(H.DEFAULT_PROJECT)
    sys.path.insert(0, str(proj))
    os.chdir(proj)
    H._install_fake_transport()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    limits = _axis_limits()
    lin = limits["linear"]
    print(f"using REAL robot.yml axis limits — linear cruise={lin[0]*100:.1f} cm/s "
          f"accel={lin[1]:.3f} decel={lin[2]:.3f} m/s²")

    n = len(args.targets)
    fig, axes = plt.subplots(n, 1, figsize=(15, 3.6 * n), squeeze=False)
    for ax, tgt in zip(axes[:, 0], args.targets):
        Xb, Vb, stops_b = _profile(tgt, carry=False, limits=limits)
        Xc, Vc, stops_c = _profile(tgt, carry=True, limits=limits)
        eliminated = [x for x in stops_b if x not in stops_c]

        ax.plot(Xb, Vb, "-", color="#999", lw=1.5, label="baseline — brakes to 0 at every boundary")
        ax.plot(Xc, Vc, "-", color="#e8731a", lw=2.0, label="time_optimal + sensor-carry — flows through")
        ax.fill_between(Xc, Vc, color="#e8731a", alpha=0.10)
        for x in eliminated:
            ax.axvline(x, color="#2a9d3a", ls=":", lw=1.3, alpha=0.7)
        if eliminated:
            ax.plot([], [], color="#2a9d3a", ls=":", label=f"full-stops eliminated ({len(eliminated)})")
        ax.set_title(f"{tgt.upper()} — velocity profile along the path", fontsize=12, fontweight="bold")
        ax.set_ylabel("speed (cm/s)")
        ax.grid(True, ls=":", alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
        ax.set_ylim(bottom=-2)
    axes[-1, 0].set_xlabel(
        f"distance along path (cm)   ·   real robot.yml limits (linear cruise {lin[0]*100:.1f} cm/s, "
        f"accel {lin[1]:.2f}/decel {lin[2]:.2f} m/s²)   ·   .until() legs at nominal length")
    fig.suptitle("Sensor-boundary velocity carry: where the robot used to STOP, it now FLOWS THROUGH",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = Path(args.out)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"wrote {out}  ({n} missions)")


if __name__ == "__main__":
    main()
