#!/usr/bin/env python3
"""Visualise a recorded ``localization.jsonl`` run on top of the game-table ftmap.

The localization recorder (see ``modules/libstp-localization``) writes one header
line + one ``frame`` line per record tick. Each frame carries the particle-filter
mean ``pose`` [x, y, theta], its ``sigma`` [sx, sy, stheta] and the full particle
cloud ``[[x, y, theta, weight], ...]``. Positions are in **metres**, heading in
**rad**. The C++ filter matches particles against the table map in **cm** using
exactly ``x_cm = x_m * 100`` (see ``localization.cpp`` surface-likelihood), so the
same ``*100`` is the natural registration onto the ftmap. The map's Y is stored
top-down in the ftmap but the filter (WorldMap) lives bottom-up (``y = h - y``),
so we flip the ftmap the same way and everything lands in one coherent frame.

Outputs (pure matplotlib — reuses the sim_run_chain table style):
  * a static overview PNG: full mean trajectory coloured by time + particle
    snapshots at a few instants + start/end markers.
  * an animation (mp4 via ffmpeg, or gif) of the particle cloud, robot footprint,
    heading and the 2-sigma uncertainty ellipse playing over the run.

Usage
-----
    .venv-test/bin/python tools/visualize_localization.py \
        --loc  .../run1_.../localization.jsonl \
        --map  .../config/2026-game-table-v2.ftmap \
        --out-dir /tmp/locviz --seconds 45

Only ``--loc`` is required; ``--map`` defaults to the cube-bot v2 ftmap if found.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

# ---------------------------------------------------------------------------
# Small geometry helpers (copied verbatim from tools/sim_run_chain.py so this
# tool stays standalone and never pulls in the sim engine on import).
# ---------------------------------------------------------------------------


def _b2w(x, y, theta, fwd, strafe):
    """Body-frame (forward, +strafe=left) → world."""
    c, s = math.cos(theta), math.sin(theta)
    return (x + fwd * c - strafe * s, y + fwd * s + strafe * c)


def _oriented_rect(cx, cy, theta, length, width, fwd_off=0.0, strafe_off=0.0):
    """Corners of a length×width rectangle centred at a body-frame offset."""
    hl, hw = length / 2.0, width / 2.0
    corners = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
    return [_b2w(cx, cy, theta, fwd_off + f, strafe_off + s) for f, s in corners]


# ---------------------------------------------------------------------------
# ftmap parsing (v1 lines[] or v2 layers[].lines[]) → flipped cm segments.
# ---------------------------------------------------------------------------


def load_ftmap(path: Path):
    """Return (segments, (table_w_cm, table_h_cm)). Segments are dicts with
    x0,y0,x1,y1 (cm, bottom-up), w (cm), wall (bool), layer (int)."""
    data = json.loads(path.read_text())
    table = data.get("table", {})
    tw = float(table.get("widthCm", 236.4232))
    th = float(table.get("heightCm", 105.918))

    def flip(node, layer):
        return {
            "x0": float(node["startX"]),
            "y0": th - float(node["startY"]),
            "x1": float(node["endX"]),
            "y1": th - float(node["endY"]),
            "w": float(node.get("widthCm", 1.0)),
            "wall": node.get("kind") == "wall",
            "layer": layer,
        }

    segs = []
    if "layers" in data:
        for li, layer in enumerate(data["layers"]):
            for node in layer.get("lines", []):
                segs.append(flip(node, li))
    else:  # v1 flat
        for node in data.get("lines", []):
            segs.append(flip(node, 0))
    return segs, (tw, th)


def draw_table(ax, segs, table):
    """Draw the ftmap in the sim_run_chain style: tape as black rectangles,
    ramp/deck tape teal, walls brown, table border box."""
    from matplotlib.patches import Polygon, Rectangle

    tw, th = table
    ax.add_patch(Rectangle((0, 0), tw, th, fill=False, ec="#333", lw=1.2, zorder=1))
    for s in segs:
        dx, dy = s["x1"] - s["x0"], s["y1"] - s["y0"]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        ang = math.atan2(dy, dx)
        cx, cy = (s["x0"] + s["x1"]) / 2.0, (s["y0"] + s["y1"]) / 2.0
        corners = _oriented_rect(cx, cy, ang, length, max(s["w"], 0.5))
        if s["wall"]:
            ax.add_patch(
                Polygon(
                    corners,
                    closed=True,
                    facecolor="#b5651d",
                    edgecolor="#7a4512",
                    lw=0.5,
                    alpha=0.85,
                    zorder=2,
                )
            )
        else:
            deck = s.get("layer", 0) >= 1
            ax.add_patch(
                Polygon(
                    corners,
                    closed=True,
                    facecolor="#0e7c7b" if deck else "#111",
                    edgecolor="#0e7c7b" if deck else "none",
                    lw=0.8 if deck else 0.0,
                    alpha=0.9,
                    zorder=3 if deck else 2,
                )
            )
    ax.set_aspect("equal")
    ax.set_xlabel("table x (cm)")
    ax.set_ylabel("table y (cm)")
    ax.grid(True, ls=":", alpha=0.3)


# ---------------------------------------------------------------------------
# localization.jsonl parsing.
# ---------------------------------------------------------------------------


def load_localization(
    path: Path, keep_particles_stride: int, t_start: float = 0.0, t_end: float | None = None
):
    """Stream the jsonl. Returns (header, frames) where frames is a list of
    dicts with t (s), x, y, th (cm/rad) and (only every ``stride``-th frame)
    a ``particles`` numpy array (N,4) in cm/rad/weight.

    Frames with t < ``t_start`` or > ``t_end`` (seconds) are dropped — used to
    cut the setup/calibration phase that runs in an un-anchored odom frame
    before ``resync_at_start_pose`` snaps the filter onto the map."""
    import numpy as np

    header = None
    frames = []
    anchor = None  # (x_cm, y_cm, th_rad) of the very FIRST frame (odometry zero)
    ocx = ocy = 0.0  # running odometry integral in the odom world frame (cm)
    fi = 0
    with path.open() as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except json.JSONDecodeError:
                continue
            if o.get("kind") == "header":
                header = o
                continue
            if o.get("kind") != "frame":
                continue
            p = o["pose"]
            xcm, ycm, thr = p[0] * 100.0, p[1] * 100.0, p[2]
            if anchor is None:
                anchor = (xcm, ycm, thr)
            od = o.get("odom_delta") or [0.0, 0.0, 0.0]
            ocx += od[0] * 100.0
            ocy += od[1] * 100.0
            t = o["t_ns"] / 1e9
            if t < t_start or (t_end is not None and t > t_end):
                continue
            rec = {
                "t": t,
                "x": xcm,
                "y": ycm,
                "th": thr,
                "ox": anchor[0] + ocx,  # odometry dead-reckoning path (odom frame, cm)
                "oy": anchor[1] + ocy,
                "sigma": list(o.get("sigma", [0, 0, 0])),
                "particles": None,
            }
            if fi % keep_particles_stride == 0:
                pts = o.get("particles") or []
                if pts:
                    arr = np.asarray(pts, dtype=float)
                    arr[:, 0] *= 100.0
                    arr[:, 1] *= 100.0
                    rec["particles"] = arr
            frames.append(rec)
            fi += 1
    if header is None:
        msg = f"No header found in {path}"
        raise SystemExit(msg)
    return header, frames, anchor


def apply_map_frame(frames, anchor, start_pose):
    """Rigidly move the recorded odom-frame trajectory into absolute MAP cm.

    On the real robot the filter is never anchored to the map (no
    ``resync_at_start_pose`` in the missions), so ``estimate_pose`` lives in the
    odometry boot frame. We know one correspondence: at the first frame the robot
    physically sits at ``start_pose`` (robot.yml, map cm). Since odometry is
    absolute (never reset mid-run), a SINGLE rigid transform T maps the whole run
    into the map frame: rotate by Δθ = start_theta − anchor_theta about the
    anchor, then translate the anchor onto start_pose. Applies to the estimate,
    the particle clouds and the odometry dead-reckoning path alike."""

    ax, ay, ath = anchor
    sx, sy, sth = start_pose  # cm, cm, rad
    dth = sth - ath
    c, s = math.cos(dth), math.sin(dth)

    def tf(x, y):
        dx, dy = x - ax, y - ay
        return sx + (c * dx - s * dy), sy + (s * dx + c * dy)

    for f in frames:
        f["x"], f["y"] = tf(f["x"], f["y"])
        f["ox"], f["oy"] = tf(f["ox"], f["oy"])
        f["th"] = f["th"] + dth
        if f["particles"] is not None:
            pc = f["particles"]
            nx, ny = tf(pc[:, 0], pc[:, 1])
            pc[:, 0], pc[:, 1] = nx, ny
            pc[:, 2] = pc[:, 2] + dth
    return math.degrees(dth)


def robot_geom(header):
    r = header.get("robot", {})
    return {
        "length_cm": float(r.get("length_cm", 26.0)),
        "width_cm": float(r.get("width_cm", 23.5)),
        "sensors": header.get("sensors", []),
    }


# ---------------------------------------------------------------------------
# Static overview.
# ---------------------------------------------------------------------------


def render_overview(header, frames, segs, table, out: Path, map_frame=False):
    import matplotlib as mpl

    mpl.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.collections import LineCollection

    fig, ax = plt.subplots(figsize=(14, 8))
    draw_table(ax, segs, table)

    xs = np.array([f["x"] for f in frames])
    ys = np.array([f["y"] for f in frames])
    ts = np.array([f["t"] for f in frames])
    oxs = np.array([f["ox"] for f in frames])
    oys = np.array([f["oy"] for f in frames])

    # Odometry dead-reckoning path (start_pose ⊕ integrated odometry) — the
    # robot's REAL motion, independent of the filter's fusion. Drawn first,
    # underneath, as a dashed reference.
    if map_frame:
        ax.plot(
            oxs,
            oys,
            "--",
            color="#8856a7",
            lw=1.6,
            alpha=0.85,
            zorder=4,
            label="odometry dead-reckoning",
        )

    # Filter estimate trajectory coloured by time.
    pts = np.column_stack([xs, ys]).reshape(-1, 1, 2)
    seglines = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(seglines, cmap="viridis", zorder=6, lw=2.0, alpha=0.9)
    lc.set_array(ts[:-1])
    ax.add_collection(lc)
    cbar = fig.colorbar(lc, ax=ax, fraction=0.025, pad=0.01)
    cbar.set_label("filter estimate — time (s)")

    # Particle snapshots at a handful of instants.
    snaps = [f for f in frames if f["particles"] is not None]
    if snaps:
        idxs = np.linspace(0, len(snaps) - 1, min(6, len(snaps))).astype(int)
        for k in idxs:
            arr = snaps[k]["particles"]
            ax.scatter(
                arr[:, 0], arr[:, 1], s=3, c="#e45756", alpha=0.25, zorder=5, edgecolors="none"
            )

    ax.plot(xs[0], ys[0], "o", ms=12, mfc="#54a24b", mec="white", zorder=8, label="start")
    ax.plot(xs[-1], ys[-1], "s", ms=12, mfc="#f58518", mec="white", zorder=8, label="filter end")
    if map_frame:
        ax.plot(
            oxs[-1], oys[-1], "D", ms=10, mfc="#8856a7", mec="white", zorder=8, label="odometry end"
        )

    # Fit view to map ∪ trajectory.
    _fit(ax, table, np.concatenate([xs, oxs]), np.concatenate([ys, oys]))
    ax.legend(loc="upper right", fontsize=9)
    dur = ts[-1] - ts[0]
    frame_note = "map frame via robot.yml start_pose" if map_frame else "pose×100 → table cm"
    ax.set_title(
        f"Localization run — {len(frames)} frames, {dur:.0f}s, "
        f"{header.get('particle_count','?')} particles  ({frame_note})",
        fontsize=12,
        fontweight="bold",
    )
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"[overview] wrote {out}")


def _fit(ax, table, xs, ys):
    tw, th = table
    xmin = min(0.0, float(xs.min())) - 12
    xmax = max(tw, float(xs.max())) + 12
    ymin = min(0.0, float(ys.min())) - 12
    ymax = max(th, float(ys.max())) + 12
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)


# ---------------------------------------------------------------------------
# Animation.
# ---------------------------------------------------------------------------


def render_animation(
    header, frames, segs, table, out: Path, fps: int, stride: int, map_frame=False, ell_angle=0.0
):
    import matplotlib as mpl

    mpl.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.animation import FFMpegWriter, PillowWriter
    from matplotlib.patches import Ellipse, Polygon

    geom = robot_geom(header)
    sel = frames[::stride]
    xs = np.array([f["x"] for f in frames])
    ys = np.array([f["y"] for f in frames])
    oxs = np.array([f["ox"] for f in frames])
    oys = np.array([f["oy"] for f in frames])

    fig, ax = plt.subplots(figsize=(14, 8))
    draw_table(ax, segs, table)
    _fit(ax, table, np.concatenate([xs, oxs]), np.concatenate([ys, oys]))

    # Odometry dead-reckoning reference path (full, static, dashed) + a moving
    # marker for where pure odometry says the robot is right now.
    odom_marker = None
    if map_frame:
        ax.plot(oxs, oys, "--", color="#8856a7", lw=1.3, alpha=0.55, zorder=4)
        (odom_marker,) = ax.plot([], [], "D", ms=9, mfc="#8856a7", mec="white", zorder=8)

    # Persistent animated artists.
    particle_sc = ax.scatter([], [], s=6, c="#e45756", alpha=0.45, edgecolors="none", zorder=5)
    (trail,) = ax.plot([], [], "-", color="#4c78a8", lw=1.4, alpha=0.7, zorder=6)
    body = Polygon(
        [(0, 0)],
        closed=True,
        facecolor="#4c78a8",
        edgecolor="#22364a",
        lw=1.2,
        alpha=0.35,
        zorder=7,
    )
    ax.add_patch(body)
    (nose,) = ax.plot([], [], "-", color="#22364a", lw=1.6, zorder=8)
    ell = Ellipse(
        (0, 0), 1, 1, angle=ell_angle, fill=False, ec="#f58518", lw=1.6, alpha=0.9, zorder=7
    )
    ax.add_patch(ell)
    sensor_sc = ax.scatter(
        [], [], s=28, c="#333", marker="o", zorder=9, edgecolors="white", linewidths=0.6
    )
    txt = ax.text(
        0.01,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=11,
        family="monospace",
        bbox={"boxstyle": "round", "fc": "white", "ec": "#999", "alpha": 0.85},
        zorder=10,
    )

    _tw, _th = table
    title = (
        "Localization replay — MAP FRAME (blue=filter estimate, " "purple=odometry)"
        if map_frame
        else "Localization particle filter — replay"
    )
    ax.set_title(title, fontsize=12, fontweight="bold")

    # Map selected-frame index → index into full frames array for the trail.
    full_idx = list(range(0, len(frames), stride))

    def _artists():
        a = [particle_sc, trail, body, nose, ell, sensor_sc, txt]
        if odom_marker is not None:
            a.append(odom_marker)
        return a

    def init():
        return _artists()

    def update(i):
        f = sel[i]
        fx, fy, fth = f["x"], f["y"], f["th"]

        # Particle cloud (fall back to last known if this frame skipped it).
        arr = f["particles"]
        if arr is not None:
            particle_sc.set_offsets(arr[:, :2])
            w = arr[:, 3]
            wn = (w - w.min()) / (float(w.max() - w.min()) + 1e-12)
            particle_sc.set_sizes(4 + 26 * wn)

        # Trail up to now.
        j = full_idx[i]
        trail.set_data(xs[: j + 1], ys[: j + 1])

        # Robot footprint + nose.
        corners = _oriented_rect(fx, fy, fth, geom["length_cm"], geom["width_cm"])
        body.set_xy(corners)
        nx, ny = _b2w(fx, fy, fth, geom["length_cm"] / 2.0 + 4.0, 0.0)
        nose.set_data([fx, nx], [fy, ny])

        # Sensors at their body offsets.
        spts = [
            _b2w(fx, fy, fth, s.get("forward_cm", 0.0), s.get("strafe_cm", 0.0))
            for s in geom["sensors"]
        ]
        if spts:
            sensor_sc.set_offsets(spts)

        # 2-sigma uncertainty ellipse (position sigma is in metres → ×100 cm).
        sx = f["sigma"][0] * 100.0
        sy = f["sigma"][1] * 100.0
        ell.set_center((fx, fy))
        ell.set_width(max(4 * sx, 0.5))
        ell.set_height(max(4 * sy, 0.5))

        if odom_marker is not None:
            odom_marker.set_data([f["ox"]], [f["oy"]])

        sth_deg = math.degrees(f["sigma"][2])
        txt.set_text(
            f"t = {f['t']:6.1f} s   frame {j}/{len(frames)}\n"
            f"filter = ({fx:6.1f}, {fy:6.1f}) cm  hdg {math.degrees(fth):6.1f}°\n"
            f"odom   = ({f['ox']:6.1f}, {f['oy']:6.1f}) cm\n"
            f"sigma  = ({sx:4.1f}, {sy:4.1f}) cm  {sth_deg:4.1f}°"
        )
        return _artists()

    from matplotlib.animation import FuncAnimation

    anim = FuncAnimation(
        fig, update, init_func=init, frames=len(sel), interval=1000 / fps, blit=True
    )

    if out.suffix.lower() == ".gif":
        writer = PillowWriter(fps=fps)
    else:
        writer = FFMpegWriter(fps=fps, bitrate=4000, metadata={"title": "localization replay"})
    print(f"[anim] rendering {len(sel)} frames → {out} ({fps} fps) ...")
    anim.save(str(out), writer=writer, dpi=110)
    plt.close(fig)
    print(f"[anim] wrote {out}")


# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--loc", required=True, type=Path, help="localization.jsonl")
    ap.add_argument("--map", type=Path, default=None, help="ftmap to draw")
    ap.add_argument("--out-dir", type=Path, default=Path("locviz"))
    ap.add_argument(
        "--seconds",
        type=float,
        default=45.0,
        help="target animation wall-clock length (controls stride)",
    )
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument(
        "--t-start",
        type=float,
        default=0.0,
        help="drop frames before this time (s); cuts the un-anchored "
        "setup phase before resync_at_start_pose",
    )
    ap.add_argument("--t-end", type=float, default=None, help="drop frames after this time (s)")
    ap.add_argument(
        "--robot-yml",
        type=Path,
        default=None,
        help="robot.yml providing physical.start_pose for the map-frame anchor",
    )
    ap.add_argument(
        "--start-pose",
        type=float,
        nargs=3,
        default=None,
        metavar=("X_CM", "Y_CM", "THETA_DEG"),
        help="override the map-frame anchor pose (else read from robot.yml)",
    )
    ap.add_argument(
        "--no-map-frame",
        action="store_true",
        help="plot raw pose×100 instead of anchoring to the map via start_pose",
    )
    ap.add_argument("--no-anim", action="store_true", help="overview PNG only")
    args = ap.parse_args()

    map_path = args.map
    if map_path is None:
        for cand in [
            Path("config/2026-game-table-v2-glb.ftmap"),
            args.loc.resolve().parents[3] / "config" / "2026-game-table-v2-glb.ftmap",
        ]:
            if cand.exists():
                map_path = cand
                break
    if map_path is None or not map_path.exists():
        msg = "Could not locate an ftmap; pass --map explicitly."
        raise SystemExit(msg)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[load] map  = {map_path}")
    print(f"[load] loc  = {args.loc}")

    segs, table = load_ftmap(map_path)

    # Choose stride so that the animation is ~seconds long at fps, but always
    # keep particle clouds for the frames we actually draw.
    # Cheap first pass: count frames that survive the trim, so the stride sizes
    # the animation to the KEPT frames (not the discarded setup phase).
    total = 0
    with args.loc.open() as fh:
        for line in fh:
            i = line.find('"t_ns":')
            if i < 0:
                continue
            j = line.find(",", i + 7)
            try:
                t = int(line[i + 7 : j]) / 1e9
            except ValueError:
                continue
            if t < args.t_start or (args.t_end is not None and t > args.t_end):
                continue
            total += 1
    total = max(total, 1)
    target_frames = max(1, int(args.seconds * args.fps))
    stride = max(1, total // target_frames)
    trim = f" (t {args.t_start}..{args.t_end if args.t_end is not None else 'end'}s)"
    print(f"[plan] {total} kept frames{trim} → stride {stride} → ~{total // stride} anim frames")

    header, frames, anchor = load_localization(
        args.loc, keep_particles_stride=stride, t_start=args.t_start, t_end=args.t_end
    )
    print(f"[load] {len(frames)} frames, {header.get('particle_count','?')} particles")

    # Resolve the map-frame anchor pose (robot.yml start_pose, unless overridden).
    map_frame = not args.no_map_frame
    ell_angle = 0.0
    if map_frame:
        start_pose = resolve_start_pose(args)
        if start_pose is None:
            print(
                "[warn] no start_pose (robot.yml/--start-pose) found → plotting raw "
                "odom frame. Pass --robot-yml or --no-map-frame."
            )
            map_frame = False
        else:
            ell_angle = apply_map_frame(frames, anchor, start_pose)
            print(
                f"[map ] anchored first frame {tuple(round(a,1) for a in anchor)} → "
                f"start_pose ({start_pose[0]:.1f},{start_pose[1]:.1f},"
                f"{math.degrees(start_pose[2]):.1f}°); rotation Δθ={ell_angle:.1f}°"
            )

    render_overview(header, frames, segs, table, args.out_dir / "overview.png", map_frame=map_frame)
    if not args.no_anim:
        render_animation(
            header,
            frames,
            segs,
            table,
            args.out_dir / "replay.mp4",
            fps=args.fps,
            stride=stride,
            map_frame=map_frame,
            ell_angle=ell_angle,
        )


def resolve_start_pose(args):
    """Return (x_cm, y_cm, theta_rad) from --start-pose or robot.yml, or None."""
    if args.start_pose is not None:
        x, y, deg = args.start_pose
        return (x, y, math.radians(deg))
    yml = args.robot_yml
    if yml is None:
        for cand in [
            Path("config/robot.yml"),
            args.loc.resolve().parents[3] / "config" / "robot.yml",
        ]:
            if cand.exists():
                yml = cand
                break
    if yml is None or not yml.exists():
        return None
    try:
        import yaml

        data = yaml.safe_load(yml.read_text())
        sp = data["physical"]["start_pose"]
        return (float(sp["x_cm"]), float(sp["y_cm"]), math.radians(float(sp["theta_deg"])))
    except Exception as exc:  # missing key / no yaml module
        print(f"[warn] could not read start_pose from {yml}: {exc!r}")
        return None


if __name__ == "__main__":
    main()
