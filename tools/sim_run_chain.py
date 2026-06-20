#!/usr/bin/env python3
"""Simulate a cube-bot competition run as ONE continuous chain and render it.

Places the robot at a single start pose on the game-table ftmap, then runs the
missions in order WITHOUT re-posing between them — pose, heading reference and
localization carry over exactly like a real run, so each mission starts where
the previous one stopped. The pose is polled throughout, and the trajectory is
drawn on the ftmap (lines + walls) so the run can be validated by eye.

Sensor-/condition-driven missions depend on the start pose matching reality;
when a `.until(...)` never fires the mission times out — the chain continues
from wherever it stopped and the timeout is marked, so the PNG still shows what
happened.

Run under the mock test venv:

    .venv-test/bin/python tools/sim_run_chain.py --start 42.8,25.63,90 --out sim_run
"""

from __future__ import annotations

import argparse
import asyncio
import importlib
import inspect
import json
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROJECT = "/media/tobias/TobiasSSD/projects/Botball/competition/Ecer2026/cube-bot"
DEFAULT_SCENE = "config/2026-game-table.ftmap"

# Colour cycle for per-mission trajectory segments.
_COLORS = [
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#17becf",
    "#bcbd22",
    "#7f7f7f",
]


def _make_sim_config():
    from src.hardware.defs import Defs  # type: ignore[import-not-found]
    from src.hardware.robot import Robot  # type: ignore[import-not-found]

    from raccoon.testing.sim import LineSensorMount, SimRobotConfig

    line_sensors = []
    for sensor, sp in Robot._sensor_positions.items():
        p = getattr(sensor, "port", None)
        if p is None:
            continue
        line_sensors.append(
            LineSensorMount(
                analog_port=int(p),
                forward_cm=float(sp.forward_cm),
                strafe_cm=float(sp.strafe_cm),
                name=f"port{p}",
            )
        )

    return SimRobotConfig(
        drivetrain="mecanum",
        fl_motor_port=Defs.front_left_motor.port,
        fr_motor_port=Defs.front_right_motor.port,
        bl_motor_port=Defs.rear_left_motor.port,
        br_motor_port=Defs.rear_right_motor.port,
        fl_motor_inverted=Defs.front_left_motor.inverted,
        fr_motor_inverted=Defs.front_right_motor.inverted,
        bl_motor_inverted=Defs.rear_left_motor.inverted,
        br_motor_inverted=Defs.rear_right_motor.inverted,
        wheelbase_m=0.125,
        track_width_m=0.20,
        wheel_radius_m=0.0375,
        max_wheel_velocity_rad_s=30.0,
        width_cm=float(getattr(Robot, "width_cm", 23.5)),
        length_cm=float(getattr(Robot, "length_cm", 26.0)),
        rotation_center_forward_cm=float(getattr(Robot, "rotation_center_forward_cm", 0.0)),
        line_sensors=line_sensors,
    )


def _robot_geometry():
    """Body/wheel/sensor geometry (cm, body frame) from the project Robot."""
    from src.hardware.robot import Robot  # type: ignore[import-not-found]

    wheels = [
        {"fwd": float(wp.forward_cm), "strafe": float(wp.strafe_cm)}
        for wp in getattr(Robot, "_wheel_positions", {}).values()
    ]
    sensors = [
        {
            "name": s.__class__.__name__ + f":{getattr(s, 'port', '?')}",
            "port": int(getattr(s, "port", -1)),
            "fwd": float(sp.forward_cm),
            "strafe": float(sp.strafe_cm),
        }
        for s, sp in getattr(Robot, "_sensor_positions", {}).items()
    ]
    return {
        "width_cm": float(getattr(Robot, "width_cm", 23.5)),
        "length_cm": float(getattr(Robot, "length_cm", 26.0)),
        "rc_fwd": float(getattr(Robot, "rotation_center_forward_cm", 0.0)),
        "rc_strafe": float(getattr(Robot, "rotation_center_strafe_cm", 0.0)),
        "wheels": wheels,
        "sensors": sensors,
    }


def _calibrate_ir_sensors(black: float = 700.0, white: float = 300.0):
    """Give the project's IR line sensors sim-scale calibration thresholds.

    The mock sim emits line sensors on a 0..1023 scale (1023 = black, 0 = white
    after the SimWorld polarity fix). cube-bot's real thresholds (~208/~3222)
    don't fit that scale, and an uncalibrated IRSensor's probabilityOfBlack
    throws — so inject thresholds that bracket the sim scale. setCalibration
    takes (black_threshold, white_threshold).
    """
    from src.hardware.robot import Robot  # type: ignore[import-not-found]

    n = 0
    for sensor in getattr(Robot, "_sensor_positions", {}):
        setcal = getattr(sensor, "setCalibration", None) or getattr(sensor, "set_calibration", None)
        if setcal is not None:
            try:
                setcal(black, white)
                n += 1
            except Exception as e:
                print(f"    [warn] calibrate port {getattr(sensor,'port','?')}: {e}")
    print(f"    calibrated {n} IR line sensor(s) to sim scale (black={black}, white={white})")


class _FakeTransport:
    """No-op transport so UI steps construct + run headless under the mock bundle.

    ``UIStep.__init__`` calls ``get_transport()`` (raises on the mock bundle),
    which makes any UI step — e.g. ``switch_calibration_set`` (a ``UIStep``) —
    fail at *construction*, breaking sequence() build for M050/M080. Stubbing
    the transport lets them build and run; screen publishes go nowhere, which
    is exactly right for a headless sim.
    """

    def publish_raw(self, *a, **k):
        pass

    def subscribe(self, *a, **k):
        return object()

    def unsubscribe(self, *a, **k):
        pass


def _sim_scene_path(real_scene: Path) -> Path:
    """Derive a sim scene from the real ftmap with interior WALL segments dropped.

    The game-table ftmap models physical structures (the drivable RAMP, rails)
    as ``wall`` segments. A 2D sim has no elevation, so those become phantom
    barriers that wrongly block the robot (e.g. the long y=66 ramp wall blocks
    the bottom of the table). The real robot doesn't collide with the ramp — it
    drives over it — so for the sim we keep the LINE segments (needed for line
    following) and let ``buildCollisionWalls`` add only the real table border.
    The real ftmap is left untouched (the real robot's localization uses it).
    """
    data = json.loads(real_scene.read_text())
    segs = data.get("lines", [])
    kept = [s for s in segs if s.get("kind") != "wall"]
    dropped = len(segs) - len(kept)
    data["lines"] = kept
    out = real_scene.with_name(real_scene.stem + "__sim_drivable.ftmap")
    out.write_text(json.dumps(data))
    print(f"    sim scene: dropped {dropped} interior wall(s) (ramp/structures are drivable)")
    return out


def _install_fake_transport():
    """Patch get_transport in the UI step module to return a no-op transport."""
    try:
        import raccoon.ui.step as _uistep

        ft = _FakeTransport()
        _uistep.get_transport = lambda: ft
    except Exception as e:
        print(f"    [warn] could not stub UI transport: {e}")


def _augment_services(robot):
    robot._services = {}

    def get_service(cls):
        svc = robot._services.get(cls)
        if svc is None:
            svc = cls(robot)
            robot._services[cls] = svc
        return svc

    robot.get_service = get_service
    # GenericRobot mixes in ClassNameLogger; some steps call robot.warn/info/…
    for level in ("trace", "debug", "info", "warn", "warning", "error"):
        if not hasattr(robot, level):
            setattr(robot, level, lambda *a, **k: None)
    return robot


def _ordered_missions(project: Path):
    """Mission classes in run order (Robot.missions order if available)."""
    import src.missions as M_pkg  # type: ignore[import-not-found]

    from raccoon.mission.api import Mission

    by_name = {}
    for mi in __import__("pkgutil").iter_modules(M_pkg.__path__):
        try:
            mod = importlib.import_module(f"src.missions.{mi.name}")
        except Exception as e:
            print(f"[skip module {mi.name}] {type(e).__name__}: {e}")
            continue
        for nm, obj in inspect.getmembers(mod, inspect.isclass):
            if issubclass(obj, Mission) and obj.__module__ == mod.__name__:
                by_name[nm] = obj

    # Prefer the order declared on the Robot class.
    order = []
    try:
        from src.hardware.robot import Robot  # type: ignore[import-not-found]

        for m in getattr(Robot, "missions", []):
            order.append(type(m).__name__)
    except Exception:
        pass
    ordered = [(nm, by_name[nm]) for nm in order if nm in by_name]
    # Append any discovered-but-unlisted missions (excluding setup/shutdown).
    for nm in sorted(by_name):
        if nm not in order and "Setup" not in nm and "Shutdown" not in nm:
            ordered.append((nm, by_name[nm]))
    return ordered


async def _simulate(project: Path, scene: Path, start, timeout: float):
    sys.path.insert(0, str(REPO_ROOT / "tests" / "python" / "sim"))
    from _robot_builder import build_robot  # type: ignore[import-not-found]

    from raccoon.testing.sim import pose, use_scene

    _install_fake_transport()  # so UI steps (switch_calibration_set) build+run
    cfg = _make_sim_config()
    robot = _augment_services(build_robot(cfg, enable_localization=True))
    # Use the project's REAL tuned motion PID, not build_robot's generic one —
    # otherwise heading / line-follow controllers behave nothing like the robot.
    try:
        from src.hardware.defs import defs as _defs  # type: ignore[import-not-found]
        from src.hardware.robot import Robot  # type: ignore[import-not-found]

        robot.motion_pid_config = Robot.motion_pid_config
        robot.defs = _defs  # SwitchCalibrationSet reads robot.defs.analog_sensors
    except Exception as e:
        print(f"    [warn] could not load project motion_pid_config/defs: {e}")
    _calibrate_ir_sensors()  # so on_black / line_follow fire against the scene
    missions = _ordered_missions(project)

    traj: list[dict] = []  # {x,y,theta,mi}
    segments: list[dict] = []  # per-mission record
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
            print(f"# [{mi}] {nm}")
            try:
                seq = cls().sequence().resolve()
            except Exception as e:
                print(f"    skip (build): {type(e).__name__}: {e}")
                segments.append(
                    {
                        "name": nm,
                        "i0": i0,
                        "i1": len(traj),
                        "status": f"build-skip: {type(e).__name__}",
                    }
                )
                continue
            try:
                await asyncio.wait_for(seq.run_step(robot), timeout=timeout)
                status = "completed"
            except TimeoutError:
                status = f"TIMEOUT {timeout:.0f}s"
            except Exception as e:
                status = f"{type(e).__name__}: {str(e)[:60]}"
            p = pose()
            print(f"    -> ({p.x:.1f}, {p.y:.1f}, {math.degrees(p.theta):.0f}°)  [{status}]")
            segments.append({"name": nm, "i0": i0, "i1": len(traj), "status": status})
        running["on"] = False
        await asyncio.gather(poller, return_exceptions=True)

    return {"traj": traj, "segments": segments, "start": list(start), "geometry": _robot_geometry()}


def _load_segments(scene: Path):
    from raccoon import sim

    m = sim.WorldMap()
    m.load_ftmap(str(scene))
    return [
        {
            "x0": s.start_x,
            "y0": s.start_y,
            "x1": s.end_x,
            "y1": s.end_y,
            "w": s.width_cm,
            "wall": "WALL" in str(s.kind),
        }
        for s in m.all_segments
    ], (m.table_width_cm, m.table_height_cm)


def _b2w(x, y, theta, fwd, strafe):
    """Body-frame (forward, +strafe=left) → world. +strafe is 90° CCW of forward."""
    c, s = math.cos(theta), math.sin(theta)
    return (x + fwd * c - strafe * s, y + fwd * s + strafe * c)


def _oriented_rect(cx, cy, theta, length, width, fwd_off=0.0, strafe_off=0.0):
    """Corners of a length×width rectangle centred at a body-frame offset."""
    hl, hw = length / 2.0, width / 2.0
    corners = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
    return [_b2w(cx, cy, theta, fwd_off + f, strafe_off + s) for f, s in corners]


def _draw_table(ax, segs, table):
    from matplotlib.patches import Polygon, Rectangle

    tw, th = table
    ax.add_patch(Rectangle((0, 0), tw, th, fill=False, ec="#333", lw=1.2, zorder=1))
    for s in segs:
        # Draw each segment at its TRUE width (cm) as an oriented filled rectangle.
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
            ax.add_patch(
                Polygon(
                    corners, closed=True, facecolor="#111", edgecolor="none", alpha=0.9, zorder=2
                )
            )
    ax.set_aspect("equal")
    ax.set_xlim(-5, tw + 5)
    ax.set_ylim(-5, th + 5)
    ax.set_xlabel("table x (cm)")
    ax.set_ylabel("table y (cm)")
    ax.grid(True, ls=":", alpha=0.3)


def _draw_robot(ax, x, y, theta, geom, *, alpha=1.0, body_color="#4c78a8", labels=False):
    """Draw the robot body + wheels + sensors at world pose (x, y, theta)."""
    from matplotlib.patches import Polygon

    if not geom:
        return
    rc_f = geom.get("rc_fwd", 0.0)
    rc_s = geom.get("rc_strafe", 0.0)
    # Body: length (forward) × width (strafe), centred on the geometric centre
    # which sits at (rc_fwd, rc_strafe) ahead of the rotation centre = pose.
    body = _oriented_rect(
        x, y, theta, geom["length_cm"], geom["width_cm"], fwd_off=rc_f, strafe_off=rc_s
    )
    ax.add_patch(
        Polygon(
            body,
            closed=True,
            facecolor=body_color,
            edgecolor="#22364a",
            lw=1.0,
            alpha=0.18 * alpha + 0.06,
            zorder=3,
        )
    )
    # Heading nose
    nose = _b2w(x, y, theta, rc_f + geom["length_cm"] / 2.0, rc_s)
    ax.plot([x, nose[0]], [y, nose[1]], "-", color="#22364a", lw=1.0, alpha=alpha, zorder=4)
    # Wheels
    for w in geom.get("wheels", []):
        wc = _oriented_rect(x, y, theta, 5.0, 2.4, fwd_off=w["fwd"], strafe_off=w["strafe"])
        ax.add_patch(
            Polygon(
                wc, closed=True, facecolor="#333", edgecolor="none", alpha=0.7 * alpha, zorder=4
            )
        )
    # Sensors
    for i, se in enumerate(geom.get("sensors", [])):
        sx, sy = _b2w(x, y, theta, se["fwd"], se["strafe"])
        col = _COLORS[(i + 2) % len(_COLORS)]
        ax.plot(
            sx,
            sy,
            "o",
            color=col,
            ms=6,
            alpha=alpha,
            zorder=6,
            markeredgecolor="white",
            markeredgewidth=0.6,
        )
        if labels:
            ax.annotate(
                se.get("name", f"s{i}"),
                (sx, sy),
                fontsize=6,
                color=col,
                zorder=6,
                xytext=(3, 3),
                textcoords="offset points",
            )


def _arrow(ax, x, y, theta, color):
    ax.annotate(
        "",
        xy=(x + 7 * math.cos(theta), y + 7 * math.sin(theta)),
        xytext=(x, y),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=1.0, alpha=0.7),
    )


def render(result, segs, table, out_dir: Path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    traj = result["traj"]
    segments = result["segments"]
    start = result["start"]
    geom = result.get("geometry")

    def _ghosts(pts, color):
        """Draw the robot footprint at sparse poses along a mission path."""
        if not pts:
            return
        n = len(pts)
        step = max(1, n // 6)  # ~6 ghosts per mission
        for k in range(0, n, step):
            p = pts[k]
            _draw_robot(ax, p["x"], p["y"], p["theta"], geom, alpha=0.45, body_color=color)

    # --- combined figure ---
    fig, ax = plt.subplots(figsize=(15, 7.5))
    _draw_table(ax, segs, table)
    for mi, seg in enumerate(segments):
        pts = [t for t in traj if t["mi"] == mi]
        if not pts:
            continue
        color = _COLORS[mi % len(_COLORS)]
        ax.plot(
            [p["x"] for p in pts],
            [p["y"] for p in pts],
            "-",
            color=color,
            lw=2.0,
            alpha=0.95,
            label=f"{seg['name']}  [{seg['status']}]",
            zorder=5,
        )
        _draw_robot(
            ax, pts[-1]["x"], pts[-1]["y"], pts[-1]["theta"], geom, alpha=0.8, body_color=color
        )
    _draw_robot(ax, start[0], start[1], start[2], geom, alpha=1.0, body_color="#000", labels=True)
    ax.plot(start[0], start[1], "*", color="black", ms=14, label="start", zorder=7)
    ax.set_title(
        "cube-bot sim run — full mission chain on 2026 game table", fontsize=13, fontweight="bold"
    )
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=7)
    fig.tight_layout()
    out = out_dir / "chain_overview.png"
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")

    # --- per-mission figures: path + robot ghosts + sensors so line/sensor
    #     alignment is visible for debugging ---
    for mi, seg in enumerate(segments):
        pts = [t for t in traj if t["mi"] == mi]
        fig, ax = plt.subplots(figsize=(15, 7.5))
        _draw_table(ax, segs, table)
        ax.plot([t["x"] for t in traj], [t["y"] for t in traj], "-", color="#bbb", lw=0.8)
        color = _COLORS[mi % len(_COLORS)]
        if pts:
            ax.plot(
                [p["x"] for p in pts], [p["y"] for p in pts], "-", color=color, lw=2.2, zorder=5
            )
            _ghosts(pts, color)
            _draw_robot(
                ax,
                pts[0]["x"],
                pts[0]["y"],
                pts[0]["theta"],
                geom,
                alpha=1.0,
                body_color="green",
                labels=True,
            )
            _draw_robot(
                ax, pts[-1]["x"], pts[-1]["y"], pts[-1]["theta"], geom, alpha=1.0, body_color="red"
            )
            ax.plot([], [], "o", color="green", label="start pose")
            ax.plot([], [], "o", color="red", label="end pose")
            ax.legend(loc="best", fontsize=8)
        ax.set_title(f"[{mi}] {seg['name']}  —  {seg['status']}", fontsize=12, fontweight="bold")
        fig.tight_layout()
        out = out_dir / f"{mi:02d}_{seg['name']}.png"
        fig.savefig(out, dpi=120, bbox_inches="tight")
        plt.close(fig)
        print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--project", default=DEFAULT_PROJECT)
    ap.add_argument("--scene", default=None)
    ap.add_argument("--start", default="42.8,25.63,90", help="'x_cm,y_cm,heading_deg'")
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--out", default=str(REPO_ROOT / "sim_run"))
    ap.add_argument(
        "--render-only",
        action="store_true",
        help="re-render from a previously saved trajectory.json (no sim)",
    )
    args = ap.parse_args()

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    scene0 = Path(args.scene) if args.scene else (Path(args.project).resolve() / DEFAULT_SCENE)
    if args.render_only:
        result = json.loads((out_dir / "trajectory.json").read_text())
        if not result.get("geometry"):
            # Backfill robot geometry from the project so the robot/wheels/
            # sensors can be drawn even for trajectories saved before capture.
            proj = Path(args.project).resolve()
            sys.path.insert(0, str(proj))
            cwd = os.getcwd()
            os.chdir(proj)
            try:
                result["geometry"] = _robot_geometry()
            finally:
                os.chdir(cwd)
        segs, table = _load_segments(scene0)
        render(result, segs, table, out_dir)
        print(f"Done → {out_dir}")
        return

    project = Path(args.project).resolve()
    sys.path.insert(0, str(project))
    os.chdir(project)
    os.environ.setdefault("LIBSTP_LOG_LEVEL", "warn")
    os.environ.setdefault("LIBSTP_NO_CALIBRATE", "1")
    os.environ["LIBSTP_TIMING_ENABLED"] = "0"
    # Opt into sim-only fallbacks for unsimulatable stop conditions (e.g.
    # on_analog_flank game-piece detection). Never set on the real robot.
    os.environ["RACCOON_SIM"] = "1"

    import sim_fasttime

    fast = sim_fasttime.enable()  # virtual sim time when RACCOON_SIM_FASTTIME=1

    scene = Path(args.scene) if args.scene else (project / DEFAULT_SCENE)
    sx, sy, sdeg = (float(v) for v in args.start.split(","))
    start = (sx, sy, math.radians(sdeg))

    segs, table = _load_segments(scene)  # real walls (shown in render for context)
    sim_scene = _sim_scene_path(scene)  # drivable copy (ramp walls dropped) for physics
    print(
        f"scene={scene.name} ({table[0]:.0f}x{table[1]:.0f}cm)  "
        f"start=({sx},{sy},{sdeg}°){'  [FASTTIME]' if fast else ''}\n"
    )

    result = asyncio.run(_simulate(project, sim_scene, start, args.timeout))

    (out_dir / "trajectory.json").write_text(json.dumps(result))
    render(result, segs, table, out_dir)
    print(f"\nDone → {out_dir}")
    os._exit(0)  # dodge pybind teardown races


if __name__ == "__main__":
    main()
