#!/usr/bin/env python3
"""Extract the TRUE driving geometry (black tape lines) of the 2026 Botball game
table from the official 100%-scale .glb CAD model, and rebuild / verify a
cube-bot ``.ftmap`` against it.

The hand-authored ``config/2026-game-table-v2.ftmap`` is known-broken (the ramp /
upper-deck lines are mis-placed and over-extended), which shows up in the sim as
M060+ timeouts. The .glb is ground truth, so we read the tape strips straight out
of it and emit a corrected map.

How it works
------------
The .glb is a doubled, point-symmetric table; the cube-bot plays one half
(glTF X >= 0). glTF is Y-up and 100% scale (1 unit = 1 m), so the table surface
is the X(depth)-Z(width) plane. We extract every flat, near-black mesh AABB on
that plane (the duct-tape strips), then map glTF -> ftmap cm with the frame that
was VERIFIED against the two trusted reference lines:

    ftmap_x = (Z_REF - glb_Z) * 100      # width  0..~236   (ramp lands east)
    ftmap_y = (glb_X - X_REF) * 100      # depth  0..~106

X_REF / Z_REF are auto-calibrated from the two full-width reference lines
(ftmap y = 30.99 / 64.01) and the east table edge. Tape width ~5 cm; strips
wider than that on their short axis are filled ZONES (dock/warehouse), not lines.
Strips at glTF Y < ~0.10 m are the ground layer; higher strips are the ramp deck.

Outputs (under --out): ``<name>.ftmap`` (corrected), ``ftmap_diff.md`` (per-line
comparison vs the current map), and ``ftmap_overlay.png``.

    .venv-test/bin/python tools/extract_ftmap_from_glb.py \
        --glb "/home/tobias/Downloads/2026 Botball Full Game Table.glb" \
        --against /path/to/cube-bot/config/2026-game-table-v2.ftmap \
        --out glb_ftmap
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np

# Tape geometry thresholds (metres, glTF units).
GROUND_Y_MAX = 0.10  # strips below this height are the ground layer
DARK_MAX = 0.10  # baseColor max channel below this == "black tape"
FLAT_Y_MAX = 0.05  # AABB vertical thickness of a flat strip
# A tape strip lying on the inclined ramp surface has a large vertical (Y) AABB
# extent (it rises with the slope) so it fails FLAT_Y_MAX. These bounds capture
# such tilted guide lines (the slope line rises ~0.15 m) while excluding tall
# perimeter walls.
RAMP_SLOPE_Y_MIN = 0.06
RAMP_SLOPE_Y_MAX = 0.20
LINE_WIDTH_MAX_CM = 9.0  # short-axis width above this == filled zone, not a line
RAMP_Z_CM = 10.0  # ftmap zCm for the ramp layer (matches existing map)
MATCH_TOL_CM = 4.0  # diff: a current line "matches" within this perpendicular dist


def _load_glb(path: Path):
    with open(path, "rb") as f:  # noqa: PTH123
        f.read(12)
        clen, _ = struct.unpack("<II", f.read(8))
        return json.loads(f.read(clen))


def _node_matrix(n):
    if "matrix" in n:
        return np.array(n["matrix"], float).reshape(4, 4).T
    M = np.eye(4)
    if "scale" in n:
        M[:3, :3] = np.diag(n["scale"])
    if "rotation" in n:
        x, y, z, w = n["rotation"]
        R = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        M[:3, :3] = R @ M[:3, :3]
    if "translation" in n:
        M[:3, 3] = n["translation"]
    return M


def _world_transforms(js):
    nodes = js["nodes"]
    world = {}

    def walk(i, P):
        M = P @ _node_matrix(nodes[i])
        world[i] = M
        for c in nodes[i].get("children", []):
            walk(c, M)

    for r in js["scenes"][0]["nodes"]:
        walk(r, np.eye(4))
    return world


def _aabb_world(M, lo, hi):
    pts = np.array(
        [
            [lo[0] if i & 1 else hi[0], lo[1] if i & 2 else hi[1], lo[2] if i & 4 else hi[2], 1.0]
            for i in range(8)
        ]
    )
    w = (M @ pts.T).T[:, :3]
    return w.min(0), w.max(0)


def extract_tape_strips(js):
    """Return flat black strips on the cube-bot half as (lo, hi) world AABBs."""
    nodes, meshes, accs, mats = js["nodes"], js["meshes"], js["accessors"], js["materials"]
    world = _world_transforms(js)

    def is_dark(mi):
        if mi is None or mi >= len(mats):
            return False
        c = mats[mi].get("pbrMetallicRoughness", {}).get("baseColorFactor", [1, 1, 1, 1])
        return max(c[:3]) < DARK_MAX

    strips = []
    for i, n in enumerate(nodes):
        mi = n.get("mesh")
        if mi is None:
            continue
        for prim in meshes[mi].get("primitives", []):
            if not is_dark(prim.get("material")):
                continue
            pa = prim["attributes"].get("POSITION")
            if pa is None or accs[pa].get("min") is None:
                continue
            lo, hi = _aabb_world(world[i], accs[pa]["min"], accs[pa]["max"])
            if lo[0] < 0.05:  # cube-bot half only (glTF X >= 0), skip centre seam
                continue
            if (hi[1] - lo[1]) < FLAT_Y_MAX:
                strips.append((lo, hi))
    return strips


def extract_ramp_structure(js, x_ref, z_ref, dx):
    """Extract the elevated platform deck + perimeter walls + the ground->ramp
    transition seam from the .glb (ground truth), in ftmap cm. The hand-authored
    map put the ramp on the WEST; the .glb has it EAST — this fixes that."""
    nodes, meshes, accs = js["nodes"], js["meshes"], js["accessors"]
    world = _world_transforms(js)

    def to_ft(gx, gz):
        return (z_ref - gz) * 100.0 + dx, (gx - x_ref) * 100.0

    def clampx(x):
        return max(0.0, min(236.42, x))

    def clampy(y):
        return max(0.0, min(105.918, y))

    decks, walls = [], []
    for i, n in enumerate(nodes):
        mi = n.get("mesh")
        if mi is None:
            continue
        for prim in meshes[mi].get("primitives", []):
            pa = prim["attributes"].get("POSITION")
            if pa is None or accs[pa].get("min") is None:
                continue
            lo, hi = _aabb_world(world[i], accs[pa]["min"], accs[pa]["max"])
            if lo[0] < 0.05:  # cube-bot half
                continue
            sz = hi - lo
            a, b = to_ft(lo[0], lo[2]), to_ft(hi[0], hi[2])
            x0, x1 = sorted((a[0], b[0]))
            y0, y1 = sorted((a[1], b[1]))
            # elevated flat panel => deck surface. Only the HIGH platform deck
            # (~17.6 cm); a lower 6 cm shelf exists but is not the ramp layer.
            if sz[1] < 0.04 and lo[1] > 0.12 and sz[0] * sz[2] > 0.05:
                decks.append((x0, y0, x1, y1, (lo[1] + hi[1]) / 2 * 100))
            # tall thin panel => wall; segment along its base, on its long axis
            elif sz[1] > 0.06 and min(sz[0], sz[2]) < 0.04 and max(sz[0], sz[2]) > 0.08:
                if (x1 - x0) >= (y1 - y0):  # horizontal wall
                    yc = (y0 + y1) / 2
                    walls.append((clampx(x0), clampy(yc), clampx(x1), clampy(yc)))
                else:
                    xc = (x0 + x1) / 2
                    walls.append((clampx(xc), clampy(y0), clampx(xc), clampy(y1)))

    # deck footprint (union of elevated panels) + height
    dxs = [v for d in decks for v in (d[0], d[2])]
    dys = [v for d in decks for v in (d[1], d[3])]
    deck = (
        {
            "x0": clampx(min(dxs)),
            "y0": clampy(min(dys)),
            "x1": clampx(max(dxs)),
            "y1": clampy(max(dys)),
            "zCm": round(float(np.median([d[4] for d in decks])), 1),
        }
        if decks
        else None
    )

    # dedup walls (same orientation + position to ~3cm, merge spans)
    merged = {}
    for x0, y0, x1, y1 in walls:
        horiz = abs(y1 - y0) < abs(x1 - x0)
        pos = round((y0 if horiz else x0) / 3) * 3
        key = (horiz, pos)
        xs = merged.setdefault(key, [1e9, -1e9, 1e9, -1e9])
        merged[key] = [
            min(xs[0], x0, x1),
            max(xs[1], x0, x1),
            min(xs[2], y0, y1),
            max(xs[3], y0, y1),
        ]
    wall_segs = []
    for (horiz, _p), (xa, xb, ya, yb) in merged.items():
        if horiz:
            wall_segs.append(
                (round(xa, 1), round((ya + yb) / 2, 1), round(xb, 1), round((ya + yb) / 2, 1))
            )
        else:
            wall_segs.append(
                (round((xa + xb) / 2, 1), round(ya, 1), round((xa + xb) / 2, 1), round(yb, 1))
            )

    # ground->ramp transition: a SINGLE vertical seam (from == to) just west of
    # the deck's west edge, spanning the deck y-range. The robot switches to the
    # ramp layer when it crosses the seam climbing onto the platform and STAYS on
    # it for the whole deck. NOTE: from/to must be the SAME edge — with two
    # different edges the bidirectional logic makes the ramp layer active only
    # *between* them (an island on the bare slope), so the robot drops back to
    # ground exactly where the deck line begins and never line-follows it.
    transition = None
    if deck:
        seam_x = round(max(0.0, deck["x0"] - 12.0), 1)  # ~12cm west of the deck line
        edge = {"startX": seam_x, "startY": deck["y0"], "endX": seam_x, "endY": deck["y1"]}
        transition = {
            "id": "ramp-1-glb",
            "name": "Ramp 1 (glb)",
            "fromLayer": "ground",
            "toLayer": "layer-2",
            "from": dict(edge),
            "to": dict(edge),
            "bidirectional": True,
            "costMultiplier": 1,
            "widthCm": 15,
        }
    return deck, wall_segs, transition


def extract_ramp_slope_lines(js, x_ref, z_ref, dx):
    """Extract the guide tape line(s) running UP the ramp slope.

    The deck guide line continues down the inclined ramp surface to the ground.
    Because that strip lies on a SLOPE its world AABB has a large vertical (Y)
    extent, so ``extract_tape_strips`` (which keeps only near-flat strips,
    Y-thickness < ``FLAT_Y_MAX``) discards it — leaving a gap in the line the
    robot line-follows while climbing (the cause of the ramp/upper-deck
    mis-positioning). Recover those tilted dark strips and project them to their
    2D centreline. The caller emits them on BOTH the ground and ramp layers so
    the line is sensed continuously across the layer transition.
    """
    nodes, meshes, accs, mats = js["nodes"], js["meshes"], js["accessors"], js["materials"]
    world = _world_transforms(js)

    def is_dark(mi):
        if mi is None or mi >= len(mats):
            return False
        c = mats[mi].get("pbrMetallicRoughness", {}).get("baseColorFactor", [1, 1, 1, 1])
        return max(c[:3]) < DARK_MAX

    def to_ft(gx, gz):
        return (z_ref - gz) * 100.0 + dx, (gx - x_ref) * 100.0

    out = []
    for i, n in enumerate(nodes):
        mi = n.get("mesh")
        if mi is None:
            continue
        for prim in meshes[mi].get("primitives", []):
            if not is_dark(prim.get("material")):
                continue
            pa = prim["attributes"].get("POSITION")
            if pa is None or accs[pa].get("min") is None:
                continue
            lo, hi = _aabb_world(world[i], accs[pa]["min"], accs[pa]["max"])
            if lo[0] < 0.05:  # cube-bot half only
                continue
            ythk = hi[1] - lo[1]
            yc = (lo[1] + hi[1]) / 2.0
            if not (RAMP_SLOPE_Y_MIN < ythk < RAMP_SLOPE_Y_MAX):
                continue  # flat (already captured) or a tall wall
            if not (0.04 < yc < 0.17):
                continue  # not spanning ground->deck height
            a, b = to_ft(lo[0], lo[2]), to_ft(hi[0], hi[2])
            x0, x1 = sorted([a[0], b[0]])
            y0, y1 = sorted([a[1], b[1]])
            wx, wy = x1 - x0, y1 - y0
            length, width = max(wx, wy), min(wx, wy)
            if length < 10.0 or width > LINE_WIDTH_MAX_CM:
                continue  # not a line-like strip
            if wx >= wy:  # horizontal: constant y at centre
                seg = (round(x0, 1), round((y0 + y1) / 2, 1), round(x1, 1), round((y0 + y1) / 2, 1))
            else:  # vertical: constant x at centre
                seg = (round((x0 + x1) / 2, 1), round(y0, 1), round((x0 + x1) / 2, 1), round(y1, 1))
            out.append(
                {"seg": seg, "width": round(width, 2), "length": round(length, 1), "kind": "line"}
            )
    # de-duplicate identical centrelines (a slope strip may be several prims)
    seen, uniq = set(), []
    for s in out:
        if s["seg"] in seen:
            continue
        seen.add(s["seg"])
        uniq.append(s)
    return uniq


def calibrate_frame(strips):
    """Find X_REF / Z_REF so the two full-width reference lines land on
    ftmap y = 30.99 / 64.01 and the ramp ends up east. Returns (x_ref, z_ref)."""
    # Full-width strips span almost the entire Z extent.
    zspan = max(hi[2] - lo[2] for lo, hi in strips)
    full = [(lo, hi) for lo, hi in strips if (hi[2] - lo[2]) > 0.9 * zspan]
    xs = sorted((lo[0] + hi[0]) / 2.0 for lo, hi in full)
    # The two reference lines are the lowest pair of full-width strips.
    x_lo = xs[0]
    # ftmap y of the lower reference line is 30.99 cm -> X_REF = x_lo - 0.3099.
    x_ref = x_lo - 0.3099
    # Z_REF: east edge. Width axis spans [-zmax, +zmax]; ramp is on the +x (east)
    # side after flipping, so ftmap_x = (z_ref - z) * 100 with z_ref = +zmax.
    z_ref = max(hi[2] for lo, hi in strips)
    return x_ref, z_ref


def strips_to_lines(strips, x_ref, z_ref):
    """Map strips to ftmap-frame centreline segments. Returns list of dicts."""
    out = []
    for lo, hi in strips:
        # ftmap_x = (z_ref - glb_z)*100 ; ftmap_y = (glb_x - x_ref)*100
        fx = [(z_ref - lo[2]) * 100.0, (z_ref - hi[2]) * 100.0]
        fy = [(lo[0] - x_ref) * 100.0, (hi[0] - x_ref) * 100.0]
        x0, x1 = min(fx), max(fx)
        y0, y1 = min(fy), max(fy)
        wx, wy = x1 - x0, y1 - y0
        width = min(wx, wy)
        length = max(wx, wy)
        layer = "ground" if (lo[1] + hi[1]) / 2.0 < GROUND_Y_MAX else "ramp"
        kind = "zone" if width > LINE_WIDTH_MAX_CM else "line"
        if wx >= wy:  # horizontal line: constant y at centre
            yc = (y0 + y1) / 2.0
            seg = (x0, yc, x1, yc)
        else:  # vertical line: constant x at centre
            xc = (x0 + x1) / 2.0
            seg = (xc, y0, xc, y1)
        out.append(
            {
                "seg": seg,
                "width": round(width, 2),
                "length": round(length, 1),
                "layer": layer,
                "kind": kind,
                "y_m": round((lo[1] + hi[1]) / 2.0, 3),
            }
        )
    return out


def merge_lines(lines, pos_tol=2.0):
    """Merge near-collinear same-orientation lines on the same layer."""
    buckets = {}
    for ln in lines:
        x0, y0, x1, y1 = ln["seg"]
        horiz = abs(y1 - y0) < abs(x1 - x0)
        pos = round((y0 if horiz else x0) / pos_tol) * pos_tol
        key = (ln["layer"], ln["kind"], horiz, pos)
        buckets.setdefault(key, []).append(ln)
    merged = []
    for (layer, kind, horiz, _pos), grp in buckets.items():
        if horiz:
            y = np.mean([(g["seg"][1] + g["seg"][3]) / 2 for g in grp])
            xs = [v for g in grp for v in (g["seg"][0], g["seg"][2])]
            seg = (min(xs), y, max(xs), y)
        else:
            x = np.mean([(g["seg"][0] + g["seg"][2]) / 2 for g in grp])
            ys = [v for g in grp for v in (g["seg"][1], g["seg"][3])]
            seg = (x, min(ys), x, max(ys))
        width = float(np.mean([g["width"] for g in grp]))
        merged.append(
            {
                "seg": tuple(round(v, 2) for v in seg),
                "width": round(width, 2),
                "layer": layer,
                "kind": kind,
            }
        )
    return merged


def align_x_to_current(lines, current_path: Path):
    """Frame-origin fix: the .glb width origin sits a few cm off the map's x=0
    convention. Measure the median x offset between .glb ground verticals and the
    map's matching ground verticals (robust — broken ramp lines are outliers) and
    shift every line by it, so the corrected map drops into the cube-bot's
    existing mission-coordinate frame. Returns the applied dx (cm)."""
    cur = json.loads(current_path.read_text())
    cur_vx = []
    for L in cur.get("layers", []):
        if L.get("zCm", 0) != 0:
            continue
        for ln in L.get("lines", []):
            if ln.get("kind") == "line" and abs(ln["endX"] - ln["startX"]) < 3:
                cur_vx.append(
                    (ln["startX"], min(ln["startY"], ln["endY"]), max(ln["startY"], ln["endY"]))
                )
    offsets = []
    for t in lines:
        if t["kind"] != "line" or t["layer"] != "ground":
            continue
        x0, y0, x1, y1 = t["seg"]
        if abs(x1 - x0) > 3:
            continue  # vertical only
        gx, gy0, gy1 = x0, min(y0, y1), max(y0, y1)
        for cx, cy0, cy1 in cur_vx:
            if min(gy1, cy1) - max(gy0, cy0) > 15 and abs(cx - gx) < 6:
                offsets.append(cx - gx)
    if not offsets:
        return 0.0
    dx = float(np.median(offsets))
    for t in lines:
        x0, y0, x1, y1 = t["seg"]
        t["seg"] = (round(x0 + dx, 2), y0, round(x1 + dx, 2), y1)
    return dx


def _seg_dist(a, b):
    """Perpendicular-ish distance between two axis-aligned segments (cm)."""
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    a_h = abs(ay1 - ay0) < abs(ax1 - ax0)
    b_h = abs(by1 - by0) < abs(bx1 - bx0)
    if a_h != b_h:
        return 1e9
    if a_h:
        if max(ax0, ax1) < min(bx0, bx1) - 5 or min(ax0, ax1) > max(bx0, bx1) + 5:
            return 1e9  # no overlap along the run
        return abs((ay0 + ay1) / 2 - (by0 + by1) / 2)
    if max(ay0, ay1) < min(by0, by1) - 5 or min(ay0, ay1) > max(by0, by1) + 5:
        return 1e9
    return abs((ax0 + ax1) / 2 - (bx0 + bx1) / 2)


def diff_against(current_path: Path, truth_lines):
    cur = json.loads(current_path.read_text())
    rows = []
    truth_by_layer = {"ground": [], "ramp": []}
    for t in truth_lines:
        if t["kind"] == "line":
            truth_by_layer["ground" if t["layer"] == "ground" else "ramp"].append(t)
    layer_map = {}
    for L in cur.get("layers", []):
        nm = (L.get("name", "") + L.get("id", "")).lower()
        layer_map[L["id"]] = "ramp" if "ramp" in nm or "rampe" in nm or "2" in nm else "ground"
    for L in cur.get("layers", []):
        lk = layer_map[L["id"]]
        for ln in L.get("lines", []):
            if ln.get("kind") != "line":
                continue
            seg = (ln["startX"], ln["startY"], ln["endX"], ln["endY"])
            cands = truth_by_layer.get(lk, []) + truth_by_layer["ground"]
            best, bd = None, 1e9
            for t in cands:
                d = _seg_dist(seg, t["seg"])
                if d < bd:
                    bd, best = d, t
            rows.append(
                {
                    "layer": L.get("name", L["id"]),
                    "cur": seg,
                    "truth": best["seg"] if best else None,
                    "dist": bd,
                }
            )
    return rows


def write_ftmap(out_path: Path, truth_lines, table, deck, walls, transition):
    ramp_z = deck["zCm"] if deck else RAMP_Z_CM
    layers = {
        "ground": {"id": "ground", "name": "Ground", "zCm": 0, "lines": []},
        "ramp": {"id": "layer-2", "name": "Rampe", "zCm": ramp_z, "lines": []},
    }
    # NOTE: tape endpoints are intentionally NOT clamped to the table rim.
    # GLB tape meshes are modelled slightly past the edge (lines reaching
    # y≈-6 / x≈239), which looks wrong in the render — but trimming them to
    # the table bounds DE-STABILISES the upper-deck missions (M070/M080/M100
    # then time out, vs only M070 with the overhang kept). The off-table
    # overhang is load-bearing for the sensor-driven deck maneuvers, so it
    # stays. The render simply shows lines leaving the table; that is cosmetic.
    for t in truth_lines:
        if t["kind"] != "line":
            continue
        x0, y0, x1, y1 = t["seg"]
        layers["ground" if t["layer"] == "ground" else "ramp"]["lines"].append(
            {
                "startX": x0,
                "startY": y0,
                "endX": x1,
                "endY": y1,
                "widthCm": round(t["width"], 2),
                "kind": "line",
            }
        )
    # NOTE: GLB-derived ramp perimeter walls are intentionally NOT emitted. In a
    # 2D kinematic sim the robot drives ONTO the ramp (it's a drivable layer), so
    # those panels become phantom collision barriers in the drive path and trap
    # the robot the instant it switches to the ramp layer. The real table border
    # is auto-added by the sim's buildCollisionWalls. (`walls` is still computed
    # for diagnostics / future use.)
    _ = walls
    doc = {
        "format": "flowchart-table-map",
        "version": 2,
        "table": table,
        "layers": [layers["ground"], layers["ramp"]],
        "transitions": [transition] if transition else [],
        "activeLayerId": "ground",
    }
    out_path.write_text(json.dumps(doc, indent=2))
    return doc


def render_overlay(out_png: Path, truth_lines, current_path: Path, table):
    import matplotlib as mpl

    mpl.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    fig, axes = plt.subplots(2, 1, figsize=(14, 11))
    cur = json.loads(current_path.read_text()) if current_path.exists() else {"layers": []}
    cur_by_layer = {}
    for L in cur.get("layers", []):
        cur_by_layer["ramp" if L.get("zCm", 0) else "ground"] = L.get("lines", [])

    for ax, layer in zip(axes, ("ground", "ramp"), strict=False):
        ax.add_patch(
            plt.Rectangle(
                (0, 0), table["widthCm"], table["heightCm"], fill=False, ec="#333", lw=1.2
            )
        )
        for ln in cur_by_layer.get(layer, []):
            c = "#d62728" if ln.get("kind") == "line" else "#d6a0a0"
            ax.plot(
                [ln["startX"], ln["endX"]],
                [ln["startY"], ln["endY"]],
                "--",
                color=c,
                lw=2.0,
                alpha=0.8,
            )
        for t in truth_lines:
            if t["layer"] != layer:
                continue
            x0, y0, x1, y1 = t["seg"]
            c = "#2ca02c" if t["kind"] == "line" else "#9bbf9b"
            ax.plot([x0, x1], [y0, y1], "-", color=c, lw=3.0, alpha=0.7)
        ax.set_aspect("equal")
        ax.set_xlim(-10, table["widthCm"] + 10)
        ax.set_ylim(-10, table["heightCm"] + 10)
        ax.set_title(
            f"{layer} layer — red dashed = current ftmap, green = .glb ground truth",
            fontweight="bold",
        )
        ax.grid(True, ls=":", alpha=0.3)
        ax.legend(
            handles=[
                Line2D([], [], color="#d62728", ls="--", label="current (broken)"),
                Line2D([], [], color="#2ca02c", label=".glb truth"),
            ],
            fontsize=8,
        )
    fig.tight_layout()
    fig.savefig(out_png, dpi=120, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--glb", default="/home/tobias/Downloads/2026 Botball Full Game Table.glb")
    ap.add_argument("--against", default=None, help="current .ftmap to diff/overlay against")
    ap.add_argument("--out", default="glb_ftmap")
    ap.add_argument("--name", default="2026-game-table-glb")
    args = ap.parse_args()

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    print("parsing .glb …", flush=True)
    js = _load_glb(Path(args.glb))
    strips = extract_tape_strips(js)
    x_ref, z_ref = calibrate_frame(strips)
    print(f"  {len(strips)} tape strips; frame X_REF={x_ref:.4f} Z_REF={z_ref:.4f}")
    lines = merge_lines(strips_to_lines(strips, x_ref, z_ref))
    nl = sum(1 for ln in lines if ln["kind"] == "line")
    print(
        f"  -> {nl} tape lines + {len(lines) - nl} zones "
        f"({sum(ln['layer'] == 'ramp' for ln in lines)} on ramp)"
    )

    table = {"widthCm": 236.4232, "heightCm": 105.918}
    against = Path(args.against) if args.against else None
    dx = 0.0
    if against:
        table = json.loads(against.read_text()).get("table", table)
        dx = align_x_to_current(lines, against)
        print(f"  frame x-offset aligned to current map: dx={dx:+.2f} cm")

    deck, walls, transition = extract_ramp_structure(js, x_ref, z_ref, dx)
    if deck:
        print(
            f"  ramp deck: x[{deck['x0']:.0f},{deck['x1']:.0f}] y[{deck['y0']:.0f},"
            f"{deck['y1']:.0f}] zCm={deck['zCm']} ({len(walls)} walls)"
        )
        print(
            f"  transition (ground->ramp): foot x={transition['from']['startX']:.0f} "
            f"-> top x={transition['to']['startX']:.0f}, y[{deck['y0']:.0f},{deck['y1']:.0f}]"
        )

    # The guide line that runs UP the ramp slope is tilted, so the flat-strip
    # extractor missed it. Recover it and emit on BOTH layers for continuous
    # line-following across the ground->ramp transition.
    for s in extract_ramp_slope_lines(js, x_ref, z_ref, dx):
        for layer in ("ground", "ramp"):
            lines.append({**s, "layer": layer})
        print(f"  + ramp-slope guide line {s['seg']} w={s['width']} (ground + ramp)")

    print("\n# .glb ground-truth tape lines (ftmap cm):")
    for ln in sorted(lines, key=lambda x: (x["layer"], x["seg"][1])):
        if ln["kind"] == "line":
            x0, y0, x1, y1 = ln["seg"]
            print(
                f"  [{ln['layer']:6}] ({x0:6.1f},{y0:6.1f})->({x1:6.1f},{y1:6.1f}) w={ln['width']}"
            )

    out_ftmap = out_dir / f"{args.name}.ftmap"
    write_ftmap(out_ftmap, lines, table, deck, walls, transition)
    print(f"\nwrote corrected map -> {out_ftmap}")

    if against:
        rows = diff_against(against, lines)
        md = [
            "# ftmap diff — current vs .glb ground truth\n",
            f"Tolerance {MATCH_TOL_CM} cm. `OFF` = current line is misplaced/over-extended.\n",
            "| layer | current line | nearest .glb truth | Δ cm | verdict |",
            "|---|---|---|--:|---|",
        ]
        nbad = 0
        for r in rows:
            cur = "({:.0f},{:.0f})->({:.0f},{:.0f})".format(*r["cur"])
            tr = "({:.0f},{:.0f})->({:.0f},{:.0f})".format(*r["truth"]) if r["truth"] else "—"
            ok = r["dist"] <= MATCH_TOL_CM
            verdict = "ok" if ok else "**OFF**"
            if not ok:
                nbad += 1
            d = "—" if r["dist"] > 1e8 else f"{r['dist']:.1f}"
            md.append(f"| {r['layer']} | {cur} | {tr} | {d} | {verdict} |")
        md.insert(2, f"\n**{nbad} of {len(rows)} current lines are off ground truth.**\n")
        (out_dir / "ftmap_diff.md").write_text("\n".join(md) + "\n")
        print(f"wrote diff -> {out_dir / 'ftmap_diff.md'}  ({nbad}/{len(rows)} lines OFF)")
        render_overlay(out_dir / "ftmap_overlay.png", lines, against, table)
        print(f"wrote overlay -> {out_dir / 'ftmap_overlay.png'}")


if __name__ == "__main__":
    main()
