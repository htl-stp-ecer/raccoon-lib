"""Log internal STM32 odometry vs calibration-board ground truth, side by side.

Runs ON THE PI. Samples both odometry sources from the raccoon_ring SHM and
writes a timestamped CSV. Use together with analyze.py.

Usage (on the Pi):
    python3 odolog.py <duration_s> [out.csv]
e.g. python3 odolog.py 75 /tmp/drive_log.csv

Columns: t, cx, cy, ch (calib board: cm, cm, deg), ix, iy, ih (internal: m, m, rad)
"""

from __future__ import annotations

import contextlib
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rring import Ring

from raccoon_transport.types.raccoon import scalar_f_t

CHANNELS = {
    "cx": "raccoon/calib_board/odom/pos_x",  # cm   ground truth (PAA optical-flow + IMU)
    "cy": "raccoon/calib_board/odom/pos_y",  # cm
    "ch": "raccoon/calib_board/odom/heading",  # deg
    "ix": "raccoon/odometry/pos_x",  # m    internal STM32 dead reckoning
    "iy": "raccoon/odometry/pos_y",  # m
    "ih": "raccoon/odometry/heading",  # rad
}


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    out = sys.argv[2] if len(sys.argv) > 2 else None
    hz = 50.0

    rings = {k: Ring(v) for k, v in CHANNELS.items()}

    def snap():
        d = {}
        for k, r in rings.items():
            raw = r.latest()
            d[k] = scalar_f_t.decode(raw).value if raw is not None else float("nan")
        return d

    last = None
    with Path(out).open("w") if out else contextlib.nullcontext() as f:
        if f:
            f.write("t,cx,cy,ch,ix,iy,ih\n")
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration:
            s = snap()
            t = time.monotonic() - t0
            if f:
                f.write(
                    f"{t:.3f},{s['cx']:.4f},{s['cy']:.4f},{s['ch']:.4f},"
                    f"{s['ix']:.5f},{s['iy']:.5f},{s['ih']:.5f}\n"
                )
            last = s
            time.sleep(1.0 / hz)
    if last:
        print(
            f"calib(cm) x={last['cx']:.2f} y={last['cy']:.2f} h={last['ch']:.2f} deg | "
            f"internal(m) x={last['ix']:.4f} y={last['iy']:.4f} h={last['ih']:.4f} rad"
        )


if __name__ == "__main__":
    main()
