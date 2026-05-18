"""
Hardware test for autotune phases 5 & 6 on a real robot.

Phase 5 — StaticFriction: sweep PWM positive/negative until BEMF exceeds
a motion threshold. The other motor is braked during each measurement so
the chassis doesn't drive away.

Phase 6 — VelLpfTuner: collect raw BEMF samples at a moderate velocity,
then replay them through IIR filters (alpha 0.05..0.95) in-memory and pick
the alpha that minimises noise+lag score.

Usage:
    python3 test_autotune_hw.py [--ports 0 1] [--inverted 0 1]

Safe to run on a robot without open space — motors spin briefly but the
braked/opposite strategy keeps the chassis roughly stationary.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time


def _log(msg: str) -> None:
    sys.stderr.write(f"[hw_autotune] {msg}\n")
    sys.stderr.flush()


# ---------------------------------------------------------------------------
# Phase 5 — Static Friction
# ---------------------------------------------------------------------------


def measure_static_friction(
    motor,
    other_motor,
    *,
    start_pct: int = 2,
    max_pct: int = 60,
    step_pct: int = 1,
    dwell_ms: int = 60,
    samples_per_step: int = 5,
    motion_threshold: int = 3,
) -> dict:
    """Sweep PWM until BEMF exceeds motion_threshold. Returns kS in both dirs."""
    port = motor.port
    _log(f"Phase 5 — StaticFriction port={port}")

    def _median_bemf(n: int) -> float:
        vals = [motor.get_bemf() for _ in range(n)]
        return statistics.median(vals)

    ks_pos = max_pct  # assume failure (never moved) — will be overwritten
    ks_neg = max_pct
    found_pos = False
    found_neg = False

    # --- positive direction ---
    other_motor.brake()
    time.sleep(0.05)
    for pct in range(start_pct, max_pct + 1, step_pct):
        motor.set_speed(pct)
        time.sleep(dwell_ms / 1000.0)
        bemf = _median_bemf(samples_per_step)
        if abs(bemf) > motion_threshold:
            ks_pos = pct
            found_pos = True
            _log(f"  port={port} kS_positive={pct}% (BEMF={bemf:.0f})")
            break
    motor.brake()
    time.sleep(0.15)

    # --- negative direction ---
    for pct in range(start_pct, max_pct + 1, step_pct):
        motor.set_speed(-pct)
        time.sleep(dwell_ms / 1000.0)
        bemf = _median_bemf(samples_per_step)
        if abs(bemf) > motion_threshold:
            ks_neg = pct
            found_neg = True
            _log(f"  port={port} kS_negative={pct}% (BEMF={bemf:.0f})")
            break
    motor.brake()
    time.sleep(0.15)

    avg = (ks_pos + ks_neg) // 2
    measured = found_pos and found_neg
    if not measured:
        _log(f"  port={port} WARNING: motor never moved — check wiring/power")
    else:
        _log(f"  port={port} kS_avg={avg}%  (pos={ks_pos}%, neg={ks_neg}%)")

    return {
        "motor_port": port,
        "ks_positive_pct": ks_pos,
        "ks_negative_pct": ks_neg,
        "ks_avg_pct": avg,
        "measured": measured,
    }


# ---------------------------------------------------------------------------
# Phase 6 — Velocity LPF alpha
# ---------------------------------------------------------------------------


def _iir_noise_lag_score(
    samples: list[float], alpha: float, noise_weight: float = 1.0, lag_weight: float = 0.5
) -> float:
    """Score an IIR filter on raw BEMF samples (lower = better)."""
    if not samples:
        return float("inf")
    filtered = []
    y = samples[0]
    for x in samples:
        y = alpha * x + (1.0 - alpha) * y
        filtered.append(y)

    # Noise: variance of filtered signal
    mean_f = statistics.mean(filtered)
    noise = statistics.mean((v - mean_f) ** 2 for v in filtered)

    # Lag: mean squared difference between consecutive filtered values
    # (a laggy filter changes slowly → low derivative → low lag score, bad)
    # Invert: penalise filters that track changes too slowly.
    if len(filtered) < 2:
        lag_score = 0.0
    else:
        diffs = [abs(filtered[i] - filtered[i - 1]) for i in range(1, len(filtered))]
        mean_diff = statistics.mean(diffs) if diffs else 1e-9
        # Higher mean_diff = more responsive = less lag penalty
        lag_score = 1.0 / max(mean_diff, 1e-9)

    return noise_weight * noise + lag_weight * lag_score


def tune_vel_lpf(
    motor,
    other_motor,
    *,
    speed_pct: int = 40,
    measure_duration_s: float = 1.0,
    sample_hz: int = 200,
    alpha_min: float = 0.05,
    alpha_max: float = 0.95,
    alpha_step: float = 0.05,
) -> dict:
    """Collect BEMF samples at steady velocity, pick best IIR alpha."""
    port = motor.port
    _log(f"Phase 6 — VelLpfTuner port={port} speed={speed_pct}%")

    # Spin up, let settle
    other_motor.brake()
    motor.set_speed(speed_pct)
    time.sleep(0.3)

    # Sample
    samples: list[float] = []
    interval = 1.0 / sample_hz
    n_samples = int(measure_duration_s * sample_hz)
    for _ in range(n_samples):
        samples.append(float(motor.get_bemf()))
        time.sleep(interval)

    motor.brake()
    other_motor.brake()
    time.sleep(0.1)

    if not samples:
        _log(f"  port={port} ERROR: no samples collected")
        return {"motor_port": port, "tuned_alpha": 0.5, "applied": False}

    steady_mean = statistics.mean(samples)
    steady_std = statistics.stdev(samples) if len(samples) > 1 else 0.0
    _log(f"  port={port} raw BEMF: mean={steady_mean:.1f}, std={steady_std:.2f}, n={len(samples)}")

    # Sweep alpha
    best_alpha = 0.5
    best_score = float("inf")
    initial_alpha = 0.5
    initial_score = _iir_noise_lag_score(samples, initial_alpha)

    steps = round((alpha_max - alpha_min) / alpha_step) + 1
    for i in range(steps):
        alpha = round(alpha_min + i * alpha_step, 6)
        score = _iir_noise_lag_score(samples, alpha)
        if score < best_score:
            best_score = score
            best_alpha = alpha

    _log(
        f"  port={port} best alpha={best_alpha:.2f} (score={best_score:.4f}, "
        f"initial={initial_score:.4f})"
    )

    return {
        "motor_port": port,
        "initial_alpha": initial_alpha,
        "tuned_alpha": best_alpha,
        "min_score": best_score,
        "applied": True,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ports", nargs=2, type=int, default=[0, 1], metavar=("LEFT", "RIGHT"))
    parser.add_argument("--inverted", nargs=2, type=int, default=[0, 0], metavar=("LEFT", "RIGHT"))
    args = parser.parse_args()

    from raccoon.hal import Motor

    ports = args.ports
    inverted = [bool(v) for v in args.inverted]
    motors = [Motor(p, inv) for p, inv in zip(ports, inverted)]

    _log(f"Motors: ports={ports}, inverted={inverted}")
    for m in motors:
        m.brake()
    time.sleep(0.2)

    results: dict = {}

    # Phase 5 — StaticFriction (one motor at a time, other braked)
    sf_results = []
    for i, motor in enumerate(motors):
        other = motors[1 - i]
        try:
            r = measure_static_friction(motor, other)
            sf_results.append(r)
        except Exception as e:
            _log(f"Phase 5 port={motor.port} FAILED: {e}")
            sf_results.append({"motor_port": motor.port, "error": str(e)})
    results["static_friction"] = sf_results

    # Brief pause between phases
    for m in motors:
        m.brake()
    time.sleep(0.5)

    # Phase 6 — VelLpf (one motor at a time, other braked)
    lpf_results = []
    for i, motor in enumerate(motors):
        other = motors[1 - i]
        try:
            r = tune_vel_lpf(motor, other)
            lpf_results.append(r)
        except Exception as e:
            _log(f"Phase 6 port={motor.port} FAILED: {e}")
            lpf_results.append({"motor_port": motor.port, "error": str(e)})
    results["vel_lpf"] = lpf_results

    # Safe shutdown
    for m in motors:
        m.off()

    print("RESULTS:" + json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
