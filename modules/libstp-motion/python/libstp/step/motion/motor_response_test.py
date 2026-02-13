"""
Motor response time diagnostic.

Measures actual velocity response via BEMF readings for both
setVelocity (firmware PID) and setSpeed (direct PWM).
Also measures full system response (chassis command → IMU heading).

Usage:
    from libstp.step.motion import motor_response_test
    await motor_response_test(robot)
    await motor_response_test(robot, test_percent=75)
"""
from __future__ import annotations

import asyncio
import time
from typing import TYPE_CHECKING

from libstp.foundation import ChassisVelocity

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


def _log(msg: str) -> None:
    print(f"[motor_diag] {msg}")


def _ema(samples: list[tuple[float, float]], alpha: float = 0.3) -> list[tuple[float, float]]:
    out = []
    fv = 0.0
    for t, v in samples:
        fv = alpha * v + (1 - alpha) * fv
        out.append((t, fv))
    return out


def _rise_analysis(filtered: list[tuple[float, float]]) -> dict:
    if len(filtered) < 5:
        return {"steady_state": 0.0}

    n = max(1, len(filtered) // 5)
    ss = sum(v for _, v in filtered[-n:]) / n
    result: dict = {"steady_state": ss}

    if abs(ss) < 0.01:
        return result

    for name, frac in [("dead_time_ms", 0.05), ("rise_10_ms", 0.10),
                       ("rise_50_ms", 0.50), ("rise_90_ms", 0.90)]:
        result[name] = None
        for t, v in filtered:
            if abs(v) >= abs(ss) * frac:
                result[name] = round(t * 1000)
                break
    return result


def _stop_analysis(filtered: list[tuple[float, float]]) -> dict:
    if len(filtered) < 5:
        return {"initial_velocity": 0.0}

    n = min(10, len(filtered))
    v0 = max((abs(v) for _, v in filtered[:n]), default=0.0)
    result: dict = {"initial_velocity": v0}

    if abs(v0) < 0.01:
        return result

    for name, frac in [("stop_50_ms", 0.50), ("stop_90_ms", 0.10), ("stop_98_ms", 0.02)]:
        result[name] = None
        for t, v in filtered:
            if abs(v) <= v0 * frac:
                result[name] = round(t * 1000)
                break
    return result


def _print_rise(result: dict) -> None:
    ss = result.get("steady_state", 0)
    _log(f"  Steady state: {ss:.1f}")

    for key in ("dead_time_ms", "rise_10_ms", "rise_50_ms", "rise_90_ms"):
        val = result.get(key)
        _log(f"  {key.replace('_ms', '')}: {val} ms" if val is not None
             else f"  {key.replace('_ms', '')}: --")

    r10, r90 = result.get("rise_10_ms"), result.get("rise_90_ms")
    if r10 is not None and r90 is not None:
        _log(f"  rise (10→90): {r90 - r10} ms")

    _print_ts(result)


def _print_stop(result: dict) -> None:
    v0 = result.get("initial_velocity", 0)
    _log(f"  Initial: {v0:.1f}")

    for key in ("stop_50_ms", "stop_90_ms", "stop_98_ms"):
        val = result.get(key)
        if val is not None:
            _log(f"  {key.replace('_ms', '').replace('stop_', 'to_')}%: {val} ms")

    _print_ts(result)


def _print_ts(result: dict) -> None:
    samples = result.get("samples", [])
    if not samples:
        return
    _log("  t(s)     value   ")
    last = -1.0
    for t, v in samples:
        if t - last >= 0.05:
            bar = "#" * min(int(abs(v) / 2), 40)
            _log(f"    {t:5.3f}  {v:+7.1f}  |{bar}")
            last = t


# ------------------------------------------------------------------
# BEMF recording
# ------------------------------------------------------------------
async def _record_bemf(motor, duration: float,
                       rate: float = 200.0,
                       t0_override: float | None = None) -> list[tuple[float, float]]:
    """Poll motor.get_bemf() at `rate` Hz for `duration` seconds.

    If t0_override is given, timestamps are relative to that monotonic time
    (e.g. the moment a command was sent), so dead-time is measured from the
    actual command rather than from the start of recording.
    """
    dt_sleep = 1.0 / rate
    samples: list[tuple[float, float]] = []
    t0 = t0_override if t0_override is not None else time.monotonic()
    t_end = time.monotonic() + duration

    while time.monotonic() < t_end:
        await asyncio.sleep(dt_sleep)
        samples.append((time.monotonic() - t0, float(motor.get_bemf())))

    return samples


# ------------------------------------------------------------------
# Heading-rate recording
# ------------------------------------------------------------------
async def _record_heading_rate(robot: "GenericRobot", duration: float,
                               rate: float = 100.0) -> list[tuple[float, float]]:
    samples: list[tuple[float, float]] = []
    prev_heading = robot.odometry.get_heading()
    t0 = time.monotonic()
    prev_t = t0
    dt_sleep = 1.0 / rate

    while time.monotonic() - t0 < duration:
        await asyncio.sleep(dt_sleep)
        now = time.monotonic()
        dt = now - prev_t
        prev_t = now
        if dt < 1e-6:
            continue
        robot.odometry.update(dt)
        heading = robot.odometry.get_heading()
        samples.append((now - t0, (heading - prev_heading) / dt))
        prev_heading = heading

    return samples


# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------
async def _test_set_speed(motor, percent: int, duration: float) -> dict:
    """Step response: setSpeed (direct PWM) → BEMF."""
    motor.brake()
    await asyncio.sleep(0.5)
    t_cmd = time.monotonic()
    motor.set_speed(percent)
    t_after = time.monotonic()
    _log(f"  set_speed() called at {t_cmd:.6f}, returned at {t_after:.6f} "
         f"(call took {(t_after - t_cmd)*1000:.2f} ms)")
    raw = await _record_bemf(motor, duration, t0_override=t_cmd)
    motor.brake()
    await asyncio.sleep(0.3)
    f = _ema(raw)
    return {**_rise_analysis(f), "cmd_sent_at": t_cmd, "samples": f}


async def _test_set_velocity(motor, bemf_target: int, duration: float) -> dict:
    """Step response: setVelocity (firmware BEMF PID) → BEMF."""
    motor.brake()
    await asyncio.sleep(0.5)
    t_cmd = time.monotonic()
    motor.set_velocity(bemf_target)
    t_after = time.monotonic()
    _log(f"  set_velocity() called at {t_cmd:.6f}, returned at {t_after:.6f} "
         f"(call took {(t_after - t_cmd)*1000:.2f} ms)")
    raw = await _record_bemf(motor, duration, t0_override=t_cmd)
    motor.brake()
    await asyncio.sleep(0.3)
    f = _ema(raw)
    return {**_rise_analysis(f), "cmd_sent_at": t_cmd, "samples": f}


async def _test_brake(motor, percent: int, duration: float) -> dict:
    """Brake response from setSpeed."""
    motor.set_speed(percent)
    await asyncio.sleep(1.5)
    t_cmd = time.monotonic()
    motor.brake()
    t_after = time.monotonic()
    _log(f"  brake() called at {t_cmd:.6f}, returned at {t_after:.6f} "
         f"(call took {(t_after - t_cmd)*1000:.2f} ms)")
    raw = await _record_bemf(motor, duration, t0_override=t_cmd)
    motor.brake()
    f = _ema(raw)
    return {**_stop_analysis(f), "cmd_sent_at": t_cmd, "samples": f}


async def _test_system_rise(robot: "GenericRobot", omega: float,
                            duration: float) -> dict:
    """Step response: chassis wz command → IMU heading rate."""
    robot.drive.hard_stop()
    robot.odometry.reset()
    await asyncio.sleep(0.5)

    t_cmd = time.monotonic()
    robot.drive.set_velocity(ChassisVelocity(0.0, 0.0, omega))
    _log(f"  drive.set_velocity() called at {t_cmd:.6f}")

    samples: list[tuple[float, float]] = []
    prev_heading = 0.0
    t0 = t_cmd
    prev_t = t_cmd

    while time.monotonic() - t0 < duration:
        await asyncio.sleep(0.01)
        now = time.monotonic()
        dt = now - prev_t
        prev_t = now
        if dt < 1e-6:
            continue
        robot.drive.update(dt)
        robot.odometry.update(dt)
        heading = robot.odometry.get_heading()
        samples.append((now - t0, (heading - prev_heading) / dt))
        prev_heading = heading

    robot.drive.hard_stop()
    await asyncio.sleep(0.3)
    f = _ema(samples)
    return {**_rise_analysis(f), "samples": f}


async def _test_system_brake(robot: "GenericRobot", omega: float,
                             duration: float) -> dict:
    """Brake response: chassis wz → stop → IMU heading rate decay."""
    robot.odometry.reset()
    await asyncio.sleep(0.1)

    robot.drive.set_velocity(ChassisVelocity(0.0, 0.0, omega))
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.5:
        robot.drive.update(0.01)
        robot.odometry.update(0.01)
        await asyncio.sleep(0.01)

    samples: list[tuple[float, float]] = []
    prev_heading = robot.odometry.get_heading()
    t0 = time.monotonic()
    prev_t = t0

    robot.drive.hard_stop()

    while time.monotonic() - t0 < duration:
        await asyncio.sleep(0.01)
        now = time.monotonic()
        dt = now - prev_t
        prev_t = now
        if dt < 1e-6:
            continue
        robot.odometry.update(dt)
        heading = robot.odometry.get_heading()
        samples.append((now - t0, (heading - prev_heading) / dt))
        prev_heading = heading

    f = _ema(samples)
    return {**_stop_analysis(f), "samples": f}


# ------------------------------------------------------------------
# Main entry point
# ------------------------------------------------------------------
async def motor_response_test(
    robot: "GenericRobot",
    test_percent: int = 50,
    duration: float = 2.0,
) -> dict:
    """
    Measure motor and system response times.

    Runs step-response and brake tests for each drive motor using both
    setSpeed (direct PWM) and setVelocity (firmware BEMF PID), reading
    back the BEMF value directly. Also tests full-system chassis→IMU
    response.

    Args:
        robot: The robot instance.
        test_percent: Motor power for the speed test (default 50).
        duration: Recording duration per test in seconds (default 2.0).

    Returns:
        Dict with per-motor and system results.
    """
    _log("=" * 60)
    _log("  MOTOR / SYSTEM RESPONSE TIME DIAGNOSTIC")
    _log("=" * 60)

    motors = robot.drive.get_motors()
    if not motors:
        _log("ERROR: No motors found")
        return {}

    _log(f"  Motors: {len(motors)},  test_percent: {test_percent}%,  duration: {duration}s\n")

    all_results: dict = {"motors": [], "system": {}}

    for idx, motor in enumerate(motors):
        _log(f"{'='*60}")
        _log(f"  Motor {idx}  (port {motor.port})")
        _log(f"{'='*60}")

        # A — setSpeed
        _log(f"\n--- [A] setSpeed({test_percent}%) → BEMF ---")
        a = await _test_set_speed(motor, test_percent, duration)
        _print_rise(a)
        await asyncio.sleep(1.0)

        # B — setVelocity (target = steady-state BEMF from test A)
        ss_bemf = a.get("steady_state", 0)
        if abs(ss_bemf) < 1:
            _log("\n--- [B] setVelocity: SKIPPED (no BEMF in test A) ---")
            b = None
        else:
            bemf_target = int(round(ss_bemf))
            _log(f"\n--- [B] setVelocity(BEMF={bemf_target}) → BEMF ---")
            b = await _test_set_velocity(motor, bemf_target, duration)
            _print_rise(b)
        await asyncio.sleep(1.0)

        # C — Brake
        _log(f"\n--- [C] Brake from setSpeed({test_percent}%) → BEMF ---")
        c = await _test_brake(motor, test_percent, duration)
        _print_stop(c)
        await asyncio.sleep(1.0)

        all_results["motors"].append({
            "port": motor.port,
            "set_speed": a,
            "set_velocity": b,
            "brake": c,
        })

    # System tests
    _log(f"\n{'='*60}")
    _log("  Full System  (chassis → IMU)")
    _log(f"{'='*60}")

    _log("\n--- [D] drive.set_velocity(wz=1.0) → IMU heading rate ---")
    d = await _test_system_rise(robot, omega=1.0, duration=duration)
    _print_rise(d)
    all_results["system"]["rise"] = d
    await asyncio.sleep(0.5)

    _log("\n--- [E] System brake from wz=1.0 → IMU heading rate ---")
    e = await _test_system_brake(robot, omega=1.0, duration=duration)
    _print_stop(e)
    all_results["system"]["brake"] = e

    robot.drive.hard_stop()

    _log(f"\n{'='*60}")
    _log("  DONE")
    _log(f"{'='*60}")

    return all_results
