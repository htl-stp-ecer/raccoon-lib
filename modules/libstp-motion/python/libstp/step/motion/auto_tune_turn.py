"""
Auto-tune PID gains for turn motion.

Measures the robot's actual response characteristics (static friction,
dead time, velocity gain) and uses finite-difference gradient descent
on (kP, kD) to minimise a cost function of overshoot, settling time,
oscillation, and final error.
"""
import asyncio
import math
from typing import TYPE_CHECKING

from libstp.foundation import ChassisVelocity
from libstp.motion import TurnMotion, TurnConfig

from .. import Step, dsl

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class AutoTuneTurn(Step):
    """
    Automatically tune PD gains for TurnMotion.

    Phase 1 – Measure static friction (kS):
        Ramp angular command until the robot starts rotating.

    Phase 2 – Measure system response:
        Step command → measure dead time and steady-state velocity gain.

    Phase 3 – Compute initial gains from the measurements.

    Phase 4 – Gradient descent:
        Finite-difference gradient on (kP, kD), minimising a cost
        of overshoot + settling + oscillation + final error.
    """

    def __init__(
        self,
        test_angle_deg: float = 90.0,
        max_rate: float = 1.0,
        gradient_steps: int = 3,
        target_overshoot_deg: float = 2.0,
    ):
        super().__init__()
        self.test_angle_deg = test_angle_deg
        self.max_rate = max_rate
        self.gradient_steps = gradient_steps
        self.target_overshoot_deg = target_overshoot_deg

    def _generate_signature(self) -> str:
        return (
            f"AutoTuneTurn(angle={self.test_angle_deg:.0f}°, "
            f"max_rate={self.max_rate:.1f}, steps={self.gradient_steps})"
        )

    # ------------------------------------------------------------------
    # Main entry
    # ------------------------------------------------------------------
    async def _execute_step(self, robot: "GenericRobot") -> None:
        self.info("=" * 55)
        self.info("  AUTO-TUNE TURN PD")
        self.info("=" * 55)

        # Phase 1
        self.info("\n--- Phase 1: Measuring static friction ---")
        ks = await self._measure_static_friction(robot)
        self.info(f"  → kS = {ks:.3f} rad/s")

        # Phase 2
        self.info("\n--- Phase 2: Measuring system response ---")
        dead_time, vel_gain = await self._measure_response(robot, ks)
        self.info(f"  → dead_time = {dead_time:.3f} s")
        self.info(f"  → velocity_gain = {vel_gain:.2f}")

        # Phase 3
        self.info("\n--- Phase 3: Computing initial gains ---")
        kP, kD = self._compute_initial_gains(dead_time, vel_gain)
        self.info(f"  → kP={kP:.3f}  kD={kD:.3f}")

        # Phase 4
        self.info("\n--- Phase 4: Gradient descent ---")
        best = await self._gradient_descent(robot, kP, kD, ks)

        # Report
        self.info("\n" + "=" * 55)
        self.info("  AUTO-TUNE RESULTS")
        self.info("=" * 55)
        self.info(f"  heading_kp = {best['kP']:.3f}")
        self.info(f"  heading_kd = {best['kD']:.3f}")
        self.info(f"  kS         = {best['kS']:.3f}")
        self.info(f"  max_rate   = {self.max_rate:.3f}")
        self.info(f"  ---")
        self.info(f"  Overshoot  = {best['overshoot_deg']:.1f}°")
        self.info(f"  Settle     = {best['settling_time']:.2f} s")
        self.info(f"  Oscillations = {best['oscillations']}")
        self.info(f"  Final err  = {best['final_error_deg']:.1f}°")
        self.info(f"  Cost       = {best['cost']:.1f}")
        self.info("=" * 55)

        # Apply to the live config so subsequent turns use these gains
        robot.motion_pid_config.heading_kp = best["kP"]
        robot.motion_pid_config.heading_ki = 0.0
        robot.motion_pid_config.heading_kd = best["kD"]
        self.info("  (gains applied to robot.motion_pid_config)")

    # ------------------------------------------------------------------
    # Phase 1 – static friction
    # ------------------------------------------------------------------
    async def _measure_static_friction(self, robot: "GenericRobot") -> float:
        rate = 1 / 50
        breakaway_cmd = 0.0

        for pct in range(5, 105, 5):  # 5 % → 100 % of max_rate
            cmd = (pct / 100.0) * self.max_rate

            robot.odometry.reset()
            await asyncio.sleep(0.05)

            # Hold this command for 0.25 s
            t0 = asyncio.get_event_loop().time()
            last = t0 - rate
            while asyncio.get_event_loop().time() - t0 < 0.25:
                now = asyncio.get_event_loop().time()
                dt = max(now - last, 0.0)
                last = now
                if dt < 1e-4:
                    await asyncio.sleep(rate)
                    continue
                robot.drive.set_velocity(ChassisVelocity(0.0, 0.0, cmd))
                robot.drive.update(dt)
                robot.odometry.update(dt)
                await asyncio.sleep(rate)

            robot.drive.hard_stop()

            heading = abs(robot.odometry.get_heading())
            self.info(f"    cmd={cmd:.2f}  heading={math.degrees(heading):.1f}°")

            if heading > 0.02:  # moved > ~1°
                breakaway_cmd = cmd
                break

            await asyncio.sleep(0.15)

        robot.drive.hard_stop()
        await asyncio.sleep(0.5)
        return breakaway_cmd * 0.7  # 70 % of breakaway

    # ------------------------------------------------------------------
    # Phase 2 – step response
    # ------------------------------------------------------------------
    async def _measure_response(
        self, robot: "GenericRobot", ks: float
    ) -> tuple[float, float]:
        cmd = min(0.5 * self.max_rate + ks, self.max_rate)
        rate = 1 / 100

        robot.odometry.reset()
        await asyncio.sleep(0.1)

        prev_heading = 0.0
        records: list[tuple[float, float, float]] = []  # (t, heading, vel)

        t0 = asyncio.get_event_loop().time()
        last = t0 - rate

        while asyncio.get_event_loop().time() - t0 < 2.0:
            now = asyncio.get_event_loop().time()
            dt = max(now - last, 0.0)
            last = now
            if dt < 1e-4:
                await asyncio.sleep(rate)
                continue
            robot.drive.set_velocity(ChassisVelocity(0.0, 0.0, cmd))
            robot.drive.update(dt)
            robot.odometry.update(dt)

            heading = robot.odometry.get_heading()
            vel = (heading - prev_heading) / dt
            prev_heading = heading
            records.append((now - t0, heading, vel))
            await asyncio.sleep(rate)

        robot.drive.hard_stop()
        await asyncio.sleep(0.5)

        # Dead time: first sample where heading exceeds 0.5°
        dead_time = 0.0
        for t, h, _ in records:
            if abs(h) > math.radians(0.5):
                dead_time = t
                break

        # Steady-state velocity (last 0.4 s)
        steady = [v for t, _, v in records if t > 1.6]
        if steady:
            ss_vel = sum(steady) / len(steady)
            vel_gain = ss_vel / cmd if cmd > 1e-6 else 1.0
        else:
            vel_gain = 1.0

        return dead_time, vel_gain

    # ------------------------------------------------------------------
    # Phase 3 – initial gains
    # ------------------------------------------------------------------
    def _compute_initial_gains(
        self, dead_time: float, vel_gain: float
    ) -> tuple[float, float]:
        lag = max(dead_time, 0.05)

        # Deceleration zone must be large enough so the robot can react
        # and brake in time.  stopping_distance ≈ lag × max_rate × gain.
        stop_dist = lag * self.max_rate * vel_gain
        decel_zone = max(stop_dist * 2.5, 0.25)

        kP = self.max_rate / decel_zone
        kP = max(0.5, min(kP, 5.0))

        # D ≈ lag × kP provides anticipatory braking
        kD = lag * kP * 0.8
        kD = max(0.05, min(kD, 3.0))

        return round(kP, 3), round(kD, 3)

    # ------------------------------------------------------------------
    # Phase 4 – gradient descent
    # ------------------------------------------------------------------
    async def _gradient_descent(
        self, robot: "GenericRobot", kP: float, kD: float, kS: float
    ) -> dict:
        best_result: dict | None = None
        best_cost = float("inf")

        perturbation = 0.20  # 20 % relative perturbation
        lr = 0.12  # initial learning rate (max relative step fraction)

        for step in range(self.gradient_steps):
            # Same direction for all 3 probes per step; alternate between steps
            angle = self.test_angle_deg if step % 2 == 0 else -self.test_angle_deg
            direction = "CCW" if angle > 0 else "CW"

            self.info(
                f"\n  Step {step + 1}/{self.gradient_steps} ({direction}):  "
                f"kP={kP:.3f}  kD={kD:.3f}"
            )

            # --- Baseline ---
            base_res = await self._do_test_turn(robot, kP, kD, kS, angle)
            c_base = self._cost(base_res)
            self._log_result("base", base_res, c_base)

            if c_base < best_cost:
                best_cost = c_base
                best_result = {"kP": kP, "kD": kD, "kS": kS, **base_res, "cost": c_base}

            # --- Probe kP ---
            dkP = kP * perturbation
            res_p = await self._do_test_turn(robot, kP + dkP, kD, kS, angle)
            c_p = self._cost(res_p)
            self._log_result(f"kP+{dkP:.3f}", res_p, c_p)

            if c_p < best_cost:
                best_cost = c_p
                best_result = {"kP": kP + dkP, "kD": kD, "kS": kS, **res_p, "cost": c_p}

            # --- Probe kD ---
            dkD = kD * perturbation
            res_d = await self._do_test_turn(robot, kP, kD + dkD, kS, angle)
            c_d = self._cost(res_d)
            self._log_result(f"kD+{dkD:.3f}", res_d, c_d)

            if c_d < best_cost:
                best_cost = c_d
                best_result = {"kP": kP, "kD": kD + dkD, "kS": kS, **res_d, "cost": c_d}

            # --- Gradient (relative: cost change per 100% parameter change) ---
            rg_kP = (c_p - c_base) / perturbation
            rg_kD = (c_d - c_base) / perturbation

            max_rg = max(abs(rg_kP), abs(rg_kD), 1e-6)

            # Fallback: if everything timed out, just halve gains
            if base_res["timed_out"] and res_p["timed_out"] and res_d["timed_out"]:
                kP *= 0.5
                kD *= 0.5
                self.info("    ↳ all timed out → halving gains")
            else:
                # Multiplicative step, normalised so max change = lr
                kP *= 1.0 - lr * rg_kP / max_rg
                kD *= 1.0 - lr * rg_kD / max_rg
                self.info(
                    f"    gradient: ∂kP={rg_kP:+.1f}  ∂kD={rg_kD:+.1f}  "
                    f"→ kP={kP:.3f}  kD={kD:.3f}"
                )

            kP = max(0.3, min(kP, 6.0))
            kD = max(0.02, min(kD, 4.0))

            lr *= 0.7  # decay learning rate

        return best_result  # type: ignore[return-value]

    # ------------------------------------------------------------------
    # Cost function
    # ------------------------------------------------------------------
    def _cost(self, result: dict) -> float:
        cost = (
            result["overshoot_deg"] * 10.0
            + result["settling_time"]
            + result["final_error_deg"] * 5.0
            + result["oscillations"] * 8.0
        )
        if result["timed_out"]:
            cost += 100.0
        return cost

    # ------------------------------------------------------------------
    # Log helper
    # ------------------------------------------------------------------
    def _log_result(self, label: str, res: dict, cost: float) -> None:
        self.info(
            f"    [{label}] ov={res['overshoot_deg']:.1f}°  "
            f"settle={res['settling_time']:.2f}s  "
            f"osc={res['oscillations']}  "
            f"err={res['final_error_deg']:.1f}°  "
            f"cost={cost:.1f}"
            f"{'  TIMEOUT' if res['timed_out'] else ''}"
        )

    # ------------------------------------------------------------------
    # Single test turn
    # ------------------------------------------------------------------
    async def _do_test_turn(
        self,
        robot: "GenericRobot",
        kP: float,
        kD: float,
        kS: float,
        angle_deg: float,
    ) -> dict:
        robot.motion_pid_config.heading_kp = kP
        robot.motion_pid_config.heading_ki = 0.0
        robot.motion_pid_config.heading_kd = kD

        config = TurnConfig()
        config.target_angle_rad = math.radians(angle_deg)
        config.max_angular_rate = self.max_rate
        config.kS = kS

        motion = TurnMotion(
            robot.drive, robot.odometry, robot.motion_pid_config, config
        )
        motion.start()

        target_rad = math.radians(angle_deg)
        sign = 1.0 if angle_deg > 0 else -1.0

        rate = 1 / 50
        headings: list[tuple[float, float]] = []
        max_overshoot = 0.0

        t0 = asyncio.get_event_loop().time()
        last = t0 - rate
        timeout = 8.0

        while not motion.is_finished():
            now = asyncio.get_event_loop().time()
            if now - t0 > timeout:
                break
            dt = max(now - last, 0.0)
            last = now
            if dt < 1e-4:
                await asyncio.sleep(rate)
                continue
            motion.update(dt)
            heading = robot.odometry.get_heading()
            headings.append((now - t0, heading))

            # Overshoot = how far past target in the turn direction
            overshoot = (heading - target_rad) * sign
            max_overshoot = max(max_overshoot, overshoot)
            await asyncio.sleep(rate)

        robot.drive.hard_stop()

        # Final error
        robot.odometry.update(0.01)
        final_error = abs(robot.odometry.get_heading_error(target_rad))

        # Settling time: last time we were outside 2° of target
        tol = math.radians(2.0)
        settling_time = 0.0
        for t, h in reversed(headings):
            if abs(h - target_rad) > tol:
                settling_time = t
                break

        # Oscillations: count zero crossings of error past target
        oscillations = 0
        prev_err_sign: int | None = None
        for _, h in headings:
            err = h - target_rad
            s = 1 if err >= 0 else -1
            if prev_err_sign is not None and s != prev_err_sign:
                oscillations += 1
            prev_err_sign = s

        total = headings[-1][0] if headings else 0.0

        await asyncio.sleep(1.0)  # settle between turns

        return {
            "overshoot_rad": max(max_overshoot, 0.0),
            "overshoot_deg": math.degrees(max(max_overshoot, 0.0)),
            "settling_time": settling_time,
            "final_error_deg": math.degrees(final_error),
            "oscillations": oscillations,
            "total_time": total,
            "timed_out": total >= timeout - 0.1,
        }


@dsl(tags=["motion", "calibration", "turn"])
def auto_tune_turn(
    test_angle_deg: float = 90.0,
    max_rate: float = 1.0,
    gradient_steps: int = 3,
    target_overshoot_deg: float = 2.0,
) -> AutoTuneTurn:
    """
    Automatically tune PD gains for turn motion.

    Measures static friction and system response, then performs gradient
    descent on (kP, kD) to converge on optimal gains.  Results are
    applied to ``robot.motion_pid_config`` so subsequent turns use
    the tuned values.

    Each gradient step performs 3 test turns (baseline + kP probe + kD probe),
    so total turns = gradient_steps × 3.

    Args:
        test_angle_deg: Angle for test turns (default 90)
        max_rate: Maximum angular rate in rad/s (default 1.0)
        gradient_steps: Number of gradient descent steps (default 3, = 9 turns)
        target_overshoot_deg: Acceptable overshoot in degrees (default 2.0)

    Returns:
        Step that auto-tunes and applies optimal turn PD gains
    """
    return AutoTuneTurn(
        test_angle_deg=test_angle_deg,
        max_rate=max_rate,
        gradient_steps=gradient_steps,
        target_overshoot_deg=target_overshoot_deg,
    )
