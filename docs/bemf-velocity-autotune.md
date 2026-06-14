# BEMF → velocity auto-tune (`auto_tune_bemf_velocity`)

Fully-automatic calibration of each drive motor's **`ticks_to_rad`** (the
BEMF-tick → wheel-angle scale that the STM32 odometry integrates) against an
external **calibration board** as ground truth. It also *characterises* the
ADC-BEMF↔velocity relationship instead of assuming it is linear, and writes a
full data + plot report you can inspect.

## Why

The on-board odometry dead-reckons from back-EMF: the STM32 samples BEMF,
integrates it into "ticks" (`MotorData.position`), and converts ticks to wheel
angle with a per-motor `ticks_to_rad`. If that scale is wrong the robot
mis-estimates how far it drove. The cheap on-board estimate has no independent
reference, so we mount a **calibration board** (separate STM32 + PMW/PAA
optical-flow + ICM IMU) that publishes an accurate fused pose; the wombat
odometry auto-switches to it when present (`OdometrySource::CalibrationBoard`).
This tuner uses that accurate pose to measure the true `ticks_to_rad`.

## How it works

For a sweep of open-loop PWM levels (back-and-forth, so it stays within ~1 m of
its start), at each level it:

1. **pre-rolls** a short distance so the chassis reaches steady-state speed,
2. opens a **measurement window** over a fixed ground-truth distance `D`
   (read from the calibration board), recording per motor the accumulated BEMF
   ticks `Δticks` and the median instantaneous `|BEMF|`,
3. derives, for that speed, `ticks_to_rad = (D / wheel_radius) / Δticks`
   (forward drive ⇒ every wheel turns `D/r` radians).

Open-loop PWM is used on purpose: closed-loop velocity control would depend on
the very calibration we are measuring.

### Linearity is measured, not assumed

Two independent diagnostics per motor:

- **CV of `ticks_to_rad` across speeds** — if a single scale really describes
  tick↔angle, this is ~constant (low coefficient of variation).
- **ω-vs-BEMF ordinary-least-squares fit** (`ω = a·bemf + b`, with R²) — a
  near-zero intercept `b` means BEMF is *proportional* to speed; a non-zero
  intercept exposes a **standstill BEMF offset**.

A motor is flagged `LINEAR` only when CV < 12 %, R² > 0.95 and the intercept is
small relative to the top speed. If the overall fit is not linear the step logs
a warning rather than silently persisting a misleading single scale.

## Usage

Prerequisites: the calibration board connected and backing the odometry pose
(`robot.odometry.get_active_source() == CALIBRATION_BOARD`), and ~1 m of clear
runway forward/back.

```python
from raccoon.step.motion import auto_tune_bemf_velocity

# in a (setup) mission sequence:
auto_tune_bemf_velocity()                       # defaults: PWM 30–90 %, 6 steps, persist=True
auto_tune_bemf_velocity(pwm_min_percent=20,
                        pwm_max_percent=85,
                        pwm_steps=9)             # thorough sweep incl. low speed
```

With `persist=True` the measured per-motor `ticks_to_rad` is written to
`definitions.<motor>.calibration.ticks_to_rad` in `raccoon.project.yml`, and the
live kinematics config is republished to the STM32.

## Report output

Every run writes a timestamped folder under the project:

```
.raccoon/auto_tune/bemf_velocity/<YYYYMMDD_HHMMSS>/
├── data.csv        # one row per speed point: pwm, distance, time, speed, ω, valid,
│                   # and per-motor Δticks / median|BEMF| / ticks_to_rad
├── summary.json    # per-motor fit (ticks_to_rad, CV, slope, intercept, R², linear) + chosen values
├── findings.md     # human-readable table + interpretation (auto-generated)
├── plot.py         # standalone; regenerates the PNGs from data.csv + summary.json
├── omega_vs_bemf.png          # BEMF → wheel speed, with linear fit + R² per motor
├── ticks_to_rad_vs_omega.png  # per-speed ticks_to_rad (flat ⇒ a single scale holds)
├── residuals_vs_omega.png     # ω-vs-BEMF fit residuals (trend ⇒ nonlinearity / offset)
└── omega_vs_pwm.png           # open-loop speed vs PWM (sanity)
```

PNG plots are rendered inline when **matplotlib** is importable (the optional
`plots` extra: `pip install 'raccoon-library[plots]'`). When it is not
installed, the data + `plot.py` are still written, and you can render the plots
anywhere matplotlib is available with `python plot.py`.

## Findings (PackingBot, 2026-06-14)

Thorough sweep, PWM 20–85 % (9 valid points, wheel speed ω ≈ 0.8–4.4 rad/s),
mecanum drive, `wheel_radius = 0.0375 m`:

| port | motor | ticks_to_rad (median) | CV | ω = a·bemf + b | R² | verdict |
|---|---|---|---|---|---|---|
| 0 | front_right | 1.7374e-5 | 2.1 % | 0.00311·bemf − 0.036 | 0.9991 | LINEAR |
| 1 | front_left  | 1.7730e-5 | 2.4 % | 0.00323·bemf − 0.068 | 0.9992 | LINEAR |
| 2 | rear_left   | 1.7326e-5 | 2.4 % | 0.00314·bemf − 0.061 | 0.9989 | LINEAR |
| 3 | rear_right  | 1.7819e-5 | 1.7 % | 0.00311·bemf + 0.037 | 0.9992 | LINEAR |

**Takeaways:**

- Over the usable speed range the BEMF↔velocity relation is **highly linear**
  (R² ≈ 0.999, CV ≈ 2 %). A single per-motor `ticks_to_rad` is justified there.
- There is a **per-motor spread of ~4 %** (ports 1 & 3 vs 0 & 2). The previous
  IMU-rotation calibration applied one global scale and could not capture this;
  this tuner does it per motor.
- The ω-vs-BEMF intercept is **small but non-zero** (≈ −0.1…+0.04 rad/s) ⇒ a
  ~20–35-count **BEMF offset at standstill**. The firmware (`bemf.c`) integrates
  BEMF into ticks with **no dead-zone / no offset subtraction**, so it counts
  phantom ticks while (nearly) stopped.
- The old single global scale (`1.98e-5`) over-reported distance by ~12–18 %.
  The new per-motor scales cut a verification 30 cm drive's error from ~18 % to
  **~6 %**. The residual ~6 % is dominated by the standstill offset above (it
  accumulates during acceleration / at low speed) — a single linear scale cannot
  remove it.

**Next (firmware level):** subtract the per-motor BEMF zero-offset and/or add a
dead-zone in `bemf.c`, and revisit the `positionAccum += bemf` integration
(no `dt`) and the one-motor-per-cycle round-robin off-time sampling. That is the
"lowest-level accurate" fix that the linear scale cannot address.

## Related

- C++: `modules/libstp-autotune/{include/autotune,src}/bemf_velocity_tune.*`
- Report writer: `modules/libstp-motion/python/raccoon/step/motion/_bemf_report.py`
- Manual ground-truth tooling: `tools/odometry-calibration/`
