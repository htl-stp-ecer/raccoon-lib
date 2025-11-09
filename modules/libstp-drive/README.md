# libstp-drive

**Purpose:** Convert chassis-space velocity setpoints into coordinated per-wheel velocity commands with per-wheel safety
and velocity control.  
**Non-goal:** Motion planning (drive-straight, turn-to-angle, path following). Keep that in your Motion layer.

## API (essentials)

- `setVelocity(ChassisVel v)` — set desired body velocities (from Motion).
- `update(double dt)` — runs per-wheel ω/ω̇ limiting and wheel velocity controllers; writes to HAL; returns `Achieved` (
  achieved body velocity + saturation info).
- `estimateState()` — estimate body state via encoders (`kinematics::IKinematics::forward`).
- `setChassisLimits`, `setWheelLimits`, `setWheelControllerGains` — safety & tuning.
- `stop(bool hard=false)` — command stop; `hard` zeros immediately.

## Control loop (example @100 Hz)

```cpp
drive.setChassisLimits({.max_v=1.5, .max_omega=6.0});
drive.setWheelLimits({.max_w=80.0, .max_w_dot=800.0});
drive.setWheelControllerGains({.kp=0.6, .ki=2.0, .kd=0.0, .ff=1.0});

for (;;) {
  foundation::ChassisCmd cmd = motion.updateAndGetCmd(dt); // your Motion
  drive.setVelocity({cmd.vx, cmd.vy, cmd.w});
  libstp::drive::Achieved a = drive.update(dt);
  // if (a.saturated_any) motion.derate(...)
}
```

### Integration notes

- Start with `ff=1.0`, small `kp`, zero `ki/kd`. Add `ki` only if you see steady-state errors.
- Motor speed ramps are disabled by default to simplify bring-up. Export `LIBSTP_ENABLE_SPEED_RAMPS=1` if you need the ω̇ limiter active.
