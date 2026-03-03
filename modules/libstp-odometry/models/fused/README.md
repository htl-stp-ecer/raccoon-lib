# libstp-odometry fused model

This submodule implements `libstp::odometry::IOdometry` by combining IMU heading with velocity estimates from a `kinematics::IKinematics` instance.

## Purpose

- Track a planar pose using IMU heading and chassis velocity integration.
- Optionally blend encoder/BEMF-derived velocity with IMU-integrated velocity through a simple complementary filter.
- Report displacement relative to the last reset pose.

## Math model

### Heading

On the first valid update after construction or reset, the model captures:

- `initial_imu_heading = imu->getHeading()`

Current heading is then:

```text
imu_heading_delta = wrapAngle(last_imu_heading - initial_imu_heading)
heading = wrapAngle(origin_heading + imu_heading_delta)
```

This makes the odometry heading relative to the last reset origin instead of absolute IMU heading.

### Velocity fusion

The kinematics model supplies a body-frame velocity estimate:

```text
v_body = (vx, vy)
```

That vector is rotated into world frame using the current heading:

```text
vx_world = cos(heading) * vx - sin(heading) * vy
vy_world = sin(heading) * vx + cos(heading) * vy
```

If `enable_accel_fusion` is enabled and at least one update has already run, the world-frame velocity is blended with `imu->getIntegratedVelocity()`:

```text
v_fused = alpha * v_bemf_world + (1 - alpha) * v_accel_world
alpha = clamp(bemf_trust, 0, 1)
```

Position is then integrated with forward Euler:

```text
position += v_fused * dt
```

### Distance from origin

`getDistanceFromOrigin()` projects the world-frame displacement from the last reset pose onto the origin's forward and right directions, then also reports Euclidean norm.

## Public C++ surface

Header: `include/odometry/fused/fused_odometry.hpp`

- `FusedOdometryConfig`
  - `imu_ready_timeout_ms`
  - `enable_accel_fusion`
  - `bemf_trust`
  - `turn_axis`
- `FusedOdometry`
  Constructor dependencies:
  - `std::shared_ptr<hal::imu::IIMU>`
  - `std::shared_ptr<kinematics::IKinematics>`

Behavior that matters to contributors:

- The constructor and zero-argument `reset()` call `imu->waitForReady(...)` and `imu->setYawRateAxisMode(turn_axis)`.
- `reset()` also calls `kinematics->resetEncoders()` before clearing origin state.

## Bindings

Binding module: `libstp.odometry_fused`

Exposed Python types:

- `DistanceFromOrigin`
- `FusedOdometryConfig`
- `FusedOdometry`

The Python binding exposes the full fused-model API, including `get_distance_from_origin`, `get_heading`, and `get_heading_error`, which are not surfaced by the base `libstp.odometry` binding.

## Tests

No tests live under this submodule today.

## Extension points

- Add alternative fusion strategies by extending this model or by introducing a sibling model under `models/`.
- If you add external pose corrections, keep the origin-relative semantics of `getHeading()` and `getDistanceFromOrigin()` explicit in the API docs.
- Any change to `turn_axis` handling must stay coordinated with the IMU firmware and HAL expectations.
