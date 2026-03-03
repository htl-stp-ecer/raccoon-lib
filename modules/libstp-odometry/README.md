# libstp-odometry

`libstp-odometry` defines the pose-estimation interface used by higher-level motion code and hosts odometry models under [`models/`](./models). The top-level module is intentionally small: it provides the interface and angle helpers, while the concrete implementation in this tree lives in [`models/fused`](./models/fused/README.md).

## Purpose

- Define a stable `IOdometry` contract for pose estimation modules.
- Provide reusable planar-angle helpers for heading normalization and error computation.
- Group concrete odometry models under one module tree without forcing a single implementation.

## Math model

At the top level, this module does not implement a pose estimator. It provides:

- `wrapAngle(angle)` to normalize to `[-pi, pi]`
- `angularError(current, target)` to compute the shortest signed heading error
- `extractHeading(quaternion)` to project a 3D orientation onto the world XY plane without Euler-angle decomposition

The actual position and heading integration logic is implemented by model submodules such as [`models/fused`](./models/fused/README.md).

## Public C++ surface

Headers:

- `include/odometry/odometry.hpp`
- `include/odometry/angle_utils.hpp`

Types and roles:

- `DistanceFromOrigin`
  Forward, lateral, and straight-line displacement relative to the most recent reset.
- `IOdometry`
  Abstract interface with `update`, `getPose`, `getDistanceFromOrigin`, `getHeading`, `getHeadingError`, and `reset()`.
- Angle helpers
  Header-only utility functions for heading wrapping and extraction.

Implementation note:

- `src/dummy.cpp` exists only so the top-level library is not empty; the meaningful behavior in this module tree is currently in the model subdirectory.

## Bindings

Binding module: `libstp.odometry`

Exposed Python type:

- `IOdometry`

The base binding exposes `update`, `get_pose`, and `reset()`. Model-specific bindings export additional behavior.

## Tests

No tests live in this module directory today.

## Extension points

- Add a new odometry model by implementing `IOdometry` under `models/<name>/`.
- Reuse `angle_utils.hpp` for consistent wraparound behavior across models.
- Keep the planar frame conventions aligned with `foundation::Pose` and `foundation::ChassisVelocity`: heading `0` along world `+X`, positive heading counter-clockwise.
