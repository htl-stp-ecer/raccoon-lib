"""Warm-start standalone motion steps from the robot's measured Ist-velocity.

A standalone motion step normally cold-starts its profile at zero velocity via
``motion.start()``. When the robot is already moving at ``on_start`` — e.g. the
previous step handed off with residual velocity — the trapezoidal profile should
ramp *from that measured velocity*, not snap its setpoint to zero. That is the
whole point: a leg that begins at -0.22 m/s must run its ramp from -0.22, not 0.

These helpers read the measured body velocity from ``robot.drive.estimate_state()``
(a :class:`ChassisVelocity` ``{vx, vy, wz}`` derived from the encoders/gyro) and
pick the component the motion regulates, then warm-start the C++ motion with it.

Below a small deadband the reading is treated as rest and the caller cold-starts,
so a stopped robot behaves *exactly* as before — encoder noise never seeds a
phantom entry velocity, and the common sequential-steps case (previous leg ramped
to zero) is byte-for-byte unchanged.

Only motions that expose ``start_warm`` (LinearMotion / TurnMotion / ArcMotion)
can use this. DiagonalMotion, SplineMotion and the line-follow adapters have no
warm entry point and still cold-start.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from raccoon.motion import LinearAxis

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

# Below these magnitudes the robot is treated as at rest, so the motion
# cold-starts and behaves identically to the pre-warm-start code path.
WARM_START_MIN_MPS = 0.01
WARM_START_MIN_RADPS = 0.05


def measured_linear_velocity(robot: "GenericRobot", axis: "LinearAxis") -> float:
    """Measured body velocity along ``axis`` (m/s): vx for Forward, vy for Lateral."""
    state = robot.drive.estimate_state()
    return float(state.vx) if axis == LinearAxis.Forward else float(state.vy)


def measured_angular_velocity(robot: "GenericRobot") -> float:
    """Measured body yaw rate wz (rad/s)."""
    return float(robot.drive.estimate_state().wz)


def warm_start_linear(motion, robot: "GenericRobot", axis: "LinearAxis") -> None:
    """Start ``motion`` ramping from the measured velocity along ``axis``.

    Cold-starts (unchanged behaviour) when the robot is at rest within the
    deadband; otherwise warm-starts so the profile's ramp begins at the current
    velocity instead of zero.
    """
    v0 = measured_linear_velocity(robot, axis)
    if abs(v0) >= WARM_START_MIN_MPS:
        motion.start_warm(0.0, v0)
    else:
        motion.start()


def warm_start_angular(motion, robot: "GenericRobot") -> None:
    """Start ``motion`` ramping from the measured yaw rate wz.

    Cold-starts (unchanged behaviour) when the robot is not turning within the
    deadband; otherwise warm-starts from the current angular velocity.
    """
    w0 = measured_angular_velocity(robot)
    if abs(w0) >= WARM_START_MIN_RADPS:
        motion.start_warm(0.0, w0)
    else:
        motion.start()
