"""ArmPreset — named arm positions with pre-solved servo angles.

Named positions are resolved at codegen time (on the dev machine) via the
raccoon-cli ArmChain generator. At runtime the Wombat only performs a dict
lookup and issues existing servo steps — no IK library required.

Instances are generated from ``raccoon.project.yml`` into ``defs.py``::

    definitions:
      arm:
        type: ArmChain
        joints:
          - servo: shoulder_servo
            length_cm: 12.5
            joint_range_deg: [0, 90]
            servo_range_deg: [10, 130]
        positions:
          grab_cone: {x: 18, y: 0, z: 5}

Generated usage::

    from src.hardware.defs import Defs

    Defs.arm.grab_cone()  # instant — all joints move in parallel
    Defs.arm.grab_cone(speed=60)  # eased at 60 deg/s per joint
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from raccoon.hal import Servo

if TYPE_CHECKING:
    from raccoon.step import Step


class _ArmPosition:
    """Callable that returns a Step moving all arm joints simultaneously.

    Analogous to ``_PresetPosition`` in ``ServoPreset``, but for multiple
    joints at once. All joints move in parallel (``parallel()`` step).
    """

    __slots__ = ("_joints", "_angles", "__name__")

    def __init__(self, joints: list[Servo], angles_deg: list[float], name: str) -> None:
        self._joints = joints
        self._angles = list(angles_deg)
        self.__name__ = name

    @property
    def values(self) -> list[float]:
        """Pre-solved servo angles in degrees (one per joint)."""
        return list(self._angles)

    def __call__(self, speed: float | None = None) -> "Step":
        """Return a Step that moves all joints to this position.

        Args:
            speed: Optional speed in deg/s. Without speed, joints jump
                instantly via ``servo()``. With speed, each joint uses
                ``SlowServo`` with EASE_IN_OUT interpolation.
        """
        from raccoon.step.parallel import parallel as _parallel
        from raccoon.step.servo.steps import SlowServo
        from raccoon.step.servo.steps import servo as _servo

        moves = [
            _servo(joint, angle) if speed is None else SlowServo(joint, angle, speed=speed)
            for joint, angle in zip(self._joints, self._angles, strict=False)
        ]
        if len(moves) == 1:
            return moves[0]
        return _parallel(*moves)

    def __repr__(self) -> str:
        return f"<ArmPosition {self.__name__!r} angles={self._angles}>"


class ArmPreset:
    """Named arm positions backed by pre-solved servo angles.

    Each position becomes a callable attribute that returns a Step.
    All joints move simultaneously via ``parallel()``.

    Calling without ``speed`` issues instant ``servo()`` commands.
    Calling with ``speed`` uses ``SlowServo`` on every joint.

    Args:
        joints: Servo hardware devices in kinematic order (base → tip).
        positions: Mapping of position name to pre-solved servo angles
            in degrees (one angle per joint, same order as ``joints``).
    """

    def __init__(self, joints: list[Servo], positions: dict[str, list[float]]) -> None:
        self._joints = list(joints)
        self._positions = dict(positions)
        for name, angles in positions.items():
            setattr(self, name, _ArmPosition(self._joints, angles, name))

    @property
    def joints(self) -> list[Servo]:
        """The underlying servo hardware devices in kinematic order."""
        return list(self._joints)

    def to(
        self,
        x: float,
        y: float,
        z: float = 0.0,
        speed: float | None = None,
    ) -> "Step":
        """Move the arm to a Cartesian position via runtime IK.

        Requires ``ikpy`` to be installed (``pip install ikpy``).
        For competition use, prefer named positions — they are pre-solved
        at codegen time and carry zero runtime overhead.

        Args:
            x: Target x in cm (forward from the first joint origin).
            y: Target y in cm (lateral).
            z: Target z in cm (vertical). Defaults to 0.
            speed: Optional speed in deg/s for eased motion.
        """
        try:
            import ikpy.chain  # noqa: F401
        except ImportError:
            msg = (
                "arm.to() requires ikpy for runtime IK.\n"
                "Install it with: pip install ikpy\n"
                "For competition use, prefer named positions (pre-solved at codegen time)."
            )
            raise ImportError(msg) from None
        msg = "Runtime IK via arm.to() is not yet implemented."
        raise NotImplementedError(msg)
