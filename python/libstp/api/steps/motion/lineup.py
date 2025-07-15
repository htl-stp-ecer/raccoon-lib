from typing import Any

from libstp import PIDController
from libstp.datatypes import ConditionalResult, Speed, while_false
from libstp.device import NativeDevice
from libstp.sensor import LightSensor
from libstp_helpers.api.steps import seq
from libstp_helpers.api.steps.drive import Drive
from libstp_helpers.api.steps.motion.drive_until import drive_until_white, drive_until_black
from libstp_helpers.api.steps.sequential import Sequential


class LineUp(Drive):
    """
    Drives forward (or backward) until both sensors exceed a confidence threshold,
    and *scales* its speed down as you approach the line for precision.
    """

    def __init__(
            self,
            left_sensor_name: str,
            right_sensor_name: str,
            base_speed: float = 0.5,
            confidence_threshold: float = 0.9,
            reverse: bool = False,
            kp: float = 1.0,
            ki: float = 0.0,
            kd: float = 0.0,
    ):
        for n in (left_sensor_name, right_sensor_name):
            if not isinstance(n, str):
                raise TypeError(f"Sensor name must be str, got {type(n)}")
        for v in (base_speed, confidence_threshold, kp, ki, kd):
            if not isinstance(v, (int, float)):
                raise TypeError(f"Numeric param must be int/float, got {type(v)}")
        if not 0 < confidence_threshold < 1:
            raise ValueError("confidence_threshold must be between 0 and 1")

        self.left_name = left_sensor_name
        self.right_name = right_sensor_name
        self.base_speed = base_speed
        self.thresh = confidence_threshold
        self.reverse = reverse

        self.left_sensor: LightSensor | None = None
        self.right_sensor: LightSensor | None = None

        # Initialize PID controllers for each wheel
        self.left_pid = PIDController(kp, ki, kd)
        self.right_pid = PIDController(kp, ki, kd)

        def done() -> bool:
            if self.left_sensor is None or self.right_sensor is None:
                return False
            lb = self.left_sensor.get_black_confidence()
            rb = self.right_sensor.get_black_confidence()
            if self.reverse:
                lb, rb = 1 - lb, 1 - rb
            return lb >= self.thresh and rb >= self.thresh

        def speed_fn(_: ConditionalResult) -> Speed:
            lb = self.left_sensor.get_black_confidence()
            rb = self.right_sensor.get_black_confidence()
            if self.reverse:
                lb, rb = 1 - lb, 1 - rb

            if lb >= self.thresh and rb >= self.thresh:
                return Speed(0, 0, 0)

            # Use PID controllers to calculate wheel speeds
            left_output = self.left_pid.calculate(0.5 - lb)
            right_output = self.right_pid.calculate(0.5 - rb)

            # Calculate final wheel speeds by applying PID output to base speed
            left_speed = left_output * self.base_speed
            right_speed = right_output * self.base_speed

            return Speed.wheels(left_speed, right_speed)

        super().__init__(while_false(done), speed_fn, do_correction=False)

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        if not hasattr(definitions, self.left_name) or not hasattr(definitions, self.right_name):
            raise RuntimeError("Sensor names not found in definitions")

        self.left_sensor = getattr(definitions, self.left_name)
        self.right_sensor = getattr(definitions, self.right_name)
        if not isinstance(self.left_sensor, LightSensor) or not isinstance(self.right_sensor, LightSensor):
            raise TypeError("Both sensors must be LightSensor instances")

        await super().run_step(device, definitions)


def lineup(
        left_sensor_name: str = "left_front_sensor",
        right_sensor_name: str = "right_front_sensor",
        base_speed: float = 0.6,
        confidence_threshold: float = 0.6,
        reverse: bool = False,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
) -> LineUp:
    """
    • base_speed: positive = forward, negative = backward (or use reverse=True)
    • kp, ki, kd: PID controller parameters for wheel speed adjustment
    • confidence_threshold: stop when each sensor's target-confidence ≥ thresh
    • reverse=True: lineup on white instead of black
    """
    return LineUp(
        left_sensor_name=left_sensor_name,
        right_sensor_name=right_sensor_name,
        base_speed=base_speed,
        confidence_threshold=confidence_threshold,
        reverse=reverse,
        kp=kp,
        ki=ki,
        kd=kd,
    )


def forward_lineup_on_black(
        left_sensor_name: str = "left_front_sensor",
        right_sensor_name: str = "right_front_sensor",
        base_speed: float = 0.6,
        confidence_threshold: float = 0.6,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
) -> Sequential:
    """
    Lines up forward on black lines.

    • base_speed: positive value determining forward speed
    • confidence_threshold: stop when each sensor's black-confidence ≥ thresh
    • kp, ki, kd: PID controller parameters for wheel speed adjustment
    """
    return seq([
        drive_until_white(left_sensor_name, 1),
        lineup(
            left_sensor_name=left_sensor_name,
            right_sensor_name=right_sensor_name,
            base_speed=abs(base_speed),  # Ensure positive value
            confidence_threshold=confidence_threshold,
            reverse=False,  # Target black
            kp=kp,
            ki=ki,
            kd=kd,
        )
    ])


def forward_lineup_on_white(
        left_sensor_name: str = "left_front_sensor",
        right_sensor_name: str = "right_front_sensor",
        base_speed: float = 0.6,
        confidence_threshold: float = 0.6,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
) -> Sequential:
    """
    Lines up forward on white lines.

    • base_speed: positive value determining forward speed
    • confidence_threshold: stop when each sensor's white-confidence ≥ thresh
    • kp, ki, kd: PID controller parameters for wheel speed adjustment
    """
    return seq([
        drive_until_black(left_sensor_name, 1),
        lineup(
            left_sensor_name=left_sensor_name,
            right_sensor_name=right_sensor_name,
            base_speed=abs(base_speed),  # Ensure positive value
            confidence_threshold=confidence_threshold,
            reverse=True,  # Target white
            kp=kp,
            ki=ki,
            kd=kd,
        )])


def backward_lineup_on_black(
        left_sensor_name: str = "left_front_sensor",
        right_sensor_name: str = "right_front_sensor",
        base_speed: float = 0.6,
        confidence_threshold: float = 0.6,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
) -> Sequential:
    """
    Lines up backward on black lines.

    • base_speed: positive value determining backward speed
    • confidence_threshold: stop when each sensor's black-confidence ≥ thresh
    • kp, ki, kd: PID controller parameters for wheel speed adjustment
    """
    return seq([
        drive_until_white(left_sensor_name, -1),
        lineup(
            left_sensor_name=left_sensor_name,
            right_sensor_name=right_sensor_name,
            base_speed=-abs(base_speed),  # Ensure negative value
            confidence_threshold=confidence_threshold,
            reverse=False,  # Target black
            kp=kp,
            ki=ki,
            kd=kd,
        )
    ])


def backward_lineup_on_white(
        left_sensor_name: str = "left_front_sensor",
        right_sensor_name: str = "right_front_sensor",
        base_speed: float = 0.6,
        confidence_threshold: float = 0.6,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
) -> Sequential:
    """
    Lines up backward on white lines.

    • base_speed: positive value determining backward speed
    • confidence_threshold: stop when each sensor's white-confidence ≥ thresh
    • kp, ki, kd: PID controller parameters for wheel speed adjustment
    """
    return seq([
        drive_until_black(left_sensor_name, -1),
        lineup(
            left_sensor_name=left_sensor_name,
            right_sensor_name=right_sensor_name,
            base_speed=-abs(base_speed),  # Ensure negative value
            confidence_threshold=confidence_threshold,
            reverse=True,  # Target white
            kp=kp,
            ki=ki,
            kd=kd,
        )
    ])
