import asyncio
from libstp.motion import StrafeMotion, StrafeConfig, UnifiedMotionPidConfig
from libstp.robot.api import GenericRobot

from .. import Step, SimulationStep, SimulationStepDelta, dsl


@dsl(hidden=True)
class Strafe(Step):
    def __init__(self, config: StrafeConfig):
        """
        Initialize the Strafe step.

        Args:
            config: The StrafeConfig object specifying strafe parameters
        """
        super().__init__()
        self.config = config

    def _generate_signature(self) -> str:
        return (
            f"Strafe(distance_m={self.config.target_distance_m:.3f}, "
            f"speed={self.config.max_speed_mps:.3f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=0.0,
            strafe=self.config.target_distance_m,
            angular=0.0,
        )
        return base

    async def _execute_step(self, robot: GenericRobot) -> None:
        """
        Run the Strafe step.

        :param robot: The robot instance to interact with hardware
        """
        motion = StrafeMotion(robot.drive, robot.odometry, UnifiedMotionPidConfig(), self.config)
        motion.start()  # Explicitly start the motion to reset odometry

        update_rate = 1 / 20  # 20 Hz
        last_time = asyncio.get_event_loop().time() - update_rate  # seed so first dt ~= update_rate
        while not motion.is_finished():
            current_time = asyncio.get_event_loop().time()
            delta_time = max(current_time - last_time, 0.0)
            last_time = current_time

            # Avoid near-zero dt on the very first iteration (causes huge accel FF)
            if delta_time < 1e-4:
                await asyncio.sleep(update_rate)
                continue

            motion.update(delta_time)
            await asyncio.sleep(update_rate)

        robot.drive.hard_stop()


@dsl(tags=["motion", "strafe"])
def strafe_left(cm: float, speed: float = 0.3) -> Strafe:
    """
    Strafe left by specified distance at a given speed.

    Args:
        cm: The distance to strafe in centimeters (positive values move left)
        speed: Maximum lateral speed in m/s (default 0.3 m/s)

    Returns:
        Strafe step configured for leftward motion
    """
    config = StrafeConfig()
    config.target_distance_m = -cm / 100.0  # Negative for left (odometry convention: negative lateral = left)
    config.max_speed_mps = speed
    return Strafe(config)


@dsl(tags=["motion", "strafe"])
def strafe_right(cm: float, speed: float = 0.3) -> Strafe:
    """
    Strafe right by specified distance at a given speed.

    Args:
        cm: The distance to strafe in centimeters (positive values move right)
        speed: Maximum lateral speed in m/s (default 0.3 m/s)

    Returns:
        Strafe step configured for rightward motion
    """
    config = StrafeConfig()
    config.target_distance_m = cm / 100.0  # Positive for right (odometry convention: positive lateral = right)
    config.max_speed_mps = speed
    return Strafe(config)
