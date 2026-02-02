from libstp.robot.api import GenericRobot
from .. import Step, dsl


@dsl(hidden=True)
class Stop(Step):
    """Step that stops all drive motors."""

    def __init__(self, hard: bool = True) -> None:
        """
        Initialize the Stop step.

        Args:
            hard: If True, immediately zero motor output. If False, decelerate smoothly.
        """
        super().__init__()
        self.hard = hard

    def _generate_signature(self) -> str:
        return f"Stop(hard={self.hard})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        """Stop all drive motors."""
        if self.hard:
            robot.drive.hard_stop()
        else:
            robot.drive.soft_stop()


@dsl(tags=["motion", "stop"])
def stop(hard: bool = True) -> Stop:
    """
    Create a step that stops all drive motors.

    Args:
        hard: If True (default), immediately zero motor output.
              If False, decelerate smoothly.

    Returns:
        Stop step instance
    """
    return Stop(hard)
