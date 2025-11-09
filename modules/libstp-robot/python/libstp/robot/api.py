from libstp.class_name_logger import ClassNameLogger
import asyncio

class GenericRobot(ClassNameLogger):
    def __init__(self):
        self._check_required_variables()

    def _check_required_variables(self):
        if not hasattr(self, "defs"):
            self.error("Robot must have a defs variable (Robot.defs)")
            raise AttributeError("Robot must have a defs variable (Robot.defs)")

        if not hasattr(self, "drive"):
            self.error("Robot must have a drive variable (Robot.drive)")
            raise AttributeError("Robot must have a drive variable (Robot.drive)")

        if not hasattr(self, "missions"):
            self.warn("Robot does not have any missions attached")

        if hasattr(self, "setup_mission"):
            self.info("Setup mission for mission found")

        if hasattr(self, "shutdown_mission"):
            self.info("Shutdown mission for mission found")
    def start(self):
        self.info("Starting robot")

        async def main_loop():
            for mission in self.missions:
                self.info(f"Starting mission: {mission}")
                await mission.run(self)
                self.info(f"Finished mission: {mission}")

        asyncio.run(main_loop())
