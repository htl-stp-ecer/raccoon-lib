from typing import List

from libstp.ui import *


class IRCalibratingScreen(UIScreen[None]):
    """
    Screen shown during IR sensor calibration.

    Shows progress spinner and sensor readings.
    """

    title = "IR Sensor Calibration"

    def __init__(self, ports: List[int] = None):
        super().__init__()
        self.ports = ports or []

    def build(self) -> Widget:
        sensor_widgets = []
        for port in self.ports:
            sensor_widgets.append(
                Card(title=f"Port {port}", children=[
                    SensorValue(port=port, sensor_type="analog"),
                ])
            )

        return Split(
            left=[
                Row(children=[
                    ProgressSpinner(size=24),
                    Spacer(8),
                    Text("Calibrating...", size="large"),
                ], align="center"),
                Spacer(8),
                Text("Move over black and white areas", size="small", muted=True),
                Spacer(8),
                AnimatedRobot(moving=True, size=80),
            ],
            right=sensor_widgets if sensor_widgets else [
                Text("No sensors", muted=True),
            ],
            ratio=(1, 1),
        )
