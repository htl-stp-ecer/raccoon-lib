from __future__ import annotations

from raccoon.ui import *

from .dataclasses import IRCalibrationChoice


class IROverviewScreen(UIScreen[IRCalibrationChoice]):
    """
    Overview screen for IR sensor black/white calibration.

    Shows current status and lets user choose to calibrate or use existing values.
    """

    title = "IR Sensor Calibration"

    def __init__(self, has_existing: bool = False, set_name: str = "default"):
        super().__init__()
        self.has_existing = has_existing
        self.set_name = set_name

    def build(self) -> Widget:
        buttons = [
            Button("start", "Start", style="primary", icon="play_arrow"),
        ]
        if self.has_existing:
            buttons.append(Button("use_existing", "Use Existing", style="secondary"))

        subtitle = "Calibrate black and white thresholds"
        if self.set_name != "default":
            subtitle = f"Calibrate thresholds for '{self.set_name}' surface"

        return Split(
            left=[
                StatusIcon(icon="sensors", color="blue"),
                Spacer(8),
                Text("IR Sensor Calibration", size="large"),
                Spacer(4),
                Text(subtitle, size="small", muted=True),
            ],
            right=[
                HintBox("Place sensors over surface", style="prominent"),
                Spacer(16),
                Row(children=buttons, spacing=12),
            ],
            ratio=(1, 1),
        )

    @on_click("start")
    async def on_start(self):
        self.close(IRCalibrationChoice(use_existing=False))

    @on_click("use_existing")
    async def on_use_existing(self):
        self.close(IRCalibrationChoice(use_existing=True))

    @on_button_press()
    async def on_press(self):
        self.close(IRCalibrationChoice(use_existing=False))
