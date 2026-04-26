"""Analog sensor calibration UI screens."""

from __future__ import annotations

from dataclasses import dataclass

from ..events import on_button_press, on_click
from ..screen import UIScreen
from ..widgets import (
    Button,
    Card,
    Center,
    Column,
    HintBox,
    Icon,
    ProgressSpinner,
    ResultsTable,
    Row,
    Spacer,
    Split,
    StatusIcon,
    Text,
    Widget,
)


@dataclass
class AnalogSensorConfirmResult:
    """Result from AnalogSensorConfirmScreen."""

    confirmed: bool


class AnalogSensorPositionScreen(UIScreen[None]):
    """Prompt the operator to position the robot at the target location."""

    title = "Analog Sensor Calibration"

    def __init__(self, port: int, set_name: str = "default"):
        super().__init__()
        self.port = port
        self.set_name = set_name

    def build(self) -> Widget:
        set_label = f"  [{self.set_name}]" if self.set_name != "default" else ""
        return Center(
            children=[
                Icon("sensors", size=48, color="cyan"),
                Spacer(16),
                Text(f"Sensor Port {self.port}{set_label}", size="title", bold=True),
                Spacer(24),
                Text("Position the robot at the target location", size="large", align="center"),
                Spacer(12),
                Text(
                    "The sensor will capture a reference reading at this position.",
                    muted=True,
                    align="center",
                ),
                Spacer(32),
                HintBox("Press button when in position", icon="touch_app", style="prominent"),
            ]
        )

    @on_button_press()
    async def on_press(self):
        self.close()


class AnalogSensorSamplingScreen(UIScreen[None]):
    """Shown while the sensor is being sampled (closed externally via run_with_ui)."""

    title = "Analog Sensor Calibration"

    def __init__(self, port: int, duration: float):
        super().__init__()
        self.port = port
        self.duration = duration

    def build(self) -> Widget:
        return Center(
            children=[
                Row(
                    children=[
                        ProgressSpinner(size=32),
                        Spacer(16),
                        Text("Sampling…", size="large"),
                    ],
                    align="center",
                ),
                Spacer(24),
                Text(f"Sensor port {self.port}", muted=True),
                Spacer(8),
                Text(f"Duration: {self.duration:.1f} s", muted=True),
                Spacer(32),
                HintBox("Hold the robot still", icon="pan_tool"),
            ]
        )


class AnalogSensorConfirmScreen(UIScreen[AnalogSensorConfirmResult]):
    """Show the captured reference value for operator confirmation."""

    title = "Analog Sensor Calibration"
    _primary_button_id = "confirm"

    def __init__(
        self,
        port: int,
        set_name: str,
        target_value: float,
        std: float,
        sample_count: int,
    ):
        super().__init__()
        self.port = port
        self.set_name = set_name
        self.target_value = target_value
        self.std = std
        self.sample_count = sample_count

    @property
    def _is_stable(self) -> bool:
        if self.target_value == 0:
            return True
        return self.std < self.target_value * 0.05  # < 5 % relative std

    def build(self) -> Widget:
        icon = "check" if self._is_stable else "warning"
        color = "green" if self._is_stable else "orange"
        status = "Stable reading" if self._is_stable else "High noise — consider retrying"

        return Split(
            left=[
                Row(
                    children=[
                        StatusIcon(icon=icon, color=color),
                        Spacer(12),
                        Column(
                            children=[
                                Text(f"Port {self.port}", size="title", bold=True),
                                Text(f"Set: {self.set_name}", size="small", muted=True),
                            ],
                            spacing=4,
                        ),
                    ],
                    align="center",
                ),
                Spacer(16),
                Text(status, size="medium", color=color),
                Spacer(24),
                Card(
                    children=[
                        ResultsTable(
                            rows=[
                                ("Target Value", f"{self.target_value:.1f}", "cyan"),
                                (
                                    "Std Dev",
                                    f"±{self.std:.1f}",
                                    "blue" if self._is_stable else "orange",
                                ),
                                ("Samples", str(self.sample_count), None),
                            ]
                        ),
                    ]
                ),
            ],
            right=[
                Column(
                    children=[
                        Button(
                            "confirm",
                            "Confirm",
                            style="success" if self._is_stable else "warning",
                            icon="check",
                        ),
                        Button("retry", "Retry", style="secondary", icon="refresh"),
                    ],
                    spacing=16,
                ),
            ],
            ratio=(5, 4),
        )

    @on_click("confirm")
    async def on_confirm(self):
        self.close(AnalogSensorConfirmResult(confirmed=True))

    @on_click("retry")
    async def on_retry(self):
        self.close(AnalogSensorConfirmResult(confirmed=False))
