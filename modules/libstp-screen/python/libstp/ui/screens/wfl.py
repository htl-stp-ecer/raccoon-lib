"""
Wait-for-light screens: calibration (legacy) and auto-detection.
"""

from dataclasses import dataclass

from ..screen import UIScreen
from ..widgets import (
    Widget, Text, Icon, Button, Spacer,
    StatusBadge, StatusIcon, HintBox, ResultsTable,
    LightBulb, SensorValue, SensorGraph,
    NumericInput, Row, Column, Split, Card, Center,
)
from ..events import on_click, on_button_press, on_change


@dataclass
class WFLMeasureResult:
    """Result from WFLMeasureScreen."""
    value: float


class WFLMeasureScreen(UIScreen[WFLMeasureResult]):
    """
    Measure light on or off state for wait-for-light calibration.

    Shows light bulb visualization and real-time sensor reading.
    Returns sensor value when button is pressed.
    """

    title = "Wait for Light Calibration"

    def __init__(self, port: int, is_on: bool):
        super().__init__()
        self.port = port
        self.is_on = is_on

    def build(self) -> Widget:
        return Split(
            left=[
                StatusBadge(
                    text="LIGHT ON" if self.is_on else "LIGHT OFF",
                    color="amber" if self.is_on else "grey",
                    glow=self.is_on,
                ),
                Spacer(12),
                LightBulb(is_on=self.is_on),
                Spacer(12),
                Text(f"Turn {'ON' if self.is_on else 'OFF'} the lamp", size="large"),
                Spacer(8),
                HintBox("Press button when ready"),
            ],
            right=[
                Card(title="Sensor Value", children=[
                    SensorValue(port=self.port),
                    Spacer(8),
                    SensorGraph(port=self.port),
                ]),
            ],
            ratio=(4, 3),
        )

    @on_button_press()
    async def on_press(self):
        value = await self.read_sensor(self.port)
        self.close(WFLMeasureResult(value=value))


@dataclass
class WFLConfirmResult:
    """Result from WFLConfirmScreen."""
    confirmed: bool
    light_off: float
    light_on: float

    @property
    def threshold(self) -> float:
        return (self.light_off + self.light_on) / 2


class WFLConfirmScreen(UIScreen[WFLConfirmResult]):
    """
    Confirm wait-for-light calibration values.

    Shows measured values (editable), threshold, and difference.
    User can confirm or retry.
    """

    title = "Wait for Light Calibration"
    _primary_button_id = "confirm"

    def __init__(self, port: int, light_off: float, light_on: float):
        super().__init__()
        self.port = port
        self.light_off = light_off
        self.light_on = light_on

    @property
    def threshold(self) -> float:
        return (self.light_off + self.light_on) / 2

    @property
    def difference(self) -> float:
        return abs(self.light_on - self.light_off)

    @property
    def is_good(self) -> bool:
        return self.difference > 100

    def build(self) -> Widget:
        return Split(
            left=[
                StatusIcon(
                    icon="check" if self.is_good else "warning",
                    color="green" if self.is_good else "orange",
                ),
                Spacer(8),
                Text(
                    "Calibration Complete" if self.is_good else "Low Contrast",
                    size="large",
                ),
                Spacer(16),
                Row(children=[
                    Column(children=[
                        Icon("lightbulb_outline", color="grey"),
                        Text("Light OFF", size="small", muted=True),
                        NumericInput(id="light_off", value=self.light_off),
                    ], spacing=4),
                    Column(children=[
                        Icon("lightbulb", color="amber"),
                        Text("Light ON", size="small", muted=True),
                        NumericInput(id="light_on", value=self.light_on),
                    ], spacing=4),
                ], spacing=24),
            ],
            right=[
                Card(children=[
                    ResultsTable(rows=[
                        ("Threshold", f"{self.threshold:.0f}", "blue"),
                        ("Difference", f"{self.difference:.0f}",
                         "green" if self.is_good else "orange"),
                    ]),
                    Spacer(24),
                    Column(children=[
                        Button("retry", "Retry", style="secondary"),
                        Button("confirm", "Confirm",
                               style="success" if self.is_good else "warning"),
                    ], spacing=8),
                ]),
            ],
            ratio=(5, 3),
        )

    @on_change("light_off")
    async def on_off_change(self, value: float):
        self.light_off = value
        await self.refresh()

    @on_change("light_on")
    async def on_on_change(self, value: float):
        self.light_on = value
        await self.refresh()

    @on_click("retry")
    async def on_retry(self):
        self.close(WFLConfirmResult(
            confirmed=False,
            light_off=self.light_off,
            light_on=self.light_on,
        ))

    @on_click("confirm")
    async def on_confirm(self):
        self.close(WFLConfirmResult(
            confirmed=True,
            light_off=self.light_off,
            light_on=self.light_on,
        ))


# =============================================================================
# Auto-detection screen (Kalman-filtered flank detection)
# =============================================================================


class WFLDetectScreen(UIScreen[None]):
    """
    Status display for automatic wait-for-light detection.

    Shows the current sensor value, Kalman-filtered baseline, trigger
    threshold, and detection status (WARMING UP / ARMED / GO!).
    This screen is display-only — no user interaction needed.
    """

    title = "Wait for Light"

    def __init__(self) -> None:
        super().__init__()
        self.status: str = "WARMING UP"
        self.status_color: str = "amber"
        self.raw_value: int = 0
        self.baseline: float = 0.0
        self.threshold: float = 0.0

    def build(self) -> Widget:
        return Center(children=[
            Column(children=[
                StatusBadge(
                    text=self.status,
                    color=self.status_color,
                    glow=self.status == "ARMED",
                ),
                Spacer(16),
                Text(f"Sensor: {self.raw_value}", size="xlarge"),
                Spacer(8),
                ResultsTable(rows=[
                    ("Baseline", f"{self.baseline:.0f}", "grey"),
                    ("Threshold", f"{self.threshold:.0f}", "blue"),
                ]),
            ], align="center", spacing=0),
        ])
