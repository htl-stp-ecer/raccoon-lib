"""
Distance calibration screens.
"""

from dataclasses import dataclass

from ..screen import UIScreen
from ..widgets import (
    Widget, Text, Button, Spacer,
    StatusIcon, HintBox, ResultsTable,
    NumericKeypad, NumericInput,
    AnimatedRobot, PulsingArrow, RobotDrivingAnimation, ProgressSpinner,
    Center, Row, Column, Split, Card,
)
from ..events import on_click, on_button_press, on_keypad, on_change


class DistancePrepareScreen(UIScreen[None]):
    """
    Prepare screen for distance calibration.

    Shows robot and target distance. Waits for button press to start.
    """

    title = "Distance Calibration"

    def __init__(self, requested_distance: float):
        super().__init__()
        self.requested_distance = requested_distance

    def build(self) -> Widget:
        return Center(children=[
            Row(children=[
                AnimatedRobot(moving=False),
                Spacer(16),
                PulsingArrow(),
            ], align="center"),
            Spacer(24),
            Text("Distance Calibration", size="title"),
            Spacer(16),
            Text(f"{self.requested_distance:.0f} cm", size="title", color="blue", bold=True),
            Spacer(8),
            Text("Robot will drive this distance forward", muted=True),
            Spacer(32),
            HintBox("Press button to start", style="prominent"),
        ])

    @on_button_press()
    async def on_press(self):
        self.close()


class DistanceDrivingScreen(UIScreen[None]):
    """
    Driving animation screen shown while robot moves.

    This screen should be closed externally when driving completes.
    """

    title = "Distance Calibration"

    def __init__(self, requested_distance: float):
        super().__init__()
        self.requested_distance = requested_distance

    def build(self) -> Widget:
        return Center(children=[
            RobotDrivingAnimation(target_distance=self.requested_distance),
            Spacer(32),
            Row(children=[
                ProgressSpinner(size=24),
                Spacer(12),
                Text("Robot is driving...", size="large"),
            ], align="center"),
            Spacer(16),
            Text(f"Target: {self.requested_distance:.0f} cm", color="orange"),
        ])


class DistanceMeasureScreen(UIScreen[float]):
    """
    Measure actual distance traveled.

    Shows numeric keypad for entering measured distance.
    """

    title = "Distance Calibration"
    _primary_button_id = "submit"

    def __init__(self, requested_distance: float, default_value: float = None):
        super().__init__()
        self.requested_distance = requested_distance
        self.value = default_value if default_value is not None else requested_distance
        self._input_str = ""

    def build(self) -> Widget:
        return Split(
            left=[
                Text("Enter actual distance", size="title"),
                Spacer(8),
                Text(f"Robot attempted: {self.requested_distance:.0f} cm", muted=True),
                Spacer(24),
                NumericInput(
                    id="value",
                    value=self.value,
                    unit="cm",
                    min_value=0,
                ),
                Spacer(24),
                Button("submit", "Submit", style="success", icon="check"),
            ],
            right=[
                NumericKeypad(),
            ],
            ratio=(5, 6),
        )

    @on_keypad()
    async def on_key(self, key: str):
        if key == "back":
            self._input_str = self._input_str[:-1]
        elif key == "." and "." not in self._input_str:
            self._input_str += "."
        elif key.isdigit():
            self._input_str += key

        try:
            self.value = float(self._input_str) if self._input_str else 0
        except ValueError:
            self.value = 0

        await self.refresh()

    @on_change("value")
    async def on_adjust(self, value: float):
        self.value = max(0, value)
        self._input_str = str(self.value)
        await self.refresh()

    @on_click("submit")
    async def on_submit(self):
        self.close(self.value)


@dataclass
class DistanceConfirmResult:
    """Result from DistanceConfirmScreen."""
    confirmed: bool
    scale_factor: float


class DistanceConfirmScreen(UIScreen[DistanceConfirmResult]):
    """
    Confirm distance calibration results.

    Shows requested, measured, scale factor, and adjustment percentage.
    """

    title = "Distance Calibration"
    _primary_button_id = "apply"

    def __init__(self, requested: float, measured: float):
        super().__init__()
        self.requested = requested
        self.measured = measured

    @property
    def scale_factor(self) -> float:
        if self.requested == 0:
            return 1.0
        return self.measured / self.requested

    @property
    def adjustment(self) -> float:
        return (self.scale_factor - 1.0) * 100

    @property
    def is_good(self) -> bool:
        return abs(self.adjustment) < 10

    def build(self) -> Widget:
        sign = "+" if self.adjustment >= 0 else ""

        return Split(
            left=[
                Row(children=[
                    StatusIcon(
                        icon="check" if self.is_good else "warning",
                        color="green" if self.is_good else "orange",
                    ),
                    Spacer(12),
                    Text(
                        "Calibration Complete!" if self.is_good else "Large Adjustment",
                        size="large",
                    ),
                ], align="center"),
                Spacer(16),
                Card(children=[
                    ResultsTable(rows=[
                        ("Requested", f"{self.requested:.1f} cm", None),
                        ("Measured", f"{self.measured:.1f} cm", None),
                        ("Scale", f"{self.scale_factor:.4f}", "blue"),
                        ("Adjust", f"{sign}{self.adjustment:.1f}%",
                         "green" if self.is_good else "orange"),
                    ]),
                ]),
            ],
            right=[
                Column(children=[
                    Button("apply", "Apply",
                           style="success" if self.is_good else "warning",
                           icon="check"),
                    Button("retry", "Retry", style="secondary", icon="refresh"),
                ], spacing=16),
            ],
            ratio=(5, 4),
        )

    @on_click("apply")
    async def on_apply(self):
        self.close(DistanceConfirmResult(
            confirmed=True,
            scale_factor=self.scale_factor,
        ))

    @on_click("retry")
    async def on_retry(self):
        self.close(DistanceConfirmResult(
            confirmed=False,
            scale_factor=self.scale_factor,
        ))
