"""
Distance calibration screens.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..events import on_button_press, on_change, on_click, on_keypad
from ..screen import UIScreen
from ..widgets import (
    AnimatedRobot,
    Button,
    Card,
    Center,
    Column,
    HintBox,
    NumericInput,
    NumericKeypad,
    ProgressSpinner,
    PulsingArrow,
    ResultsTable,
    RobotDrivingAnimation,
    Row,
    Spacer,
    Split,
    StatusIcon,
    Text,
    Widget,
)


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
        return Center(
            children=[
                Row(
                    children=[
                        AnimatedRobot(moving=False),
                        Spacer(16),
                        PulsingArrow(),
                    ],
                    align="center",
                ),
                Spacer(24),
                Text("Distance Calibration", size="title"),
                Spacer(16),
                Text(f"{self.requested_distance:.0f} cm", size="title", color="blue", bold=True),
                Spacer(8),
                Text("Robot will drive this distance forward", muted=True),
                Spacer(32),
                HintBox("Press button to start", style="prominent"),
            ]
        )

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
        return Center(
            children=[
                RobotDrivingAnimation(target_distance=self.requested_distance),
                Spacer(32),
                Row(
                    children=[
                        ProgressSpinner(size=24),
                        Spacer(12),
                        Text("Robot is driving...", size="large"),
                    ],
                    align="center",
                ),
                Spacer(16),
                Text(f"Target: {self.requested_distance:.0f} cm", color="orange"),
            ]
        )


class _RetryRequested:
    """Sentinel result: the operator wants to re-drive instead of measuring.

    Returned by :class:`DistanceMeasureScreen` when ``allow_retry`` is set and the
    operator taps *Retry* (e.g. because the robot was bumped or moved before they
    could measure). Callers re-run the drive instead of recording a sample.
    """

    __slots__ = ()

    def __repr__(self) -> str:  # pragma: no cover - debug aid
        return "DISTANCE_MEASURE_RETRY"


#: Singleton sentinel, identity-compared (``measured is DISTANCE_MEASURE_RETRY``).
DISTANCE_MEASURE_RETRY = _RetryRequested()


class DistanceMeasureScreen(UIScreen[float]):
    """
    Measure actual distance traveled.

    Shows numeric keypad for entering measured distance. When ``allow_retry`` is
    set, an extra *Retry* button lets the operator re-drive (returning
    :data:`DISTANCE_MEASURE_RETRY`) instead of entering a measurement — useful when
    the robot was moved after the drive.
    """

    title = "Distance Calibration"
    _primary_button_id = "submit"

    def __init__(
        self,
        requested_distance: float,
        default_value: float | None = None,
        allow_retry: bool = False,
    ):
        super().__init__()
        self.requested_distance = requested_distance
        self.value = default_value if default_value is not None else requested_distance
        self.allow_retry = allow_retry
        self._input_str = ""

    def build(self) -> Widget:
        actions: list[Widget] = [Button("submit", "Submit", style="success", icon="check")]
        if self.allow_retry:
            actions.append(
                Button("retry", "Retry", style="secondary", icon="refresh"),
            )
        left: list[Widget] = [
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
            Row(children=actions, spacing=12),
        ]
        if self.allow_retry:
            left.append(Spacer(8))
            left.append(Text("Robot moved? Tap Retry to drive again.", size="small", muted=True))
        return Split(
            left=left,
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

    @on_click("retry")
    async def on_retry(self):
        self.close(DISTANCE_MEASURE_RETRY)


@dataclass
class DistanceConfirmResult:
    """Result from DistanceConfirmScreen.

    ``confirmed`` applies the trim. When ``confirmed`` is ``False`` the operator
    backed out: ``reenter`` distinguishes *just fix the typed number* (re-open the
    measurement keypad, keep the drive) from *redo the whole calibration*
    (``reenter=False``, drive again).
    """

    confirmed: bool
    scale_factor: float
    reenter: bool = False


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
        # Distance-correction factor: how much to scale the NEXT commanded
        # distance to land on target. Driving short (measured < requested)
        # yields a factor > 1 so the next drive is lengthened, not shortened.
        if self.measured == 0:
            return 1.0
        return self.requested / self.measured

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
                Row(
                    children=[
                        StatusIcon(
                            icon="check" if self.is_good else "warning",
                            color="green" if self.is_good else "orange",
                        ),
                        Spacer(12),
                        Text(
                            "Calibration Complete!" if self.is_good else "Large Adjustment",
                            size="large",
                        ),
                    ],
                    align="center",
                ),
                Spacer(16),
                Card(
                    children=[
                        ResultsTable(
                            rows=[
                                ("Requested", f"{self.requested:.1f} cm", None),
                                ("Measured", f"{self.measured:.1f} cm", None),
                                ("Scale", f"{self.scale_factor:.4f}", "blue"),
                                (
                                    "Adjust",
                                    f"{sign}{self.adjustment:.1f}%",
                                    "green" if self.is_good else "orange",
                                ),
                            ]
                        ),
                    ]
                ),
            ],
            right=[
                Column(
                    children=[
                        Button(
                            "apply",
                            "Apply",
                            style="success" if self.is_good else "warning",
                            icon="check",
                        ),
                        Button("reenter", "Re-enter Value", style="secondary", icon="edit"),
                        Button("retry", "Redo Calibration", style="secondary", icon="refresh"),
                    ],
                    spacing=16,
                ),
            ],
            ratio=(5, 4),
        )

    @on_click("apply")
    async def on_apply(self):
        self.close(
            DistanceConfirmResult(
                confirmed=True,
                scale_factor=self.scale_factor,
            )
        )

    @on_click("reenter")
    async def on_reenter(self):
        self.close(
            DistanceConfirmResult(
                confirmed=False,
                scale_factor=self.scale_factor,
                reenter=True,
            )
        )

    @on_click("retry")
    async def on_retry(self):
        self.close(
            DistanceConfirmResult(
                confirmed=False,
                scale_factor=self.scale_factor,
            )
        )
