from typing import List

from libstp.ui import *

from .dataclasses import IRConfirmResult


class IRConfirmScreen(UIScreen[IRConfirmResult]):
    """
    Confirm IR sensor calibration results.

    Shows black/white thresholds and collected values.
    """

    title = "IR Sensor Calibration"

    def __init__(
        self,
        black_threshold: float,
        white_threshold: float,
        collected_values: List[float] = None,
    ):
        super().__init__()
        self.black_threshold = black_threshold
        self.white_threshold = white_threshold
        self.collected_values = collected_values or []

    @property
    def is_good(self) -> bool:
        return abs(self.white_threshold - self.black_threshold) > 100

    def build(self) -> Widget:
        return Split(
            left=[
                Row(children=[
                    StatusIcon(
                        icon="check" if self.is_good else "warning",
                        color="green" if self.is_good else "orange",
                    ),
                    Spacer(8),
                    Text(
                        "Complete" if self.is_good else "Low Contrast",
                        size="large",
                    ),
                ], align="center"),
                Spacer(12),
                Row(children=[
                    Column(children=[
                        Text("Black", size="small", muted=True),
                        NumericInput(id="black", value=self.black_threshold),
                    ], spacing=2),
                    Column(children=[
                        Text("White", size="small", muted=True),
                        NumericInput(id="white", value=self.white_threshold),
                    ], spacing=2),
                ], spacing=16),
            ],
            right=[
                ResultsTable(rows=[
                    ("Black", f"{self.black_threshold:.0f}", "grey"),
                    ("White", f"{self.white_threshold:.0f}", "white"),
                    ("Diff", f"{abs(self.white_threshold - self.black_threshold):.0f}",
                     "green" if self.is_good else "orange"),
                ]),
                Spacer(12),
                Row(children=[
                    Button("retry", "Retry", style="secondary"),
                    Button("confirm", "Confirm",
                           style="success" if self.is_good else "warning"),
                ], spacing=8),
            ],
            ratio=(1, 1),
        )

    @on_change("black")
    async def on_black_change(self, value: float):
        self.black_threshold = value
        await self.refresh()

    @on_change("white")
    async def on_white_change(self, value: float):
        self.white_threshold = value
        await self.refresh()

    @on_click("retry")
    async def on_retry(self):
        self.close(IRConfirmResult(
            confirmed=False,
            black_threshold=self.black_threshold,
            white_threshold=self.white_threshold,
        ))

    @on_click("confirm")
    async def on_confirm(self):
        self.close(IRConfirmResult(
            confirmed=True,
            black_threshold=self.black_threshold,
            white_threshold=self.white_threshold,
        ))
