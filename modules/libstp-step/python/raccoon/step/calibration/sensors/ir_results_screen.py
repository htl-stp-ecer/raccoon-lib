from __future__ import annotations

from raccoon.ui import *

from .dataclasses import IRDashboardResult, SensorCalibrationData

# Distinct per-sensor series colors (cycled). Kept in sync with the colors the
# botui CalibrationChart widget knows how to parse.
_SERIES_COLORS = ["blue", "green", "amber", "red", "purple", "cyan", "orange", "pink"]


def _series_color(index: int) -> str:
    return _SERIES_COLORS[index % len(_SERIES_COLORS)]


class IRResultsDashboardScreen(UIScreen[IRDashboardResult]):
    """
    Calibration-set results: a status overview plus one combined graph.

    Top row: a short status overview (overall icon + per-sensor badges).
    Below: a single chart plotting every sensor's samples at once, each sensor
    drawn in its own color so the whole calibration set is visible in one view.
    Bottom: retry + done buttons.
    """

    title = "IR Sensor Calibration"
    _primary_button_id = "done"

    def __init__(
        self,
        sensors: list[SensorCalibrationData],
        set_name: str | None = None,
    ):
        super().__init__()
        self.sensors = sensors
        self.set_name = set_name
        if set_name:
            self.title = f"IR Sensor Calibration: {set_name.upper()}"

    @property
    def all_good(self) -> bool:
        return all(s.is_good for s in self.sensors)

    @property
    def _num_good(self) -> int:
        return sum(1 for s in self.sensors if s.is_good)

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------

    def build(self) -> Widget:
        children: list[Widget] = []
        if self.set_name:
            children.append(
                Text(f"Calibration set: {self.set_name.upper()}", size="large", bold=True)
            )
            children.append(Spacer(8))
        children.extend(
            [
                self._build_status_row(),
                Spacer(12),
                # Let the chart take the remaining height instead of a fixed 320px:
                # on shorter robot displays a fixed height overflowed the screen and
                # the chart's lower edge — the white/lower threshold line — was
                # clipped off-screen. Expanded makes the column fill and the chart
                # shrink to fit, keeping both thresholds visible.
                Expanded(child=self._build_chart()),
                Spacer(12),
                Row(
                    children=[
                        Button("retry", "Retry", style="secondary", icon="refresh"),
                        Button(
                            "done",
                            "Done",
                            style="success" if self.all_good else "warning",
                            icon="check",
                        ),
                    ],
                    spacing=12,
                    align="center",
                ),
            ]
        )
        return Column(children=children)

    def _build_status_row(self) -> Widget:
        # Per-sensor badge colored to match its graph series, so the status
        # overview and the chart legend read as one. Failing sensors flip to red.
        badges: list[Widget] = []
        for index, sensor in enumerate(self.sensors):
            status = "OK" if sensor.is_good else "LOW"
            badges.append(
                StatusBadge(
                    text=f"P{sensor.port} · {status} · Δ{sensor.separation:.0f}",
                    color=_series_color(index) if sensor.is_good else "red",
                )
            )

        return Row(
            children=[
                StatusIcon(
                    icon="check" if self.all_good else "warning",
                    color="green" if self.all_good else "orange",
                ),
                Spacer(8),
                Text(
                    f"{self._num_good}/{len(self.sensors)} sensors OK",
                    size="large",
                    bold=True,
                ),
                Spacer(16),
                *badges,
            ],
            align="center",
            spacing=8,
        )

    def _build_chart(self) -> Widget:
        # One colored series per sensor, each carrying its own black/white
        # thresholds — the chart draws them as dashed lines in the series color
        # and lists the values in the legend.
        series = [
            {
                "label": f"Port {sensor.port}",
                "color": _series_color(index),
                "samples": [float(v) for v in sensor.samples],
                "black_threshold": float(sensor.black_threshold),
                "white_threshold": float(sensor.white_threshold),
            }
            for index, sensor in enumerate(self.sensors)
        ]
        return CalibrationChart(series=series, height=320)

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    @on_click("retry")
    async def on_retry(self):
        self.close(IRDashboardResult(confirmed=False, sensors=self.sensors))

    @on_click("done")
    async def on_done(self):
        self.close(IRDashboardResult(confirmed=True, sensors=self.sensors))
