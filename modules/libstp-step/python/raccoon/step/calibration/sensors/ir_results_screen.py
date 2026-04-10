from typing import List, Optional

from raccoon.ui import *

from .dataclasses import SensorCalibrationData, IRDashboardResult


class IRResultsDashboardScreen(UIScreen[IRDashboardResult]):
    """
    Calibration results dashboard with sensor list and detail view.

    Left panel: sensor list (clickable) + dashboard summary button.
    Right panel: dashboard overview or per-sensor detail with graph.
    Bottom: retry + done buttons.
    """

    title = "IR Sensor Calibration"
    _primary_button_id = "done"

    def __init__(self, sensors: List[SensorCalibrationData]):
        super().__init__()
        self.sensors = sensors
        self.selected_port: Optional[int] = None  # None = dashboard view

    @property
    def all_good(self) -> bool:
        return all(s.is_good for s in self.sensors)

    def _selected_sensor(self) -> Optional[SensorCalibrationData]:
        if self.selected_port is None:
            return None
        return next((s for s in self.sensors if s.port == self.selected_port), None)

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------

    def build(self) -> Widget:
        return Column(children=[
            Expanded(child=Split(
                left=self._build_sensor_list(),
                right=self._build_content(),
                ratio=(1, 3),
            )),
            Spacer(8),
            Row(children=[
                Button("retry", "Retry", style="secondary", icon="refresh"),
                Button("done", "Done",
                       style="success" if self.all_good else "warning",
                       icon="check"),
            ], spacing=12, align="center"),
        ])

    def _build_sensor_list(self) -> List[Widget]:
        items: List[Widget] = []

        # Dashboard button
        items.append(Button(
            "nav_dashboard", "Dashboard",
            style="primary" if self.selected_port is None else "secondary",
            icon="dashboard",
        ))
        items.append(Spacer(4))
        items.append(Divider())
        items.append(Spacer(4))

        # Per-sensor buttons
        for sensor in self.sensors:
            icon = "check_circle" if sensor.is_good else "warning"
            style = "success" if (self.selected_port == sensor.port) else "secondary"
            label = f"Port {sensor.port}"
            if not sensor.is_good:
                label += " !"

            items.append(Button(
                f"nav_sensor_{sensor.port}", label,
                style=style,
                icon=icon,
            ))

        return items

    def _build_content(self) -> List[Widget]:
        sensor = self._selected_sensor()
        if sensor is None:
            return self._build_dashboard()
        return self._build_sensor_detail(sensor)

    # ------------------------------------------------------------------
    # Dashboard view
    # ------------------------------------------------------------------

    def _build_dashboard(self) -> List[Widget]:
        widgets: List[Widget] = []

        # Header
        widgets.append(Row(children=[
            StatusIcon(
                icon="check" if self.all_good else "warning",
                color="green" if self.all_good else "orange",
            ),
            Spacer(8),
            Text(
                "All Sensors OK" if self.all_good else "Some Sensors Need Attention",
                size="large",
            ),
        ], align="center"))
        widgets.append(Spacer(8))

        # Summary cards per sensor
        for sensor in self.sensors:
            status_color = "green" if sensor.is_good else "orange"
            widgets.append(Card(title=f"Port {sensor.port}", children=[
                Row(children=[
                    StatusIcon(
                        icon="check" if sensor.is_good else "warning",
                        color=status_color,
                    ),
                    Spacer(8),
                    Column(children=[
                        Row(children=[
                            Text("Black: ", size="small", muted=True),
                            Text(f"{sensor.black_threshold:.0f}", size="small", bold=True),
                            Spacer(12),
                            Text("White: ", size="small", muted=True),
                            Text(f"{sensor.white_threshold:.0f}", size="small", bold=True),
                            Spacer(12),
                            Text("Sep: ", size="small", muted=True),
                            Text(f"{sensor.separation:.0f}", size="small", bold=True,
                                 color=status_color),
                        ], spacing=2),
                    ], spacing=2),
                ], align="center"),
            ]))
            widgets.append(Spacer(4))

        return widgets

    # ------------------------------------------------------------------
    # Sensor detail view
    # ------------------------------------------------------------------

    def _build_sensor_detail(self, sensor: SensorCalibrationData) -> List[Widget]:
        widgets: List[Widget] = []

        # Title row
        widgets.append(Row(children=[
            StatusIcon(
                icon="check" if sensor.is_good else "warning",
                color="green" if sensor.is_good else "orange",
            ),
            Spacer(8),
            Text(f"Port {sensor.port}", size="large"),
            Spacer(8),
            StatusBadge(
                text="OK" if sensor.is_good else "LOW CONTRAST",
                color="green" if sensor.is_good else "orange",
            ),
        ], align="center"))
        widgets.append(Spacer(8))

        # Chart with samples + threshold lines
        if sensor.samples:
            widgets.append(CalibrationChart(
                samples=sensor.samples,
                thresholds=[
                    (sensor.black_threshold, "Black", "grey"),
                    (sensor.white_threshold, "White", "amber"),
                ],
                height=180,
            ))
            widgets.append(Spacer(8))

        # Threshold values + stats
        widgets.append(Row(children=[
            Card(title="Black", children=[
                Text(f"{sensor.black_threshold:.0f}", size="large", bold=True),
                Text(f"mean={sensor.black_mean:.0f} std={sensor.black_std:.1f}",
                     size="small", muted=True),
            ]),
            Card(title="White", children=[
                Text(f"{sensor.white_threshold:.0f}", size="large", bold=True),
                Text(f"mean={sensor.white_mean:.0f} std={sensor.white_std:.1f}",
                     size="small", muted=True),
            ]),
            Card(title="Separation", children=[
                Text(f"{sensor.separation:.0f}", size="large", bold=True,
                     color="green" if sensor.is_good else "orange"),
                Text(f"{len(sensor.samples)} samples",
                     size="small", muted=True),
            ]),
        ], spacing=8))

        return widgets

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    @on_click("nav_dashboard")
    async def on_nav_dashboard(self):
        self.selected_port = None
        await self.refresh()

    @on_click("retry")
    async def on_retry(self):
        self.close(IRDashboardResult(confirmed=False, sensors=self.sensors))

    @on_click("done")
    async def on_done(self):
        self.close(IRDashboardResult(confirmed=True, sensors=self.sensors))

    async def _dispatch_event(self, event: dict) -> None:
        """Handle sensor navigation clicks dynamically."""
        await super()._dispatch_event(event)

        action = event.get("_action")
        button_id = event.get("button_id", "")

        if action == "click" and button_id.startswith("nav_sensor_"):
            port = int(button_id.replace("nav_sensor_", ""))
            self.selected_port = port
            await self.refresh()
