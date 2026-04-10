"""
Deadzone calibration UI screens.

These screens guide the user through finding the minimum motor power
where the wheel starts turning.
"""

from dataclasses import dataclass
from typing import Optional

from raccoon.ui.screen import UIScreen
from raccoon.ui.widgets import (
    Widget, Text, Icon, Button, Spacer, HintBox, StatusBadge,
    Center, Row, Column, Card, ResultsTable, StatusIcon,
)
from raccoon.ui.events import on_click, on_button_press


@dataclass
class DeadzoneTestResult:
    """Result from the testing screen."""
    is_turning: bool


@dataclass
class DeadzoneConfirmResult:
    """Result from the confirmation screen."""
    confirmed: bool
    retry_motor: Optional[str] = None  # Motor name to retry, if any


class DeadzoneIntroScreen(UIScreen[None]):
    """
    Introduction screen for deadzone calibration.

    Shows motor info and instructions, waits for button press.
    """

    title = "Deadzone Calibration"

    def __init__(
        self,
        motor_name: str,
        motor_port: int,
        direction: str,  # "FORWARD" or "REVERSE"
        start_percent: int = 1,
        max_percent: int = 30,
    ):
        super().__init__()
        self.motor_name = motor_name
        self.motor_port = motor_port
        self.direction = direction
        self.start_percent = start_percent
        self.max_percent = max_percent

    def build(self) -> Widget:
        direction_color = "green" if self.direction == "FORWARD" else "blue"

        return Center(children=[
            Icon("settings", size=48, color="cyan"),
            Spacer(16),
            Text(f"Motor: {self.motor_name}", size="title", bold=True),
            Text(f"Port {self.motor_port}", size="medium", muted=True),
            Spacer(24),
            StatusBadge(self.direction, color=direction_color),
            Spacer(24),
            Card(children=[
                Text("Watch the wheel carefully!", size="medium", align="center"),
                Spacer(8),
                Text(
                    f"Testing power from {self.start_percent}% to {self.max_percent}%",
                    size="small",
                    muted=True,
                    align="center"
                ),
            ]),
            Spacer(32),
            HintBox("Press button to start testing", icon="touch_app"),
        ])

    @on_button_press()
    async def on_press(self):
        self.close()


class DeadzoneTestingScreen(UIScreen[DeadzoneTestResult]):
    """
    Screen shown during deadzone testing.

    Shows current power percentage and direction, with buttons
    to indicate whether the wheel is turning.
    """

    title = "Is the wheel turning?"

    def __init__(
        self,
        motor_name: str,
        motor_port: int,
        current_percent: int,
        direction: str,
        max_percent: int = 30,
    ):
        super().__init__()
        self.motor_name = motor_name
        self.motor_port = motor_port
        self.current_percent = current_percent
        self.direction = direction
        self.max_percent = max_percent

    def build(self) -> Widget:
        direction_color = "green" if self.direction == "FORWARD" else "blue"

        return Center(children=[
            Row(children=[
                StatusBadge(self.direction, color=direction_color),
                Spacer(16),
                Text(f"{self.motor_name}", size="medium", muted=True),
            ], align="center"),
            Spacer(24),
            Text(f"{self.current_percent}%", size="title", bold=True, color="cyan"),
            Spacer(8),
            Text(f"of {self.max_percent}% max", size="small", muted=True),
            Spacer(32),
            Text("Is the wheel turning?", size="large", align="center"),
            Spacer(24),
            Row(children=[
                Button(
                    "not_turning",
                    "Not Turning",
                    style="secondary",
                    icon="close",
                ),
                Button(
                    "turning",
                    "It's Turning!",
                    style="success",
                    icon="check",
                ),
            ], spacing=24),
            Spacer(32),
            HintBox("Watch the wheel carefully", icon="visibility", style="normal"),
        ])

    @on_click("turning")
    async def on_turning(self):
        self.close(DeadzoneTestResult(is_turning=True))

    @on_click("not_turning")
    async def on_not_turning(self):
        self.close(DeadzoneTestResult(is_turning=False))

    @on_button_press()
    async def on_button(self):
        # Physical button also indicates "It's turning"
        self.close(DeadzoneTestResult(is_turning=True))


class DeadzoneResultsScreen(UIScreen[DeadzoneConfirmResult]):
    """
    Screen showing calibration results for a single motor.

    Allows user to confirm or retry the calibration.
    """

    title = "Calibration Results"
    _primary_button_id = "confirm"

    def __init__(
        self,
        motor_name: str,
        motor_port: int,
        forward_percent: int,
        reverse_percent: int,
        release_percent: int,  # Kept for compatibility but not displayed
    ):
        super().__init__()
        self.motor_name = motor_name
        self.motor_port = motor_port
        self.forward_percent = forward_percent
        self.reverse_percent = reverse_percent
        self.release_percent = release_percent

    @property
    def start_percent(self) -> int:
        """The higher of forward/reverse as the start threshold."""
        return max(self.forward_percent, self.reverse_percent)

    @property
    def kS(self) -> float:
        """Normalized static friction coefficient for ff.kS."""
        return self.start_percent / 100.0

    @property
    def is_symmetric(self) -> bool:
        """Check if forward and reverse are roughly equal."""
        return abs(self.forward_percent - self.reverse_percent) <= 2

    def build(self) -> Widget:
        status_icon = "check" if self.is_symmetric else "warning"
        status_color = "green" if self.is_symmetric else "orange"
        status_text = "Good calibration" if self.is_symmetric else "Asymmetric friction"

        return Center(children=[
            Row(children=[
                StatusIcon(icon=status_icon, color=status_color),
                Spacer(12),
                Column(children=[
                    Text(self.motor_name, size="title", bold=True),
                    Text(f"Port {self.motor_port}", size="small", muted=True),
                ], spacing=4),
            ], align="center"),
            Spacer(16),
            Text(status_text, size="medium", color=status_color),
            Spacer(24),
            Card(children=[
                ResultsTable(rows=[
                    ("Forward Start", f"{self.forward_percent}%", "green"),
                    ("Reverse Start", f"{self.reverse_percent}%", "blue"),
                    ("ff.kS", f"{self.kS:.4f}", "cyan"),
                ]),
            ]),
            Spacer(32),
            Row(children=[
                Button("retry", "Retry Motor", style="secondary", icon="refresh"),
                Button("confirm", "Confirm", style="success", icon="check"),
            ], spacing=16),
        ])

    @on_click("confirm")
    async def on_confirm(self):
        self.close(DeadzoneConfirmResult(confirmed=True))

    @on_click("retry")
    async def on_retry(self):
        self.close(DeadzoneConfirmResult(confirmed=False, retry_motor=self.motor_name))


class DeadzoneSummaryScreen(UIScreen[bool]):
    """
    Summary screen showing results for all motors.

    Allows user to apply all calibrations or cancel.
    """

    title = "ff.kS Calibration Summary"
    _primary_button_id = "apply"

    def __init__(self, results: list):
        """
        Args:
            results: List of dicts with keys: motor_name, port, forward, reverse, kS
        """
        super().__init__()
        self.results = results

    def build(self) -> Widget:
        rows = []
        for r in self.results:
            rows.append((
                f"{r['motor_name']} (port {r['port']})",
                f"F:{r['forward']}% R:{r['reverse']}%",
                None,
            ))
            rows.append((
                "",
                f"ff.kS = {r['kS']:.4f}",
                "cyan",
            ))

        return Center(children=[
            Icon("check_circle", size=48, color="green"),
            Spacer(16),
            Text("Static Friction Calibration Complete", size="title", bold=True),
            Spacer(8),
            Text(f"{len(self.results)} motor(s) calibrated", size="medium", muted=True),
            Spacer(24),
            Card(children=[
                ResultsTable(rows=rows),
            ]),
            Spacer(32),
            Row(children=[
                Button("cancel", "Cancel", style="secondary"),
                Button("apply", "Apply All", style="success", icon="save"),
            ], spacing=16),
        ])

    @on_click("apply")
    async def on_apply(self):
        self.close(True)

    @on_click("cancel")
    async def on_cancel(self):
        self.close(False)
