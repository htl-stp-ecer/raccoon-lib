"""
Basic pre-built screens for common UI patterns.
"""

from typing import Optional, List

from ..screen import UIScreen
from ..widgets import (
    Widget, Text, Icon, Button, Spacer, HintBox, ProgressSpinner,
    Center, Row, Column,
)
from ..events import on_click, on_button_press


class WaitForButtonScreen(UIScreen[None]):
    """
    Shows a message and waits for physical button press.

    Example:
        await self.show(WaitForButtonScreen("Place robot at start"))
    """

    title = "Ready"

    def __init__(self, message: str = "Press the button to continue",
                 icon_name: str = "touch_app", icon_color: str = "amber"):
        super().__init__()
        self.message = message
        self.icon_name = icon_name
        self.icon_color = icon_color

    def build(self) -> Widget:
        return Center(children=[
            Icon(self.icon_name, size=64, color=self.icon_color),
            Spacer(16),
            Text(self.message, size="large", align="center"),
            Spacer(24),
            HintBox("Waiting for button press..."),
        ])

    @on_button_press()
    async def on_press(self):
        self.close()


class ConfirmScreen(UIScreen[bool]):
    """
    Two-button confirmation dialog. Returns True or False.

    Example:
        confirmed = await self.show(ConfirmScreen(
            "Warning",
            "Robot will move. Continue?"
        ))
    """

    def __init__(self, title: str, message: str,
                 confirm_label: str = "Confirm",
                 cancel_label: str = "Cancel",
                 confirm_style: str = "success",
                 icon_name: str = "help_outline",
                 icon_color: str = "blue"):
        super().__init__()
        self.title = title
        self.message = message
        self.confirm_label = confirm_label
        self.cancel_label = cancel_label
        self.confirm_style = confirm_style
        self.icon_name = icon_name
        self.icon_color = icon_color

    def build(self) -> Widget:
        return Center(children=[
            Icon(self.icon_name, size=48, color=self.icon_color),
            Spacer(16),
            Text(self.message, size="large", align="center"),
            Spacer(32),
            Row(children=[
                Button("cancel", self.cancel_label, style="secondary"),
                Button("confirm", self.confirm_label, style=self.confirm_style),
            ], spacing=16),
        ])

    @on_click("confirm")
    async def on_confirm(self):
        self.close(True)

    @on_click("cancel")
    async def on_cancel(self):
        self.close(False)


class MessageScreen(UIScreen[None]):
    """
    Shows a message with a single dismiss button.

    Example:
        await self.show(MessageScreen(
            "Success",
            "Calibration complete!",
            icon="check",
            icon_color="green"
        ))
    """

    def __init__(self, title: str, message: str,
                 button_label: str = "OK",
                 icon_name: Optional[str] = None,
                 icon_color: str = "blue"):
        super().__init__()
        self.title = title
        self.message = message
        self.button_label = button_label
        self.icon_name = icon_name
        self.icon_color = icon_color

    def build(self) -> Widget:
        children = []

        if self.icon_name:
            children.append(Icon(self.icon_name, size=48, color=self.icon_color))
            children.append(Spacer(16))

        children.extend([
            Text(self.message, size="large", align="center"),
            Spacer(32),
            Button("ok", self.button_label, style="primary"),
        ])

        return Center(children=children)

    @on_click("ok")
    async def on_ok(self):
        self.close()


class ChoiceScreen(UIScreen[str]):
    """
    Multiple choice selection. Returns the selected option ID.

    Example:
        choice = await self.show(ChoiceScreen(
            "Select Mode",
            "Choose a driving mode:",
            [
                ("careful", "Careful", "Slow and precise"),
                ("normal", "Normal", "Balanced speed"),
                ("fast", "Fast", "Maximum speed"),
            ]
        ))
    """

    def __init__(self, title: str, message: str,
                 choices: List[tuple],  # [(id, label, description?), ...]
                 cancel_label: Optional[str] = "Cancel"):
        super().__init__()
        self.title = title
        self.message = message
        self.choices = choices
        self.cancel_label = cancel_label

        # Register handlers for each choice
        for choice_id, _, *_ in choices:
            # Create a closure to capture choice_id
            def make_handler(cid):
                async def handler(self_inner=self):
                    self_inner.close(cid)
                return handler

            handler = make_handler(choice_id)
            handler._ui_event = ("click", choice_id)
            self._event_handlers[("click", choice_id)] = lambda s=self, cid=choice_id: s.close(cid)

    def build(self) -> Widget:
        choice_buttons = []

        for item in self.choices:
            choice_id = item[0]
            label = item[1]
            description = item[2] if len(item) > 2 else None

            if description:
                choice_buttons.append(
                    Column(children=[
                        Button(choice_id, label, style="primary"),
                        Text(description, size="small", muted=True, align="center"),
                    ], spacing=4)
                )
            else:
                choice_buttons.append(Button(choice_id, label, style="primary"))

        children = [
            Text(self.message, size="large", align="center"),
            Spacer(24),
            Column(children=choice_buttons, spacing=12),
        ]

        if self.cancel_label:
            children.extend([
                Spacer(16),
                Button("_cancel", self.cancel_label, style="secondary"),
            ])

        return Center(children=children)

    @on_click("_cancel")
    async def on_cancel(self):
        self.close(None)


class ProgressScreen(UIScreen[None]):
    """
    Progress/loading screen for use with non-blocking display.

    Use with `display()` or `showing()` while running background tasks.

    Example:
        async with self.showing(ProgressScreen("Calibrating...")) as ctx:
            for i in range(100):
                ctx.screen.progress = i
                ctx.screen.status = f"Step {i+1}/100"
                await ctx.screen.refresh()
                await asyncio.sleep(0.1)
    """

    title = "Working"

    def __init__(
        self,
        message: str = "Please wait...",
        show_spinner: bool = True,
        show_progress: bool = False,
    ):
        super().__init__()
        self.message = message
        self.show_spinner = show_spinner
        self.show_progress = show_progress
        self.progress: int = 0  # 0-100
        self.status: str = ""  # Optional status text

    def build(self) -> Widget:
        children = []

        if self.show_spinner:
            children.append(ProgressSpinner(size=48))
            children.append(Spacer(24))

        children.append(Text(self.message, size="large", align="center"))

        if self.status:
            children.append(Spacer(8))
            children.append(Text(self.status, size="small", muted=True, align="center"))

        if self.show_progress:
            children.append(Spacer(16))
            children.append(Text(f"{self.progress}%", size="title", bold=True, color="blue"))

        return Center(children=children)


class StatusScreen(UIScreen[None]):
    """
    Simple status display screen for non-blocking use.

    Shows an icon, message, and optional status text.
    Good for showing current state during long operations.

    Example:
        await self.display(StatusScreen(
            "Connecting...",
            icon="wifi",
            icon_color="blue"
        ))
    """

    title = "Status"

    def __init__(
        self,
        message: str,
        icon_name: str = "info",
        icon_color: str = "blue",
        status: str = "",
    ):
        super().__init__()
        self.message = message
        self.icon_name = icon_name
        self.icon_color = icon_color
        self.status = status

    def build(self) -> Widget:
        children = [
            Icon(self.icon_name, size=64, color=self.icon_color),
            Spacer(16),
            Text(self.message, size="large", align="center"),
        ]

        if self.status:
            children.append(Spacer(8))
            children.append(Text(self.status, size="small", muted=True, align="center"))

        return Center(children=children)
