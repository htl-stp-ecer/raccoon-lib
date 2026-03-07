"""
Input screens for collecting user data.
"""

from dataclasses import dataclass
from typing import Optional

from ..screen import UIScreen
from ..widgets import (
    Widget, Text, Button, Spacer, Slider,
    NumericKeypad, NumericInput, TextInput,
    Center, Row, Column, Split,
)
from ..events import on_click, on_keypad, on_change, on_slider


@dataclass
class NumberInputResult:
    """Result from NumberInputScreen."""
    value: float
    confirmed: bool


@dataclass
class TextInputResult:
    """Result from TextInputScreen."""
    value: str
    confirmed: bool


@dataclass
class SliderInputResult:
    """Result from SliderInputScreen."""
    value: float
    confirmed: bool


class NumberInputScreen(UIScreen[NumberInputResult]):
    """
    Numeric input with touch keypad. Returns NumberInputResult.

    Example:
        result = await self.show(NumberInputScreen(
            "Measurement",
            "Enter measured distance:",
            unit="cm",
            default=30.0
        ))
        if result.confirmed:
            distance = result.value
    """

    _primary_button_id = "submit"

    def __init__(
        self,
        title: str = "Input",
        prompt: str = "Enter value:",
        unit: str = "",
        initial_value: float = 0,
        min_value: Optional[float] = None,
        max_value: Optional[float] = None,
        # Legacy aliases
        label: str = None,
        default: float = None,
    ):
        super().__init__()
        self.title = title
        self.label = label or prompt  # Support both names
        self.unit = unit
        self.value = default if default is not None else initial_value
        self.min_value = min_value
        self.max_value = max_value
        self._input_str = str(self.value) if self.value else ""

    def _clamp_value(self, v: float) -> float:
        """Clamp value to min/max range."""
        if self.min_value is not None:
            v = max(self.min_value, v)
        if self.max_value is not None:
            v = min(self.max_value, v)
        return v

    def build(self) -> Widget:
        return Split(
            left=[
                Text(self.label, size="title"),
                Spacer(24),
                NumericInput(
                    id="value",
                    value=self.value,
                    unit=self.unit,
                    min_value=self.min_value,
                    max_value=self.max_value,
                ),
                Spacer(24),
                Row(children=[
                    Button("cancel", "Cancel", style="secondary"),
                    Button("submit", "Submit", style="success", icon="check"),
                ], spacing=12),
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

        # Parse and clamp
        try:
            self.value = float(self._input_str) if self._input_str else 0
            self.value = self._clamp_value(self.value)
        except ValueError:
            self.value = 0

        await self.refresh()

    @on_change("value")
    async def on_adjust(self, value: float):
        self.value = self._clamp_value(value)
        self._input_str = str(self.value)
        await self.refresh()

    @on_click("submit")
    async def on_submit(self):
        self.close(NumberInputResult(value=self.value, confirmed=True))

    @on_click("cancel")
    async def on_cancel(self):
        self.close(NumberInputResult(value=self.value, confirmed=False))


class TextInputScreen(UIScreen[TextInputResult]):
    """
    Text input screen. Returns TextInputResult.

    Example:
        result = await self.show(TextInputScreen(
            "Name",
            "Enter robot name:",
            default="MyRobot"
        ))
        if result.confirmed:
            name = result.value
    """

    _primary_button_id = "submit"

    def __init__(
        self,
        title: str = "Input",
        prompt: str = "Enter text:",
        default: str = "",
        placeholder: str = "",
        # Legacy alias
        label: str = None,
    ):
        super().__init__()
        self.title = title
        self.label = label or prompt
        self.value = default
        self.placeholder = placeholder

    def build(self) -> Widget:
        return Center(children=[
            Text(self.label, size="large"),
            Spacer(24),
            TextInput(
                id="value",
                value=self.value,
                placeholder=self.placeholder,
            ),
            Spacer(24),
            Row(children=[
                Button("cancel", "Cancel", style="secondary"),
                Button("submit", "Submit", style="success"),
            ], spacing=16),
        ])

    @on_change("value")
    async def on_change_value(self, value: str):
        self.value = value

    @on_click("submit")
    async def on_submit(self):
        self.close(TextInputResult(value=self.value, confirmed=True))

    @on_click("cancel")
    async def on_cancel(self):
        self.close(TextInputResult(value=self.value, confirmed=False))


class SliderInputScreen(UIScreen[SliderInputResult]):
    """
    Slider input screen. Returns SliderInputResult.

    Example:
        result = await self.show(SliderInputScreen(
            "Speed",
            "Select speed:",
            min=0, max=100, default=50
        ))
        if result.confirmed:
            speed = result.value
    """

    _primary_button_id = "submit"

    def __init__(
        self,
        title: str = "Input",
        prompt: str = "Select value:",
        min: float = 0,
        max: float = 100,
        default: float = None,
        # Legacy alias
        label: str = None,
    ):
        super().__init__()
        self.title = title
        self.label = label or prompt
        self.min = min
        self.max = max
        self.value = default if default is not None else min

    def build(self) -> Widget:
        return Center(children=[
            Text(self.label, size="large"),
            Spacer(16),
            Text(f"{self.value:.1f}", size="title", bold=True),
            Spacer(16),
            Slider(
                id="value",
                min=self.min,
                max=self.max,
                value=self.value,
            ),
            Spacer(32),
            Row(children=[
                Button("cancel", "Cancel", style="secondary"),
                Button("submit", "OK", style="success"),
            ], spacing=16),
        ])

    @on_slider("value")
    async def on_slider_change(self, value: float):
        self.value = value
        await self.refresh()

    @on_change("value")
    async def on_change_value(self, value: float):
        self.value = value
        await self.refresh()

    @on_click("submit")
    async def on_submit(self):
        self.close(SliderInputResult(value=self.value, confirmed=True))

    @on_click("cancel")
    async def on_cancel(self):
        self.close(SliderInputResult(value=self.value, confirmed=False))
