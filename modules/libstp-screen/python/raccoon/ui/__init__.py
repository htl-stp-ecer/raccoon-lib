"""
BotUI - Dynamic UI system for robotics applications.

This module provides a type-safe, event-driven system for creating
UI screens in Python that are rendered by Flutter.

Architecture:
    - UIScreen: Self-contained screens with their own events and state
    - UIStep: Steps that orchestrate which screens to show
    - Widgets: Type-safe dataclasses that serialize to JSON for Flutter

================================================================================
QUICK START - Simple one-liners (recommended for most cases)
================================================================================

    class MyStep(UIStep):
        async def _execute_step(self, robot):
            # Show a message
            await self.message("Calibration complete!")

            # Ask for confirmation
            if await self.confirm("Start calibration?"):
                await self.do_calibration()

            # Get a number
            distance = await self.input_number("Enter distance:", unit="cm")

            # Let user choose
            mode = await self.choose("Select mode:", ["Fast", "Normal", "Precise"])

            # Wait for button press
            await self.wait_for_button("Position robot and press button")

================================================================================
BLOCKING MODE - Show screen, wait for result
================================================================================

    class MyStep(UIStep):
        async def _execute_step(self, robot):
            # Show screen and wait for it to close
            result = await self.show(MyScreen())

================================================================================
NON-BLOCKING MODE - UI + background logic together
================================================================================

    class MyStep(UIStep):
        async def _execute_step(self, robot):
            # Option 1: Context manager (recommended)
            async with self.showing(ProgressScreen()) as ctx:
                for i in range(100):
                    ctx.screen.progress = i
                    await ctx.screen.refresh()
                    await asyncio.sleep(0.1)

            # Option 2: Manual display + pump
            await self.display(StatusScreen("Processing..."))
            for i in range(100):
                await self.pump_events()  # Handle UI events
                await asyncio.sleep(0.1)
            await self.close_ui()

            # Option 3: Run task with UI
            result = await self.run_with_ui(
                LoadingScreen("Calibrating..."),
                self.do_calibration
            )

================================================================================
CUSTOM SCREENS - Full control
================================================================================

    class MyCustomScreen(UIScreen[bool]):
        title = "Custom"

        def __init__(self, message: str):
            super().__init__()
            self.message = message

        def build(self) -> Widget:
            return Center(children=[
                Text(self.message, size="large"),
                Button("ok", "OK", style="success"),
            ])

        @on_click("ok")
        async def on_ok(self):
            self.close(True)
"""

# Widgets
from __future__ import annotations
from .widgets import (
    # Base
    Widget,
    # Display
    Text,
    Icon,
    Spacer,
    Divider,
    StatusBadge,
    StatusIcon,
    HintBox,
    DistanceBadge,
    ResultsTable,
    # Input
    Button,
    Slider,
    Checkbox,
    Dropdown,
    NumericKeypad,
    NumericInput,
    TextInput,
    # Visualization
    SensorValue,
    SensorGraph,
    CalibrationChart,
    LightBulb,
    AnimatedRobot,
    CircularSlider,
    ProgressSpinner,
    PulsingArrow,
    RobotDrivingAnimation,
    MeasuringTape,
    # Layout
    Row,
    Column,
    Center,
    Card,
    Split,
    Expanded,
)

# Events
from .events import (
    on_click,
    on_change,
    on_submit,
    on_keypad,
    on_button_press,
    on_slider,
    on_screen_tap,
)

# Core classes
from .screen import UIScreen
from .step import UIStep

# Pre-built screens
from .screens import (
    # Basic
    WaitForButtonScreen,
    ConfirmScreen,
    MessageScreen,
    ChoiceScreen,
    ProgressScreen,
    StatusScreen,
    # Input
    NumberInputScreen,
    NumberInputResult,
    TextInputScreen,
    TextInputResult,
    SliderInputScreen,
    SliderInputResult,
    # Calibration - WFL
    WFLMeasureScreen,
    WFLConfirmScreen,
    WFLMeasureResult,
    WFLConfirmResult,
    # Calibration - Distance
    DistancePrepareScreen,
    DistanceDrivingScreen,
    DistanceMeasureScreen,
    DistanceConfirmScreen,
    DistanceConfirmResult,
)

__all__ = [
    # Widgets - Display
    "Widget",
    "Text",
    "Icon",
    "Spacer",
    "Divider",
    "StatusBadge",
    "StatusIcon",
    "HintBox",
    "DistanceBadge",
    "ResultsTable",
    # Widgets - Input
    "Button",
    "Slider",
    "Checkbox",
    "Dropdown",
    "NumericKeypad",
    "NumericInput",
    "TextInput",
    # Widgets - Visualization
    "SensorValue",
    "SensorGraph",
    "CalibrationChart",
    "LightBulb",
    "AnimatedRobot",
    "CircularSlider",
    "ProgressSpinner",
    "PulsingArrow",
    "RobotDrivingAnimation",
    "MeasuringTape",
    # Widgets - Layout
    "Row",
    "Column",
    "Center",
    "Card",
    "Split",
    "Expanded",
    # Events
    "on_click",
    "on_change",
    "on_submit",
    "on_keypad",
    "on_button_press",
    "on_slider",
    "on_screen_tap",
    # Core
    "UIScreen",
    "UIStep",
    # Screens - Basic
    "WaitForButtonScreen",
    "ConfirmScreen",
    "MessageScreen",
    "ChoiceScreen",
    "ProgressScreen",
    "StatusScreen",
    # Screens - Input
    "NumberInputScreen",
    "NumberInputResult",
    "TextInputScreen",
    "TextInputResult",
    "SliderInputScreen",
    "SliderInputResult",
    # Screens - Calibration WFL
    "WFLMeasureScreen",
    "WFLConfirmScreen",
    "WFLMeasureResult",
    "WFLConfirmResult",
    # Screens - Calibration Distance
    "DistancePrepareScreen",
    "DistanceDrivingScreen",
    "DistanceMeasureScreen",
    "DistanceConfirmScreen",
    "DistanceConfirmResult",
]
