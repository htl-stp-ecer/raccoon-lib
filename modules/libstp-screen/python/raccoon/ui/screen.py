"""
UIScreen - Base class for self-contained UI screens.

Each screen handles its own events and state.
The UIStep orchestrates which screens to show.
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Optional, TypeVar, Generic, TYPE_CHECKING, Dict, Any

from .widgets import Widget
from raccoon.class_name_logger import ClassNameLogger

if TYPE_CHECKING:
    from .step import UIStep


T = TypeVar('T')  # Result type


class UIScreen(ABC, Generic[T], ClassNameLogger):
    """
    A self-contained screen with its own layout and event handlers.

    Subclass this to create reusable screen components. The screen
    runs until close() is called, then returns a result to the Step.

    Example:
        class ConfirmScreen(UIScreen[bool]):
            title = "Confirm"

            def __init__(self, message: str):
                super().__init__()
                self.message = message

            def build(self) -> Widget:
                return Center(children=[
                    Text(self.message, size="large"),
                    Row(children=[
                        Button("no", "Cancel", style="secondary"),
                        Button("yes", "Confirm", style="success"),
                    ]),
                ])

            @on_click("yes")
            async def on_yes(self):
                self.close(True)

            @on_click("no")
            async def on_no(self):
                self.close(False)
    """

    # Override in subclass
    title: str = "Screen"
    _primary_button_id: Optional[str] = None  # Physical button triggers this button's click

    def __init__(self):
        super().__init__()
        self._closed = False
        self._result: Optional[T] = None
        self._step: Optional[UIStep] = None
        self._values: Dict[str, Any] = {}  # Current input values from UI
        self._event_handlers: Dict[tuple, Any] = {}

        # Collect decorated event handlers
        for name in dir(self.__class__):
            try:
                method = getattr(self, name)
                if callable(method) and hasattr(method, "_ui_event"):
                    event_type, event_id = method._ui_event
                    self._event_handlers[(event_type, event_id)] = method
            except AttributeError:
                continue

    @abstractmethod
    def build(self) -> Widget:
        """
        Build the screen layout.

        Called on every render. Return a Widget tree describing
        what to display.
        """
        raise NotImplementedError

    def close(self, result: T = None) -> None:
        """
        Close this screen and return to the Step.

        Args:
            result: The value to return to the Step. Type should match
                   the Generic type parameter.
        """
        self._result = result
        self._closed = True
        self.debug(f"Screen closing with result: {result}")

    async def refresh(self) -> None:
        """
        Re-render this screen.

        Call this after changing state that affects the UI.
        """
        if self._step:
            await self._step._render_screen(self)

    def get_value(self, widget_id: str, default: Any = None) -> Any:
        """
        Get the current value of an input widget.

        Args:
            widget_id: The ID of the input widget
            default: Value to return if widget not found

        Returns:
            The current value of the widget
        """
        return self._values.get(widget_id, default)

    def set_value(self, widget_id: str, value: Any) -> None:
        """
        Set the value of an input widget.

        Args:
            widget_id: The ID of the input widget
            value: The new value
        """
        self._values[widget_id] = value

    # --- Convenience Properties ---

    @property
    def robot(self):
        """Access the robot instance."""
        return self._step._robot if self._step else None

    @property
    def is_closed(self) -> bool:
        """Check if screen is closed."""
        return self._closed

    # --- Sensor Helpers ---

    async def read_sensor(self, port: int, sensor_type: str = "analog") -> float:
        """
        Read a sensor value.

        Args:
            port: The sensor port number
            sensor_type: "analog" or "digital"

        Returns:
            The sensor reading
        """
        if not self.robot:
            self.warn("No robot available for sensor reading")
            return 0.0

        if sensor_type == "analog":
            return self.robot.sensors.analog(port).read()
        else:
            return self.robot.sensors.digital(port).read()

    # --- Internal Methods ---

    def _to_dict(self) -> dict:
        """Serialize screen to JSON for LCM."""
        return {
            "screen": "dynamic",
            "title": self.title,
            "body": self.build().to_dict(),
        }

    async def _dispatch_event(self, event: dict) -> None:
        """Dispatch an event to the appropriate handler."""
        action = event.get("_action")

        # Update stored values
        if "values" in event:
            self._values.update(event["values"])

        # Determine event type and find handler
        handler = None
        handler_args = []

        if action == "button_press":
            handler = self._event_handlers.get(("button_press", None))
            # Fallback: trigger primary button click if no explicit handler
            if not handler and self._primary_button_id:
                handler = self._event_handlers.get(("click", self._primary_button_id))
                if handler:
                    self.debug(f"Physical button -> primary button '{self._primary_button_id}'")

        elif action == "click":
            button_id = event.get("button_id")
            handler = self._event_handlers.get(("click", button_id))

        elif action == "change":
            widget_id = event.get("widget_id")
            value = event.get("value")
            self._values[widget_id] = value
            handler = self._event_handlers.get(("change", widget_id))
            if handler:
                handler_args = [value]

        elif action == "keypad":
            key = event.get("key")
            # Try specific key handler first, then wildcard
            handler = self._event_handlers.get(("keypad", key))
            if not handler:
                handler = self._event_handlers.get(("keypad", None))
            if handler:
                handler_args = [key]

        elif action == "slider":
            slider_id = event.get("widget_id")
            value = event.get("value")
            self._values[slider_id] = value
            handler = self._event_handlers.get(("slider", slider_id))
            if handler:
                handler_args = [value]

        elif action == "submit":
            widget_id = event.get("widget_id")
            handler = self._event_handlers.get(("submit", widget_id))
            if not handler:
                handler = self._event_handlers.get(("submit", None))

        elif action == "screen_tap":
            handler = self._event_handlers.get(("screen_tap", None))

        # Call handler if found
        if handler:
            self.debug(f"Dispatching {action} to {handler.__name__}")
            await handler(*handler_args)
        else:
            self.debug(f"No handler for event: {action}")
