"""
UI Events - Decorators for binding methods to UI events.
"""

from typing import Optional, Callable, TypeVar

F = TypeVar('F', bound=Callable)


def on_click(button_id: str) -> Callable[[F], F]:
    """
    Decorator: Called when a button with the given ID is clicked.

    Example:
        @on_click("confirm")
        async def handle_confirm(self):
            self.close(True)
    """
    def decorator(func: F) -> F:
        func._ui_event = ("click", button_id)
        return func
    return decorator


def on_change(widget_id: str) -> Callable[[F], F]:
    """
    Decorator: Called when an input widget's value changes.
    The handler receives the new value as an argument.

    Example:
        @on_change("speed")
        async def handle_speed_change(self, value: float):
            self.speed = value
            await self.refresh()
    """
    def decorator(func: F) -> F:
        func._ui_event = ("change", widget_id)
        return func
    return decorator


def on_submit(widget_id: Optional[str] = None) -> Callable[[F], F]:
    """
    Decorator: Called when a form/input is submitted.

    Example:
        @on_submit()
        async def handle_submit(self):
            self.close(self.value)
    """
    def decorator(func: F) -> F:
        func._ui_event = ("submit", widget_id)
        return func
    return decorator


def on_keypad(key: Optional[str] = None) -> Callable[[F], F]:
    """
    Decorator: Called on numeric keypad input.
    If key is None, handles any key. Handler receives key as argument.

    Keys: "0"-"9", ".", "back"

    Example:
        @on_keypad()
        async def handle_key(self, key: str):
            if key == "back":
                self.input_str = self.input_str[:-1]
            else:
                self.input_str += key
    """
    def decorator(func: F) -> F:
        func._ui_event = ("keypad", key)
        return func
    return decorator


def on_button_press() -> Callable[[F], F]:
    """
    Decorator: Called when the physical robot button is pressed.

    Example:
        @on_button_press()
        async def handle_button(self):
            self.close()
    """
    def decorator(func: F) -> F:
        func._ui_event = ("button_press", None)
        return func
    return decorator


def on_slider(slider_id: str) -> Callable[[F], F]:
    """
    Decorator: Called when a slider value changes.
    Handler receives the new value as argument.

    Example:
        @on_slider("motor_power")
        async def handle_power(self, value: float):
            self.power = value
    """
    def decorator(func: F) -> F:
        func._ui_event = ("slider", slider_id)
        return func
    return decorator


def on_screen_tap() -> Callable[[F], F]:
    """
    Decorator: Called when the screen is tapped anywhere.

    Example:
        @on_screen_tap()
        async def handle_tap(self):
            self.close()
    """
    def decorator(func: F) -> F:
        func._ui_event = ("screen_tap", None)
        return func
    return decorator
