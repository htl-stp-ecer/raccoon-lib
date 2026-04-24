"""
UIStep - Base class for Steps that show UI screens.

The Step orchestrates the flow, screens handle their own UI/events.
"""

from __future__ import annotations
from abc import ABC
from typing import Optional, TypeVar, TYPE_CHECKING, Callable, Awaitable, Any, List, Union
from contextlib import asynccontextmanager
from contextvars import ContextVar
import asyncio
import json

import time
from raccoon_transport import Transport


class _SetupTimerState:
    """Mutable state for the active setup-phase countdown.

    Stored in a ContextVar so that every UIStep rendered during a
    SetupMission (including the WFL gate) can read the true remaining
    time without requiring per-second LCM messages.

    Pausing shifts ``start_monotonic`` forward on resume so elapsed time
    genuinely freezes during the pause rather than merely hiding it in
    the UI.
    """
    __slots__ = ("duration", "start_monotonic", "paused", "_pause_start")

    def __init__(self, duration: int) -> None:
        self.duration = duration
        self.start_monotonic = time.monotonic()
        self.paused = False
        self._pause_start = 0.0

    @property
    def remaining(self) -> int:
        """Remaining seconds — negative once overtime begins."""
        now = self._pause_start if self.paused else time.monotonic()
        elapsed = now - self.start_monotonic
        return round(self.duration - elapsed)

    def pause(self) -> None:
        if not self.paused:
            self._pause_start = time.monotonic()
            self.paused = True

    def resume(self) -> None:
        if self.paused:
            self.start_monotonic += time.monotonic() - self._pause_start
            self.paused = False

    def reset(self) -> None:
        """Restart the countdown from full duration and unpause."""
        self.start_monotonic = time.monotonic()
        self.paused = False


# Set by SetupMission.setup_timer_context(); None outside a setup phase.
_active_setup_timer: ContextVar[Optional[_SetupTimerState]] = ContextVar(
    "active_setup_timer", default=None
)


def set_setup_timer_paused(paused: bool) -> None:
    """Pause or resume the setup-timer countdown.

    No-op when called outside a SetupMission (e.g. in a regular mission
    or in tests).  Used by WaitForLight to freeze the timer while armed.
    """
    state = _active_setup_timer.get()
    if state is None:
        return
    if paused:
        state.pause()
    else:
        state.resume()


def reset_setup_timer() -> None:
    """Restart the setup-timer countdown from its full duration.

    No-op outside a SetupMission.
    """
    state = _active_setup_timer.get()
    if state is not None:
        state.reset()
from raccoon.step.base import Step
from raccoon.button import is_pressed
from .raccoon.screen_render_t import screen_render_t
from .raccoon.screen_render_answer_t import screen_render_answer_t

from .screen import UIScreen
from .widgets import Text, Button, Center, Column, Row, Spacer

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

T = TypeVar('T')


class UIStep(Step, ABC):
    """
    Base class for Steps that show UI screens.

    The Step is responsible for orchestrating the flow by showing
    screens and collecting their results. Each screen handles its
    own events and state.

    Example:
        class MyCalibration(UIStep):
            async def _execute_step(self, robot):
                # Show first screen, wait for result
                off_value = await self.show(MeasureScreen(is_on=False))

                # Show second screen
                on_value = await self.show(MeasureScreen(is_on=True))

                # Show confirmation
                result = await self.show(ConfirmScreen(off_value, on_value))

                if not result.confirmed:
                    # Retry
                    return await self._execute_step(robot)

                self.result = result
    """

    # LCM channel name for dynamic UI
    _SCREEN_NAME = "dynamic_ui"

    def __init__(self):
        super().__init__()
        self._robot: Optional[GenericRobot] = None
        self._current_screen: Optional[UIScreen] = None
        self._transport = Transport()
        self._lcm_pump_task: Optional[asyncio.Task] = None
        self._ui_active: bool = False  # Track if any UI has been shown
        # Persistent subscription state for non-blocking display/pump_events
        self._pump_queue: Optional[asyncio.Queue] = None
        self._pump_sub = None

    async def show(self, screen: UIScreen[T]) -> T:
        """
        Show a screen and wait for it to close.

        The screen's event handlers will be called as the user interacts.
        When the screen calls close(result), this method returns that result.

        Args:
            screen: The UIScreen instance to display

        Returns:
            Whatever the screen passed to close()
        """
        # Set up screen
        screen._step = self
        screen._closed = False
        screen._result = None
        self._current_screen = screen

        self.debug(f"Showing screen: {screen.__class__.__name__}")

        # Initial render
        await self._render_screen(screen)

        # Run event loop for this screen
        await self._run_screen_events(screen)

        # Cleanup
        self._current_screen = None

        return screen._result

    async def _render_screen(self, screen: UIScreen) -> None:
        """Send screen to Flutter via LCM."""
        msg = screen_render_t()
        msg.timestamp = int(time.time() * 1_000_000)
        msg.screen_name = self._SCREEN_NAME

        timer_state = _active_setup_timer.get()
        if timer_state is not None and timer_state.duration > 0:
            setup_timer = {"seconds": timer_state.remaining, "paused": timer_state.paused}
        else:
            setup_timer = None

        msg.entries = json.dumps(screen._to_dict(setup_timer=setup_timer))
        self.debug(f"[LCM TX] screen_render -> {screen.__class__.__name__}: {screen.title}")
        self._transport.publish("raccoon/screen_render", msg)
        self._ui_active = True

    async def _run_screen_events(self, screen: UIScreen) -> None:
        """Run event loop until screen closes."""
        loop = asyncio.get_event_loop()
        event_queue: asyncio.Queue = asyncio.Queue()

        # LCM message handler — receives unwrapped payload from reliable envelope
        def lcm_handler(channel, data):
            msg = screen_render_answer_t.decode(data)
            if msg.screen_name == self._SCREEN_NAME:
                try:
                    event = json.loads(msg.reason) if msg.reason else {}
                except json.JSONDecodeError:
                    event = {}
                event["_action"] = msg.value
                self.debug(f"[LCM RX] screen_answer <- action={msg.value}, data={msg.reason[:100] if msg.reason else 'none'}...")
                loop.call_soon_threadsafe(event_queue.put_nowait, event)

        # Subscribe with reliable=True to receive Flutter's reliable-published answers
        sub = self._transport.subscribe(
            "raccoon/screen_render/answer", lcm_handler, reliable=True,
        )

        # Start button listener task
        button_task = asyncio.create_task(self._button_listener(event_queue))

        try:
            while not screen._closed:
                # Handle LCM messages (includes reliable ACK handling)
                self._transport.spin_once(timeout_ms=0)

                # Process events from queue
                try:
                    event = event_queue.get_nowait()
                    await screen._dispatch_event(event)
                except asyncio.QueueEmpty:
                    pass

                await asyncio.sleep(0.01)
        finally:
            button_task.cancel()
            try:
                await button_task
            except asyncio.CancelledError:
                pass
            self._transport._lcm.unsubscribe(sub)

    async def _button_listener(self, queue: asyncio.Queue) -> None:
        """Listen for physical button presses."""
        button_sensor = getattr(getattr(self._robot, "defs", None), "button", None)

        def _pressed() -> bool:
            if button_sensor is not None:
                return bool(button_sensor.read())
            return is_pressed()

        was_pressed = _pressed()

        while True:
            pressed = _pressed()

            # Detect rising edge (button just pressed)
            if pressed and not was_pressed:
                queue.put_nowait({"_action": "button_press"})

            was_pressed = pressed
            await asyncio.sleep(0.02)

    async def close_ui(self) -> None:
        """Close any active screen."""
        if not self._ui_active:
            return
        self._teardown_pump_sub()
        msg = screen_render_t()
        msg.timestamp = int(time.time() * 1_000_000)
        msg.screen_name = self._SCREEN_NAME
        msg.entries = json.dumps({"screen": "close"})
        self.debug("[LCM TX] screen_render -> close")
        self._transport.publish("raccoon/screen_render", msg)
        self._current_screen = None
        self._ui_active = False

    async def run_step(self, robot: GenericRobot) -> None:
        """
        Run the step with automatic UI cleanup.

        Ensures the UI is closed even if the step fails or is cancelled.
        """
        self._robot = robot

        try:
            await super().run_step(robot)
        finally:
            # Always close UI on exit (normal or exception)
            await self._force_close_ui()

    async def _force_close_ui(self) -> None:
        """Force close any active UI, swallowing errors to ensure cleanup."""
        try:
            await self.close_ui()
        except Exception as e:
            # Log but don't raise - we're in cleanup
            self.warn(f"Error closing UI during cleanup: {e}")

    # =========================================================================
    # NON-BLOCKING UI - Background code + UI together
    # =========================================================================

    async def display(self, screen: UIScreen) -> None:
        """
        Show a screen without waiting for it to close.

        Use this when you want to run background logic while the UI is visible.
        Events will still be dispatched to the screen's handlers.

        Example:
            await self.display(StatusScreen("Processing..."))
            for i in range(100):
                await self.update_progress(i)
                await asyncio.sleep(0.1)
            await self.close_ui()
        """
        # Tear down any previous persistent subscription
        self._teardown_pump_sub()

        screen._step = self
        screen._closed = False
        screen._result = None
        self._current_screen = screen
        self.debug(f"Displaying screen (non-blocking): {screen.__class__.__name__}")
        await self._render_screen(screen)

        # Set up persistent LCM subscription so events are buffered between
        # pump_events() calls instead of being silently dropped.
        loop = asyncio.get_event_loop()
        self._pump_queue = asyncio.Queue()
        queue = self._pump_queue  # capture for closure

        def lcm_handler(channel, data):
            msg = screen_render_answer_t.decode(data)
            if msg.screen_name == self._SCREEN_NAME:
                try:
                    event = json.loads(msg.reason) if msg.reason else {}
                except json.JSONDecodeError:
                    event = {}
                event["_action"] = msg.value
                self.debug(f"[LCM RX] pump <- action={msg.value}")
                loop.call_soon_threadsafe(queue.put_nowait, event)

        self._pump_sub = self._transport.subscribe(
            "raccoon/screen_render/answer", lcm_handler, reliable=True,
        )

    def _teardown_pump_sub(self) -> None:
        """Unsubscribe the persistent pump subscription if active."""
        if self._pump_sub is not None:
            self._transport._lcm.unsubscribe(self._pump_sub)
            self._pump_sub = None
        self._pump_queue = None

    async def pump_events(self, timeout: float = 0) -> None:
        """
        Process pending UI events without blocking.

        Call this periodically in your background loop to handle user input.

        Example:
            await self.display(MyScreen())
            while running:
                await self.pump_events()
                # ... do background work ...
                await asyncio.sleep(0.05)
        """
        if not self._current_screen or self._pump_queue is None:
            return

        screen = self._current_screen

        # Deliver any pending LCM messages into the queue
        self._transport.spin_once(timeout_ms=int(timeout * 1000))
        # Yield so that call_soon_threadsafe callbacks run
        await asyncio.sleep(0)

        while True:
            try:
                event = self._pump_queue.get_nowait()
                await screen._dispatch_event(event)
            except asyncio.QueueEmpty:
                break

    @asynccontextmanager
    async def showing(self, screen: UIScreen[T]):
        """
        Context manager for displaying a screen while running code.

        Events are automatically pumped. Screen closes when exiting context.

        Example:
            async with self.showing(ProgressScreen()) as ctx:
                for i in range(100):
                    ctx.screen.progress = i
                    await ctx.screen.refresh()
                    await asyncio.sleep(0.1)
        """
        await self.display(screen)
        try:
            yield _ShowingContext(screen, self)
        finally:
            if not screen._closed:
                await self.close_ui()

    async def run_with_ui(
        self,
        screen: UIScreen,
        task: Union[Callable[[], Awaitable[Any]], Awaitable[Any]],
        poll_interval: float = 0.05,
    ) -> Any:
        """
        Run a background task while showing a screen.

        Events are pumped automatically. Returns when task completes.

        Args:
            screen: The screen to display while task runs
            task: Either a coroutine or a callable that returns a coroutine
            poll_interval: How often to pump UI events (seconds)

        Example:
            # With callable:
            result = await self.run_with_ui(
                LoadingScreen("Calibrating..."),
                self.do_calibration
            )

            # With coroutine directly:
            result = await self.run_with_ui(
                LoadingScreen("Calibrating..."),
                self.do_calibration()
            )
        """
        await self.display(screen)

        # Accept both coroutines and callables
        if asyncio.iscoroutine(task):
            coro = task
        elif callable(task):
            coro = task()
        else:
            raise TypeError(f"task must be a coroutine or callable, got {type(task)}")

        task_obj = asyncio.create_task(coro)

        try:
            while not task_obj.done() and not screen._closed:
                await self.pump_events()
                await asyncio.sleep(poll_interval)
            return await task_obj
        finally:
            if not screen._closed:
                await self.close_ui()

    # =========================================================================
    # QUICK HELPERS - One-liners for common UI patterns
    # =========================================================================

    async def message(
        self,
        text: str,
        title: str = "Info",
        button_label: str = "OK",
    ) -> None:
        """
        Show a simple message and wait for acknowledgment.

        Example:
            await self.message("Calibration complete!")
            await self.message("Error occurred", title="Error")
        """
        screen = _QuickMessageScreen(text, title, button_label)
        await self.show(screen)

    async def confirm(
        self,
        text: str,
        title: str = "Confirm",
        yes_label: str = "Yes",
        no_label: str = "No",
    ) -> bool:
        """
        Show a yes/no confirmation dialog.

        Example:
            if await self.confirm("Start calibration?"):
                await self.do_calibration()
        """
        screen = _QuickConfirmScreen(text, title, yes_label, no_label)
        return await self.show(screen)

    async def input_number(
        self,
        prompt: str,
        title: str = "Input",
        default: float = 0,
        unit: str = "",
        min_value: Optional[float] = None,
        max_value: Optional[float] = None,
    ) -> Optional[float]:
        """
        Get a numeric value from the user.

        Returns None if cancelled.

        Example:
            distance = await self.input_number("Enter distance:", unit="cm")
            speed = await self.input_number("Speed:", min_value=0, max_value=100)
        """
        from .screens.input import NumberInputScreen
        screen = NumberInputScreen(
            title=title,
            prompt=prompt,
            initial_value=default,
            unit=unit,
            min_value=min_value,
            max_value=max_value,
        )
        result = await self.show(screen)
        return result.value if result and result.confirmed else None

    async def choose(
        self,
        prompt: str,
        options: List[str],
        title: str = "Choose",
    ) -> Optional[str]:
        """
        Let user choose from a list of options.

        Returns None if cancelled.

        Example:
            mode = await self.choose("Select mode:", ["Fast", "Normal", "Precise"])
        """
        screen = _QuickChoiceScreen(prompt, options, title)
        return await self.show(screen)

    async def wait_for_button(self, text: str = "Press button to continue", title: str = "Ready") -> None:
        """
        Wait for physical button press.

        Example:
            await self.wait_for_button("Position robot and press button")
        """
        screen = _QuickWaitButtonScreen(text, title)
        await self.show(screen)

    async def input_text(
        self,
        prompt: str,
        title: str = "Input",
        default: str = "",
        placeholder: str = "",
    ) -> Optional[str]:
        """
        Get text input from the user.

        Returns None if cancelled.

        Example:
            name = await self.input_text("Enter robot name:")
        """
        from .screens.input import TextInputScreen
        screen = TextInputScreen(
            title=title,
            prompt=prompt,
            default=default,
            placeholder=placeholder,
        )
        result = await self.show(screen)
        return result.value if result and result.confirmed else None

    async def input_slider(
        self,
        prompt: str,
        min: float,
        max: float,
        title: str = "Input",
        default: float = None,
    ) -> Optional[float]:
        """
        Get a value using a slider.

        Returns None if cancelled.

        Example:
            speed = await self.input_slider("Select speed:", min=0, max=100)
        """
        from .screens.input import SliderInputScreen
        screen = SliderInputScreen(
            title=title,
            prompt=prompt,
            min=min,
            max=max,
            default=default,
        )
        result = await self.show(screen)
        return result.value if result and result.confirmed else None


class _ShowingContext:
    """Context for `showing()` context manager."""
    def __init__(self, screen: UIScreen, step: UIStep):
        self.screen = screen
        self._step = step

    async def pump(self) -> None:
        """Pump events manually."""
        await self._step.pump_events()


# =============================================================================
# QUICK SCREEN IMPLEMENTATIONS
# =============================================================================

class _QuickMessageScreen(UIScreen[None]):
    """Simple message screen."""

    _primary_button_id = "ok"

    def __init__(self, text: str, title: str, button_label: str):
        super().__init__()
        self.title = title
        self._text = text
        self._button_label = button_label

    def build(self):
        return Center(children=[
            Column(children=[
                Text(self._text, size="large", align="center"),
                Spacer(height=24),
                Button("ok", self._button_label, style="primary"),
            ], align="center"),
        ])

    async def _dispatch_event(self, event: dict) -> None:
        await super()._dispatch_event(event)
        action = event.get("_action")
        if action == "click" and event.get("button_id") == "ok":
            self.close()
        elif action == "button_press":
            self.close()


class _QuickConfirmScreen(UIScreen[bool]):
    """Yes/No confirmation screen."""

    _primary_button_id = "yes"

    def __init__(self, text: str, title: str, yes_label: str, no_label: str):
        super().__init__()
        self.title = title
        self._text = text
        self._yes_label = yes_label
        self._no_label = no_label

    def build(self):
        return Center(children=[
            Column(children=[
                Text(self._text, size="large", align="center"),
                Spacer(height=24),
                Row(children=[
                    Button("no", self._no_label, style="secondary"),
                    Button("yes", self._yes_label, style="success"),
                ], align="center", spacing=16),
            ], align="center"),
        ])

    async def _dispatch_event(self, event: dict) -> None:
        await super()._dispatch_event(event)
        action = event.get("_action")
        if action == "click":
            btn = event.get("button_id")
            if btn == "yes":
                self.close(True)
            elif btn == "no":
                self.close(False)
        elif action == "button_press":
            self.close(True)


class _QuickChoiceScreen(UIScreen[Optional[str]]):
    """Choice selection screen."""

    def __init__(self, prompt: str, options: List[str], title: str):
        super().__init__()
        self.title = title
        self._prompt = prompt
        self._options = options

    def build(self):
        buttons = [
            Button(f"opt_{i}", opt, style="primary")
            for i, opt in enumerate(self._options)
        ]
        return Center(children=[
            Column(children=[
                Text(self._prompt, size="large", align="center"),
                Spacer(height=16),
                Column(children=buttons, spacing=8),
                Spacer(height=16),
                Button("cancel", "Cancel", style="secondary"),
            ], align="center"),
        ])

    async def _dispatch_event(self, event: dict) -> None:
        await super()._dispatch_event(event)
        if event.get("_action") == "click":
            btn = event.get("button_id", "")
            if btn == "cancel":
                self.close(None)
            elif btn.startswith("opt_"):
                idx = int(btn.split("_")[1])
                self.close(self._options[idx])


class _QuickWaitButtonScreen(UIScreen[None]):
    """Wait for physical button press."""

    def __init__(self, text: str, title: str):
        super().__init__()
        self.title = title
        self._text = text

    def build(self):
        from .widgets import HintBox
        return Center(children=[
            Column(children=[
                Text(self._text, size="large", align="center"),
                Spacer(height=24),
                HintBox("Press the button to continue", icon="touch_app"),
            ], align="center"),
        ])

    async def _dispatch_event(self, event: dict) -> None:
        await super()._dispatch_event(event)
        if event.get("_action") == "button_press":
            self.close()
