"""Regression tests for the UIStep stale-answer settle guard.

UI answers travel on a ``reliable`` transport channel; the external renderer
re-sends an un-ACKed answer to whichever subscriber attaches next.  A fresh
process therefore receives an answer left over from a previous screen (or run)
the instant it subscribes — observed ~4 ms after the render — which used to
auto-submit a screen the operator never touched (e.g. the setup mission's
``first_cube_line_gap`` prompt closing itself immediately).

``UIStep._is_stale_answer`` guards against this by discarding renderer answers
that arrive within ``_STALE_ANSWER_GUARD_S`` of a screen first being shown,
while always honouring physical button presses (which come from a separate
local listener, not the reliable channel).

The full ``raccoon.ui.step`` module pulls in compiled/hardware imports, so this
test loads the edited source file in isolation with those dependencies stubbed
— mirroring ``test_screen_messages.py``.
"""

from __future__ import annotations

import importlib.util
import sys
import time
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
STEP_PATH = REPO_ROOT / "modules/libstp-screen/python/raccoon/ui/step.py"


def _stub(name: str, **attrs: object) -> types.ModuleType:
    mod = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(mod, key, value)
    sys.modules[name] = mod
    return mod


def _load_step_module() -> types.ModuleType:
    # Stub the heavy/hardware imports pulled in at module import time.
    _stub("raccoon.button", is_pressed=lambda: False)
    _stub("raccoon.foundation", Subscription=object, get_transport=lambda: None)
    _stub("raccoon.step.base", Step=type("Step", (), {}))
    _stub("raccoon.ui.messages", ScreenRender=object, ScreenRenderAnswer=object)
    import typing

    _ScreenT = typing.TypeVar("_ScreenT")

    class _UIScreen(typing.Generic[_ScreenT]):
        pass

    _stub("raccoon.ui.screen", UIScreen=_UIScreen)
    _stub(
        "raccoon.ui.widgets",
        Button=object,
        Center=object,
        Column=object,
        Row=object,
        Spacer=object,
        Text=object,
    )
    # Provide the parent packages so the relative imports resolve.
    for pkg in ("raccoon", "raccoon.ui", "raccoon.step"):
        if pkg not in sys.modules:
            module = types.ModuleType(pkg)
            module.__path__ = []  # mark as package
            sys.modules[pkg] = module

    spec = importlib.util.spec_from_file_location("raccoon.ui.step", STEP_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["raccoon.ui.step"] = module
    spec.loader.exec_module(module)
    return module


STEP = _load_step_module()
UIStep = STEP.UIStep
GUARD_S = STEP._STALE_ANSWER_GUARD_S


def _make_step(shown_offset_s: float) -> UIStep:
    """A UIStep whose current screen was shown ``shown_offset_s`` ago."""
    step = object.__new__(UIStep)  # bypass __init__ (needs a transport)
    step._screen_shown_monotonic = time.monotonic() - shown_offset_s
    return step


def test_fresh_renderer_answer_is_stale() -> None:
    # A click delivered immediately after the screen was shown is the phantom.
    step = _make_step(shown_offset_s=0.0)
    assert step._is_stale_answer({"_action": "click", "button_id": "submit"}) is True
    assert step._is_stale_answer({"_action": "keypad", "values": {"value": 28.0}}) is True


def test_renderer_answer_after_window_is_honoured() -> None:
    # Once the settle window has elapsed, real input must pass through.
    step = _make_step(shown_offset_s=GUARD_S + 0.5)
    assert step._is_stale_answer({"_action": "click", "button_id": "submit"}) is False


def test_physical_button_press_is_never_stale() -> None:
    # Physical presses come from the local listener, not the reliable channel,
    # so they are honoured even inside the settle window.
    step = _make_step(shown_offset_s=0.0)
    assert step._is_stale_answer({"_action": "button_press"}) is False


def test_guard_window_is_below_human_reaction_time() -> None:
    # The window must eat transport re-sends without blocking real users: it
    # sits well under the seconds it takes to read and answer a screen.
    assert 0.0 < GUARD_S <= 1.5
