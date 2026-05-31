from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MESSAGES_PATH = REPO_ROOT / "modules/libstp-screen/python/raccoon/ui/messages.py"
SPEC = importlib.util.spec_from_file_location("screen_messages", MESSAGES_PATH)
if SPEC is None or SPEC.loader is None:
    msg = f"unable to load screen messages module from {MESSAGES_PATH}"
    raise RuntimeError(msg)
screen_messages = importlib.util.module_from_spec(SPEC)
sys.modules["screen_messages"] = screen_messages
SPEC.loader.exec_module(screen_messages)
ScreenRender = screen_messages.ScreenRender
ScreenRenderAnswer = screen_messages.ScreenRenderAnswer


def test_screen_render_round_trip() -> None:
    original = ScreenRender(
        timestamp=1_717_171_717_123_456,
        screen_name="dynamic_ui",
        entries='{"screen":"Status","title":"Calibration","items":["left","right"]}',
    )

    decoded = ScreenRender.decode(original.encode())

    assert decoded == original


def test_screen_render_answer_round_trip() -> None:
    original = ScreenRenderAnswer(
        timestamp=1_717_171_717_654_321,
        screen_name="dynamic_ui",
        value="submit",
        reason='{"field":"distance","value":42.5,"valid":true}',
    )

    decoded = ScreenRenderAnswer.decode(original.encode())

    assert decoded == original


if __name__ == "__main__":
    test_screen_render_round_trip()
    test_screen_render_answer_round_trip()
    print("screen message round-trip tests passed")
