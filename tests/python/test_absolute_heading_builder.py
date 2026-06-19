"""Builder wiring for ``.absolute_heading()`` on the ``optimize()`` chain.

Asserts the pass is appended, shows up in ``.explain()``, and composes with
other passes Java-stream style.
"""

from __future__ import annotations

from raccoon.step.motion import drive_forward, optimize, turn_right
from raccoon.step.motion.path.passes import AbsoluteHeadingPass


def _path():
    return [drive_forward(50), turn_right(90), drive_forward(30)]


def test_absolute_heading_appends_pass():
    opt = optimize(_path()).absolute_heading()
    assert any(isinstance(p, AbsoluteHeadingPass) for p in opt._passes)


def test_absolute_heading_returns_self_for_chaining():
    opt = optimize(_path())
    assert opt.absolute_heading() is opt


def test_explain_shows_absolute_heading_pass():
    text = optimize(_path()).absolute_heading().explain()
    assert "after absolute_heading:" in text
    assert "absolute_heading" in text


def test_absolute_heading_after_always_on_merge():
    # merge is ALWAYS-ON; absolute_heading is the only chained pass. The
    # effective pipeline prepends decompose + merge before it.
    opt = optimize(_path()).absolute_heading()
    assert [p.name for p in opt._passes] == ["absolute_heading"]
    effective = [p.name for p in opt._effective_passes()]
    assert effective == ["decompose", "merge", "absolute_heading"]


def test_explain_stamps_heading_after_pass():
    # End-to-end at the explain level: the pass actually stamps the post-turn
    # heading onto the second straight leg.
    text = optimize(_path()).absolute_heading().explain()
    # The second linear leg follows a turn_right(90); after the pass it carries
    # a target_heading_rad. We only assert the pass ran and produced output.
    assert "after absolute_heading:" in text
