"""Tests for the absolute-plan compiler (Phase 3, Commit B).

Covers the **dummes Desugaring 1:1** contract from
``docs/design/absolute-motion-plan.md``: the compiler validates node types,
wraps them into a frozen :class:`CompiledAbsolutePlan`, and otherwise
leaves the sequence untouched. Optimizer passes are Phase 5 and are
explicitly *not* exercised here.

Snapshot strategy: inline asserts on individual node fields plus a
deterministic ``repr`` snapshot for one representative mission. No
file-based snapshots — tests stay self-contained.
"""

from __future__ import annotations

import math

import pytest

from raccoon.step.motion.path import (
    Action,
    CompiledAbsolutePlan,
    Goto,
    Resync,
    TurnTo,
    action,
    compile_plan,
    goto,
    resync,
    turn_to,
)

# --- helpers -----------------------------------------------------------------


class _MockStep:
    """Stand-in for a Step instance — the compiler never inspects it."""


# --- Test 1: pure goto chain --------------------------------------------------


def test_compile_pure_goto_chain_preserves_order_and_fields() -> None:
    plan = compile_plan([goto(80, 0), goto(80, 50)])

    assert isinstance(plan, CompiledAbsolutePlan)
    assert len(plan.nodes) == 2

    first, second = plan.nodes
    assert isinstance(first, Goto)
    assert first.x_m == pytest.approx(0.80)
    assert first.y_m == pytest.approx(0.00)
    assert first.theta_rad is None

    assert isinstance(second, Goto)
    assert second.x_m == pytest.approx(0.80)
    assert second.y_m == pytest.approx(0.50)


# --- Test 2: mixed goto / turn_to / goto -------------------------------------


def test_compile_goto_turn_goto_keeps_sequence() -> None:
    plan = compile_plan([goto(80, 0), turn_to(90), goto(80, 50)])

    assert len(plan.nodes) == 3
    a, b, c = plan.nodes
    assert isinstance(a, Goto)
    assert isinstance(b, TurnTo)
    assert isinstance(c, Goto)
    # Phase 3 is dumb: TurnTo stays in the middle, no folding.
    assert b.theta_rad == pytest.approx(math.pi / 2.0)


# --- Test 3: action(blocking=False) is preserved verbatim --------------------


def test_compile_action_non_blocking_stays_in_place() -> None:
    step = _MockStep()
    plan = compile_plan(
        [
            goto(80, 0),
            action(step, blocking=False),
            goto(80, 50),
        ]
    )

    assert len(plan.nodes) == 3
    middle = plan.nodes[1]
    assert isinstance(middle, Action)
    assert middle.step is step
    assert middle.blocking is False


def test_compile_resync_passes_through() -> None:
    plan = compile_plan(
        [
            resync("io_button", expected_x_cm=10, expected_y_cm=10),
            goto(80, 0),
        ]
    )

    assert len(plan.nodes) == 2
    assert isinstance(plan.nodes[0], Resync)
    assert plan.nodes[0].method == "io_button"


# --- Test 4: type errors -----------------------------------------------------


def test_compile_rejects_int_node() -> None:
    with pytest.raises(TypeError, match="index 0"):
        compile_plan([42])  # type: ignore[list-item]


def test_compile_rejects_string_node() -> None:
    with pytest.raises(TypeError, match="expected one of"):
        compile_plan(["not-a-node"])  # type: ignore[list-item]


def test_compile_rejects_bad_node_in_middle() -> None:
    with pytest.raises(TypeError, match="index 1"):
        compile_plan([goto(0, 0), object(), goto(1, 1)])  # type: ignore[list-item]


# --- Test 5: frozenness / immutability ---------------------------------------


def test_compiled_plan_nodes_is_tuple() -> None:
    plan = compile_plan([goto(80, 0)])
    assert isinstance(plan.nodes, tuple)
    assert not isinstance(plan.nodes, list)


def test_compiled_plan_passes_applied_empty_in_phase_3() -> None:
    plan = compile_plan([goto(80, 0)])
    assert plan.passes_applied == ()
    assert isinstance(plan.passes_applied, tuple)


def test_compiled_plan_is_frozen() -> None:
    plan = compile_plan([goto(80, 0)])
    with pytest.raises(Exception):  # FrozenInstanceError subclasses Exception
        plan.nodes = ()  # type: ignore[misc]


def test_compiled_plan_input_list_mutation_does_not_leak() -> None:
    inputs = [goto(80, 0)]
    plan = compile_plan(inputs)
    inputs.append(goto(80, 50))
    # The compiled plan stored a tuple snapshot; later mutation of the
    # caller's list must not affect it.
    assert len(plan.nodes) == 1


# --- Test 6: empty input -----------------------------------------------------


def test_compile_empty_sequence_yields_empty_plan() -> None:
    plan = compile_plan([])
    assert isinstance(plan, CompiledAbsolutePlan)
    assert plan.nodes == ()
    assert plan.passes_applied == ()


# --- repr snapshot -----------------------------------------------------------


def test_compiled_plan_repr_is_deterministic() -> None:
    """One repr snapshot — guards against accidental format drift."""
    empty = compile_plan([])
    assert repr(empty) == "CompiledAbsolutePlan(nodes=(), passes_applied=())"

    plan = compile_plan([goto(80, 0), turn_to(90)])
    text = repr(plan)
    # Sanity: starts with header, contains both nodes in order.
    assert text.startswith("CompiledAbsolutePlan(")
    assert text.index("Goto(") < text.index("TurnTo(")
    assert "passes_applied=()" in text
