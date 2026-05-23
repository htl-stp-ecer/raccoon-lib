"""Toggle the firmware's SpeedMode (BEMF on/off) and synchronize the library flag.

SpeedMode disables the STM32's BEMF closed-loop control. The firmware
no longer measures wheel velocity and integrates encoder ticks, so the
robot gains roughly 10% top speed at the cost of cm-accurate
distance and angle termination. While SpeedMode is on every motion
primitive must terminate via an explicit ``until``-condition; using a
distance- or angle-based goal raises ``std::logic_error`` from the
C++ controller.

This step is the only sanctioned way to flip the flag — it owns the
LCM handshake with ``stm32-data-reader``:

1. Publish ``raccoon::scalar_i32_t(value=1|0)`` on
   ``raccoon/cmd/feature/bemf_enabled`` (1 = BEMF on / normal mode,
   0 = BEMF off / SpeedMode).
2. Wait for the retained ACK on ``raccoon/feature/bemf_enabled``.
3. On ACK, update ``raccoon.foundation.set_speed_mode_enabled`` so
   the motion runtime sees the new state.
4. On timeout, raise ``RuntimeError`` — the firmware did not confirm
   the change and the library state must not diverge from the
   hardware state.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from raccoon.foundation import get_transport, set_speed_mode_enabled
from raccoon_transport.types.raccoon import scalar_i32_t

from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


_CMD_CHANNEL = "raccoon/cmd/feature/bemf_enabled"
_ACK_CHANNEL = "raccoon/feature/bemf_enabled"
_DEFAULT_ACK_TIMEOUT_S = 0.5


@dsl_step(tags=["motion", "feature", "speed-mode"])
class SetSpeedMode(Step):
    """Enable or disable SpeedMode (BEMF off / on) via an LCM handshake.

    SpeedMode disables the firmware's BEMF closed-loop measurements,
    granting ~10% additional top speed in exchange for cm precision.
    Once SpeedMode is active, every motion that has a distance- or
    angle-based goal will refuse to start — use ``until``-conditions
    (sensors, timeouts, light gates, ...) to terminate motions
    instead. Returning to normal mode restores BEMF and cm accuracy.

    The step publishes the command, waits for the firmware's ACK on
    the retained ``raccoon/feature/bemf_enabled`` channel, and only
    then updates the library's ``SpeedModeContext`` flag. If no ACK
    arrives within the timeout it raises so the program does not
    proceed with library / firmware state out of sync.

    Args:
        enabled: ``True`` to engage SpeedMode (BEMF off, 10% faster,
            no cm precision); ``False`` to return to normal closed-loop
            BEMF control.
        timeout_s: Maximum time in seconds to wait for the firmware
            ACK before raising ``RuntimeError``. Defaults to 0.5 s.

    Raises:
        RuntimeError: If the firmware does not acknowledge the
            command within ``timeout_s``.

    Example::

        from raccoon.step.motion import set_speed_mode, drive_forward
        from raccoon.step.sensor import on_black

        set_speed_mode(True)
        drive_forward(speed=1.0).until(on_black(line_sensor))
        set_speed_mode(False)
    """

    def __init__(self, enabled: bool, timeout_s: float = _DEFAULT_ACK_TIMEOUT_S) -> None:
        super().__init__()
        if not isinstance(enabled, bool):
            msg = f"enabled must be a bool, got {type(enabled).__name__}"
            raise TypeError(msg)
        if not isinstance(timeout_s, int | float):
            msg = f"timeout_s must be a number, got {type(timeout_s).__name__}"
            raise TypeError(msg)
        if timeout_s <= 0:
            msg = f"timeout_s must be > 0, got {timeout_s}"
            raise ValueError(msg)
        self._enabled = enabled
        self._timeout_s = float(timeout_s)

    def _generate_signature(self) -> str:
        mode = "on" if self._enabled else "off"
        return f"SetSpeedMode(speed_mode={mode}, timeout={self._timeout_s:.2f}s)"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        # The ACK payload uses the same scalar_i32_t encoding as the
        # command. We treat any matching value as a successful ACK and
        # reconcile the library flag from it (the firmware is the source
        # of truth — if it reports a different value than what we asked
        # for, we sync to what was actually applied).
        #
        # Wire contract on `raccoon/cmd/feature/bemf_enabled`:
        #   value=1 → BEMF on  → normal mode (speed_mode OFF)
        #   value=0 → BEMF off → speed mode  (speed_mode ON)
        # So engaging SpeedMode publishes 0, returning to normal publishes 1.
        target_value = 0 if self._enabled else 1
        loop = asyncio.get_running_loop()
        ack_future: asyncio.Future[int] = loop.create_future()
        transport = get_transport()

        def _on_ack(_channel: str, data: bytes) -> None:
            try:
                msg = scalar_i32_t.decode(data)
            except Exception:
                return
            if ack_future.done():
                return
            loop.call_soon_threadsafe(ack_future.set_result, int(msg.value))

        # stm32-data-reader subscribes to the command channel in reliable
        # mode, so the request has to be wrapped in a reliable envelope. The
        # ACK is published with retained=true but the reader also emits a
        # live publish on the plain channel, which is what a plain subscribe
        # picks up — do NOT request_retained here or we'd immediately fire on
        # the stale cached value from a previous run and skip the handshake.
        sub = transport.subscribe(_ACK_CHANNEL, _on_ack)
        try:
            cmd = scalar_i32_t()
            cmd.value = target_value
            transport.publish(_CMD_CHANNEL, cmd, reliable=True)

            deadline = loop.time() + self._timeout_s
            while not ack_future.done():
                if loop.time() >= deadline:
                    break
                await asyncio.sleep(0.01)

            if not ack_future.done():
                msg = (
                    f"SetSpeedMode: no ACK from stm32-data-reader on "
                    f"{_ACK_CHANNEL} within {self._timeout_s * 1000:.0f}ms "
                    f"after publishing {target_value} on {_CMD_CHANNEL}."
                )
                raise RuntimeError(msg)

            acked_value = ack_future.result()
        finally:
            transport.unsubscribe(sub)

        # Reconcile to the firmware's reported state. Speed mode is
        # "BEMF disabled", so the library flag is the inverse of the
        # ack'd BEMF-enabled value.
        set_speed_mode_enabled(acked_value == 0)


# The public `set_speed_mode(...)` factory and `SetSpeedModeBuilder`
# are generated by `tools/generate_step_builders.py` into the sibling
# `set_speed_mode_dsl.py` module from the `@dsl_step` decorator on
# `SetSpeedMode` above. Do not hand-write a factory here — it would
# collide with the codegen output.


__all__ = ["SetSpeedMode"]
