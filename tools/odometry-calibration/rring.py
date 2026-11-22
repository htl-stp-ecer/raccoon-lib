"""The latest value of transport channels, for the odometry calibration tools.

This runs ON THE PI (the transport lives there). `Ring(channel)` subscribes to a channel
through `raccoon_transport`, and `.latest()` returns the payload of the most recent frame
(None if nothing was published yet). Decode it with the message classes from
`raccoon_transport.types.raccoon`.
"""

from __future__ import annotations

import raccoon_transport as rt


class Ring:
    """Keeps the most recent frame of one channel."""

    def __init__(self, channel: str):
        self.channel = channel
        self._latest: bytes | None = None
        self._transport = rt.get_transport()
        self._subscription = self._transport.subscribe(channel, self._on_frame)

    def _on_frame(self, _channel: str, data: bytes) -> None:
        self._latest = bytes(data)

    def latest(self) -> bytes | None:
        return self._latest

    def close(self) -> None:
        self._transport.unsubscribe(self._subscription)


if __name__ == "__main__":
    # Quick health/odometry dump.
    import time

    from raccoon_transport import channels
    from raccoon_transport.types.raccoon import scalar_f_t, scalar_i32_t, string_t

    calib = channels.hardware.calib_board
    odometry = channels.hardware.odometry.state
    SPECS = [
        (calib.state.BOARD, string_t),
        (calib.state.PAA, string_t),
        (calib.paa.state.CAL_VALID, scalar_i32_t),
        (calib.paa.state.SQUAL, scalar_i32_t),
        (calib.odom.state.POS_X, scalar_f_t),  # cm  (ground truth)
        (calib.odom.state.POS_Y, scalar_f_t),  # cm
        (calib.odom.state.HEADING, scalar_f_t),  # deg
        (odometry.POS_X, scalar_f_t),  # m   (internal STM32)
        (odometry.POS_Y, scalar_f_t),  # m
        (odometry.HEADING, scalar_f_t),  # rad
    ]
    rings = [(chan, typ, Ring(chan)) for chan, typ in SPECS]
    time.sleep(2.0)  # a channel that is not retained shows its first frame after this
    for chan, typ, ring in rings:
        raw = ring.latest()
        val = typ.decode(raw).value if raw is not None else "<no frame>"
        print(f"{chan:46s} = {val}")
        ring.close()
