"""Direct reader for raccoon_ring SHM channels.

Why this exists
---------------
The C++ stack publishes everything over `raccoon_ring` — a SHM ring buffer,
one file per channel under `/dev/shm/raccoon_ring_<channel>` (`/` -> `_2F`).
The Python `raccoon_transport` package (0.1.63) is still the in-process `_Memq`
stub and CANNOT read that cross-process bus, so we read the SHM files directly.

This must run ON THE PI (the SHM lives there). Decoding reuses the message
classes from `raccoon_transport.types.raccoon` (those work fine; only the
transport/subscribe layer is stubbed).

Ring format (raccoon-transport/cpp/src/raccoon_ring.c), little-endian:
  header 256 B: magic@0 (0x52435242), version@4, slot_count@8, slot_size@12,
                max_payload@16, producer_seq@32 (u64, 0 = no data yet)
  slots @256, stride = slot_size; slot hdr 16 B: seq@0 (u64, 0 = being written),
                len@8 (u32); payload @ slot+16, `len` bytes.
"""

from __future__ import annotations

import mmap
import os
import struct

RRB_MAGIC = 0x52435242
HDR = 256
SLOT_HDR = 16


def path(channel: str) -> str:
    return "/dev/shm/raccoon_ring_" + channel.replace("/", "_2F")


class Ring:
    """Read-only attach to one raccoon_ring channel; .latest() returns the
    most recent payload bytes (None if the producer hasn't published yet)."""

    def __init__(self, channel: str):
        self.channel = channel
        self.fd = os.open(path(channel), os.O_RDONLY)
        size = os.fstat(self.fd).st_size
        self.mm = mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ)
        magic, _ver, self.slot_count, self.slot_size, self.max_payload = struct.unpack_from(
            "<IIIII", self.mm, 0
        )
        if magic != RRB_MAGIC:
            msg = f"bad ring magic {magic:#x} on {channel}"
            raise ValueError(msg)

    def producer_seq(self) -> int:
        return struct.unpack_from("<Q", self.mm, 32)[0]

    def latest(self):
        for _ in range(8):  # SeqLock retry loop
            pseq = self.producer_seq()
            if pseq == 0:
                return None
            idx = (pseq - 1) % self.slot_count
            off = HDR + idx * self.slot_size
            s0 = struct.unpack_from("<Q", self.mm, off)[0]
            if s0 == 0:
                continue
            ln = struct.unpack_from("<I", self.mm, off + 8)[0]
            if ln > self.max_payload:
                continue
            data = self.mm[off + SLOT_HDR : off + SLOT_HDR + ln]
            if struct.unpack_from("<Q", self.mm, off)[0] == s0:  # slot not overwritten mid-read
                return bytes(data)
        return None

    def close(self):
        self.mm.close()
        os.close(self.fd)


if __name__ == "__main__":
    # Quick health/odometry dump.
    from raccoon_transport.types.raccoon import scalar_f_t, scalar_i32_t, string_t

    SPECS = [
        ("raccoon/calib_board/status/board", string_t),
        ("raccoon/calib_board/status/paa", string_t),
        ("raccoon/calib_board/paa/cal/valid", scalar_i32_t),
        ("raccoon/calib_board/paa/squal", scalar_i32_t),
        ("raccoon/calib_board/odom/pos_x", scalar_f_t),  # cm  (ground truth)
        ("raccoon/calib_board/odom/pos_y", scalar_f_t),  # cm
        ("raccoon/calib_board/odom/heading", scalar_f_t),  # deg
        ("raccoon/odometry/pos_x", scalar_f_t),  # m   (internal STM32)
        ("raccoon/odometry/pos_y", scalar_f_t),  # m
        ("raccoon/odometry/heading", scalar_f_t),  # rad
    ]
    for chan, typ in SPECS:
        try:
            r = Ring(chan)
            raw = r.latest()
            val = typ.decode(raw).value if raw is not None else "<no frame>"
            print(f"{chan:42s} = {val}")
            r.close()
        except Exception as e:
            print(f"{chan:42s} ERR {e}")
