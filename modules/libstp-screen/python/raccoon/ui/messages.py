from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import ClassVar, Self

_I64 = struct.Struct(">q")
_U32 = struct.Struct(">I")


def _encode_string(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return _U32.pack(len(encoded)) + encoded


def _decode_i64(data: bytes, offset: int) -> tuple[int, int]:
    end = offset + _I64.size
    if end > len(data):
        msg = "buffer ended while reading int64 field"
        raise ValueError(msg)
    return _I64.unpack_from(data, offset)[0], end


def _decode_string(data: bytes, offset: int) -> tuple[str, int]:
    length_end = offset + _U32.size
    if length_end > len(data):
        msg = "buffer ended while reading string length"
        raise ValueError(msg)
    length = _U32.unpack_from(data, offset)[0]
    end = length_end + length
    if end > len(data):
        msg = "buffer ended while reading string bytes"
        raise ValueError(msg)
    return data[length_end:end].decode("utf-8"), end


@dataclass(slots=True)
class ScreenRender:
    """Raw `raccoon/screen_render` wire message.

    Wire format, all big-endian and tightly packed:
    - bytes `0..7`: `timestamp` as signed int64
    - bytes `8..11`: `screen_name` byte length as unsigned uint32
    - next `N` bytes: UTF-8 `screen_name`
    - next `4` bytes: `entries` byte length as unsigned uint32
    - next `M` bytes: UTF-8 `entries`
    """

    timestamp: int
    screen_name: str
    entries: str

    _FIXED_PREFIX: ClassVar[struct.Struct] = struct.Struct(">q")

    def encode(self) -> bytes:
        return b"".join(
            (
                self._FIXED_PREFIX.pack(self.timestamp),
                _encode_string(self.screen_name),
                _encode_string(self.entries),
            )
        )

    @classmethod
    def decode(cls, data: bytes) -> Self:
        timestamp, offset = _decode_i64(data, 0)
        screen_name, offset = _decode_string(data, offset)
        entries, offset = _decode_string(data, offset)
        if offset != len(data):
            msg = "trailing bytes after ScreenRender payload"
            raise ValueError(msg)
        return cls(timestamp=timestamp, screen_name=screen_name, entries=entries)


@dataclass(slots=True)
class ScreenRenderAnswer:
    """Raw `raccoon/screen_render/answer` wire message.

    Wire format, all big-endian and tightly packed:
    - bytes `0..7`: `timestamp` as signed int64
    - bytes `8..11`: `screen_name` byte length as unsigned uint32
    - next `N` bytes: UTF-8 `screen_name`
    - next `4` bytes: `value` byte length as unsigned uint32
    - next `P` bytes: UTF-8 `value`
    - next `4` bytes: `reason` byte length as unsigned uint32
    - next `Q` bytes: UTF-8 `reason`
    """

    timestamp: int
    screen_name: str
    value: str
    reason: str

    _FIXED_PREFIX: ClassVar[struct.Struct] = struct.Struct(">q")

    def encode(self) -> bytes:
        return b"".join(
            (
                self._FIXED_PREFIX.pack(self.timestamp),
                _encode_string(self.screen_name),
                _encode_string(self.value),
                _encode_string(self.reason),
            )
        )

    @classmethod
    def decode(cls, data: bytes) -> Self:
        timestamp, offset = _decode_i64(data, 0)
        screen_name, offset = _decode_string(data, offset)
        value, offset = _decode_string(data, offset)
        reason, offset = _decode_string(data, offset)
        if offset != len(data):
            msg = "trailing bytes after ScreenRenderAnswer payload"
            raise ValueError(msg)
        return cls(timestamp=timestamp, screen_name=screen_name, value=value, reason=reason)
