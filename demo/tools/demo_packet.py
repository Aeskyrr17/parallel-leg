from __future__ import annotations

import struct
from dataclasses import dataclass


USART_HOST_MAGIC = 0x31535544
USART_DEVICE_MAGIC = 0x31535552
USB_HOST_MAGIC = 0x31425344
USB_DEVICE_MAGIC = 0x31425352
CHECKSUM_SEED = 0xA5A51234

HOST_BODY = struct.Struct("<IIfB3xI")
HOST_PACKET = struct.Struct("<IIfB3xII")
DEVICE_BODY = struct.Struct("<IIBBBxfIII")
DEVICE_PACKET = struct.Struct("<IIBBBxfIIII")


@dataclass(frozen=True)
class HostPacket:
    magic: int
    seq: int
    value: float
    flag: bool
    counter: int
    checksum: int


@dataclass(frozen=True)
class DevicePacket:
    magic: int
    seq: int
    status: int
    ready: bool
    flag: bool
    value: float
    rx_count: int
    tx_count: int
    error_count: int
    checksum: int


def checksum_bytes(data: bytes) -> int:
    checksum = CHECKSUM_SEED
    for byte in data:
        checksum = (((checksum << 5) & 0xFFFFFFFF) ^ (checksum >> 2) ^ byte) & 0xFFFFFFFF
    return checksum


def build_host_packet(magic: int, seq: int, value: float, flag: bool, counter: int) -> bytes:
    body = HOST_BODY.pack(magic, seq, value, 1 if flag else 0, counter)
    checksum = checksum_bytes(body)
    return body + struct.pack("<I", checksum)


def parse_host_packet(data: bytes) -> HostPacket:
    if len(data) != HOST_PACKET.size:
        raise ValueError(f"host packet must be {HOST_PACKET.size} bytes, got {len(data)}")
    magic, seq, value, flag, counter, checksum = HOST_PACKET.unpack(data)
    expected = checksum_bytes(data[:-4])
    if checksum != expected:
        raise ValueError(f"bad host checksum: got 0x{checksum:08x}, expected 0x{expected:08x}")
    return HostPacket(magic, seq, value, bool(flag), counter, checksum)


def parse_device_packet(data: bytes, expected_magic: int, expected_seq: int | None = None) -> DevicePacket:
    if len(data) != DEVICE_PACKET.size:
        raise ValueError(f"device packet must be {DEVICE_PACKET.size} bytes, got {len(data)}")
    magic, seq, status, ready, flag, value, rx_count, tx_count, error_count, checksum = DEVICE_PACKET.unpack(data)
    expected_checksum = checksum_bytes(data[:-4])
    if checksum != expected_checksum:
        raise ValueError(f"bad device checksum: got 0x{checksum:08x}, expected 0x{expected_checksum:08x}")
    if magic != expected_magic:
        raise ValueError(f"bad magic: got 0x{magic:08x}, expected 0x{expected_magic:08x}")
    if expected_seq is not None and seq != expected_seq:
        raise ValueError(f"bad seq: got {seq}, expected {expected_seq}")
    return DevicePacket(magic, seq, status, bool(ready), bool(flag), value, rx_count, tx_count, error_count, checksum)


def self_test() -> None:
    packet = build_host_packet(USART_HOST_MAGIC, 7, 1.25, True, 9)
    parsed = parse_host_packet(packet)
    assert parsed.magic == USART_HOST_MAGIC
    assert parsed.seq == 7
    assert parsed.flag is True
    assert parsed.counter == 9


if __name__ == "__main__":
    self_test()
    print("PASS demo_packet self-test")
